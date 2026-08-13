/* USER CODE BEGIN Header */
    /**
    ******************************************************************************
    * @file           : main.c
    * @brief          : Main program body
    ******************************************************************************
    * @attention
    *
    * Copyright (c) 2026 STMicroelectronics.
    * All rights reserved.
    *
    * This software is licensed under terms that can be found in the LICENSE file
    * in the root directory of this software component.
    * If no LICENSE file comes with this software, it is provided AS-IS.
    *
    ******************************************************************************
    */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

    /* NEMA17: 1.8 degrees/full step = 200 full steps/revolution */
    #define MOTOR_FULL_STEPS_PER_REV   200U

    /* TMC2209 standalone microstepping used by this rig */
    #define MOTOR_MICROSTEPS           8U

    #define MOTOR_PULSES_PER_REV \
        (MOTOR_FULL_STEPS_PER_REV * MOTOR_MICROSTEPS)

    /*
     * Timer setup used by this version:
     *
     * Motor 1 -> TIM3_CH1
     *   Timer clock = 72 MHz
     *   Prescaler   = 71
     *   Period      = 4999
     *   Pulse       = 2500
     *   STEP rate   = 200 Hz
     *
     * Motor 2 -> TIM2_CH1
     *   Timer clock = 72 MHz
     *   Prescaler   = 71
     *   Runtime STEP rate uses a ramp:
     *       50 Hz -> 200 Hz -> 50 Hz
     *   Pulse width remains 10 us.
     *
     * TIM2's ARR is changed while the motor is moving, so its CubeMX
     * starting Period is not used as the final motion speed.
     */
    #define M1_STEP_PWM_PULSE_COUNT    2500U
    #define M2_STEP_PWM_PULSE_COUNT    10U

    /* TIM2 counter frequency after 72 MHz / (71 + 1). */
    #define M2_TIMER_COUNTER_HZ        1000000U

    /*
     * Conservative Motor 2 acceleration/deceleration profile.
     * 1.8 degree motor + 1/8 microstepping:
     *   50 Hz  = 11.25 deg/s
     *   200 Hz = 45.00 deg/s
     */
    #define M2_START_FREQ_HZ           50U
    #define M2_MAX_FREQ_HZ             200U

    /* First 20% accelerates, middle 60% cruises, last 20% decelerates. */
    #define M2_RAMP_DIVISOR            5U

    /*
     * Verified command convention:
     *   direction_sign = +1 -> clockwise
     *   direction_sign = -1 -> counterclockwise
     *
     * Because the two motors may be mounted in opposite orientations,
     * each motor has an independent GPIO direction mapping. If one motor
     * turns the wrong way, swap only that motor's POSITIVE/NEGATIVE values.
     */
    #define M1_DIR_POSITIVE            GPIO_PIN_SET
    #define M1_DIR_NEGATIVE            GPIO_PIN_RESET

    #define M2_DIR_POSITIVE            GPIO_PIN_SET
    #define M2_DIR_NEGATIVE            GPIO_PIN_RESET

    /* One relative command may request 0 to 180 degrees */
    #define MOTOR_MIN_MOVE_DEG         0.0f
    #define MOTOR_MAX_MOVE_DEG         180.0f

    /* Wait after all commanded motion before reporting DONE */
    #define MOTOR_SETTLE_TIME_MS       1000U

    /* UART line-command buffer */
    #define UART_RX_BUFFER_SIZE        96U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
    /* Open-loop cumulative software positions; positive and negative allowed. */
    static int32_t m1_position_pulses = 0;
    static int32_t m2_position_pulses = 0;

    static uint8_t m1_enabled = 0U;
    static uint8_t m2_enabled = 0U;

    static char uart_rx_buffer[UART_RX_BUFFER_SIZE];
    static uint32_t uart_rx_index = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

    static uint32_t Stepper_AngleToPulses(float angle_deg);
    static float Stepper_PulsesToAngle(int32_t pulses);

    static void Motor2_SetStepFrequency(uint32_t frequency_hz);
    static uint32_t Motor2_GetRampFrequency(uint32_t pulse_number,
                                            uint32_t total_pulses);

    static void Stepper_MovePulsePair(uint32_t m1_pulses,
                                      GPIO_PinState m1_direction,
                                      uint32_t m2_pulses,
                                      GPIO_PinState m2_direction);

    static uint32_t Motor_MoveRelative(uint8_t motor_id,
                                       float move_angle_deg,
                                       int32_t direction_sign);

    static void Motors_MoveRelative(float m1_angle_deg,
                                    int32_t m1_direction_sign,
                                    float m2_angle_deg,
                                    int32_t m2_direction_sign,
                                    uint32_t *m1_pulses_sent,
                                    uint32_t *m2_pulses_sent);

    static uint32_t RunSingleMotorTest(uint8_t motor_id,
                                       float move_angle_deg,
                                       int32_t direction_sign);

    static void RunDualMotorTest(float m1_angle_deg,
                                 int32_t m1_direction_sign,
                                 float m2_angle_deg,
                                 int32_t m2_direction_sign,
                                 uint32_t *m1_pulses_sent,
                                 uint32_t *m2_pulses_sent);

    static void UART_SendString(const char *text);
    static void UART_SendStatus(void);
    static void UART_SendSingleAck(uint8_t motor_id,
                                   int32_t direction_sign,
                                   float requested_angle_deg,
                                   uint32_t pulses);
    static void UART_SendSingleDone(uint8_t motor_id,
                                    int32_t direction_sign,
                                    float requested_angle_deg,
                                    uint32_t pulses,
                                    int32_t cumulative_position_pulses);
    static void UART_SendDualAck(int32_t m1_direction_sign,
                                 float m1_angle_deg,
                                 uint32_t m1_pulses,
                                 int32_t m2_direction_sign,
                                 float m2_angle_deg,
                                 uint32_t m2_pulses);
    static void UART_SendDualDone(int32_t m1_direction_sign,
                                  float m1_angle_deg,
                                  uint32_t m1_pulses,
                                  int32_t m2_direction_sign,
                                  float m2_angle_deg,
                                  uint32_t m2_pulses);

    static void UART_ProcessCommand(char *command);
    static void UART_Poll(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

    static int32_t RoundFloatToInt32(float value)
    {
        if (value >= 0.0f)
        {
            return (int32_t)(value + 0.5f);
        }

        return (int32_t)(value - 0.5f);
    }


    static uint8_t ParseInt32Token(const char *token, int32_t *value)
    {
        char *end_pointer;
        long parsed_value;

        if ((token == NULL) || (value == NULL) || (token[0] == '\0'))
        {
            return 0U;
        }

        end_pointer = NULL;
        parsed_value = strtol(token, &end_pointer, 10);

        if ((end_pointer == token) ||
            (end_pointer == NULL) ||
            (*end_pointer != '\0'))
        {
            return 0U;
        }

        *value = (int32_t)parsed_value;
        return 1U;
    }


    static uint8_t ParseFloatToken(const char *token, float *value)
    {
        char *end_pointer;
        float parsed_value;

        if ((token == NULL) || (value == NULL) || (token[0] == '\0'))
        {
            return 0U;
        }

        end_pointer = NULL;
        parsed_value = strtof(token, &end_pointer);

        if ((end_pointer == token) ||
            (end_pointer == NULL) ||
            (*end_pointer != '\0'))
        {
            return 0U;
        }

        *value = parsed_value;
        return 1U;
    }


    static GPIO_PinState Motor_GetDirectionLevel(uint8_t motor_id,
                                                  int32_t direction_sign)
    {
        if (motor_id == 1U)
        {
            return (direction_sign > 0)
                ? M1_DIR_POSITIVE
                : M1_DIR_NEGATIVE;
        }

        return (direction_sign > 0)
            ? M2_DIR_POSITIVE
            : M2_DIR_NEGATIVE;
    }


    /**
    * @brief Convert a positive angle magnitude to the nearest pulse count.
    *
    * 1.8 degree motor + 1/8 microstepping:
    *   1600 pulses/revolution
    *   0.225 degree/pulse
    */
    static uint32_t Stepper_AngleToPulses(float angle_deg)
    {
        float pulse_count;

        if (angle_deg <= 0.0f)
        {
            return 0U;
        }

        pulse_count =
            angle_deg * (float)MOTOR_PULSES_PER_REV / 360.0f;

        return (uint32_t)(pulse_count + 0.5f);
    }


    /**
    * @brief Convert cumulative software pulses back to degrees.
    */
    static float Stepper_PulsesToAngle(int32_t pulses)
    {
        return (float)pulses * 360.0f /
               (float)MOTOR_PULSES_PER_REV;
    }


    /**
    * @brief Set Motor 2 STEP frequency by changing TIM2 ARR.
    *
    * TIM2 runs at 1 MHz after the prescaler:
    *   ARR = (1,000,000 / frequency_hz) - 1
    *
    * The STEP high pulse remains 10 us.
    */
    static void Motor2_SetStepFrequency(uint32_t frequency_hz)
    {
        uint32_t period;

        if (frequency_hz < M2_START_FREQ_HZ)
        {
            frequency_hz = M2_START_FREQ_HZ;
        }

        if (frequency_hz > M2_MAX_FREQ_HZ)
        {
            frequency_hz = M2_MAX_FREQ_HZ;
        }

        period = (M2_TIMER_COUNTER_HZ / frequency_hz) - 1U;

        __HAL_TIM_SET_AUTORELOAD(&htim2, period);
        __HAL_TIM_SET_COMPARE(&htim2,
                              TIM_CHANNEL_1,
                              M2_STEP_PWM_PULSE_COUNT);
    }


    /**
    * @brief Return Motor 2 STEP frequency for the current pulse.
    *
    * Profile:
    *   first 20%  : 50 -> 200 Hz acceleration
    *   middle 60% : 200 Hz constant speed
    *   last 20%   : 200 -> 50 Hz deceleration
    *
    * pulse_number is 1-based: 1 ... total_pulses.
    */
    static uint32_t Motor2_GetRampFrequency(uint32_t pulse_number,
                                            uint32_t total_pulses)
    {
        uint32_t ramp_pulses;
        uint32_t frequency_span;
        uint32_t pulses_remaining;

        if ((total_pulses <= 1U) || (pulse_number <= 1U))
        {
            return M2_START_FREQ_HZ;
        }

        if (pulse_number > total_pulses)
        {
            pulse_number = total_pulses;
        }

        ramp_pulses = total_pulses / M2_RAMP_DIVISOR;

        /*
         * Very short moves stay slow instead of attempting an aggressive
         * acceleration profile with only one or two ramp pulses.
         */
        if (ramp_pulses < 2U)
        {
            return M2_START_FREQ_HZ;
        }

        frequency_span = M2_MAX_FREQ_HZ - M2_START_FREQ_HZ;

        /* Acceleration region */
        if (pulse_number <= ramp_pulses)
        {
            return M2_START_FREQ_HZ +
                   (frequency_span * (pulse_number - 1U)) /
                   (ramp_pulses - 1U);
        }

        /* Deceleration region */
        if (pulse_number > (total_pulses - ramp_pulses))
        {
            pulses_remaining = total_pulses - pulse_number;

            return M2_START_FREQ_HZ +
                   (frequency_span * pulses_remaining) /
                   (ramp_pulses - 1U);
        }

        /* Constant-speed region */
        return M2_MAX_FREQ_HZ;
    }


    static void StopAndResetTimer(TIM_HandleTypeDef *timer,
                                  uint32_t channel)
    {
        (void)HAL_TIM_PWM_Stop(timer, channel);
        __HAL_TIM_SET_COUNTER(timer, 0U);
        __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
    }


    /**
    * @brief Generate exact STEP counts for one or both motors.
    *
    * Motor 1: TIM3_CH1 -> PA6
    *   Keeps the existing fixed TIM3 rate from CubeMX.
    *
    * Motor 2: TIM2_CH1 -> PA0
    *   Uses a conservative acceleration/deceleration ramp:
    *       50 Hz -> 200 Hz -> 50 Hz
    *   with approximately 20% acceleration, 60% cruise, 20% deceleration.
    *
    * When both pulse counts are nonzero, both motors start together. Each
    * motor stops after its own requested pulse count.
    *
    * This implementation polls timer update flags; timer interrupts are not
    * required by this function.
    */
    static void Stepper_MovePulsePair(uint32_t m1_pulses,
                                      GPIO_PinState m1_direction,
                                      uint32_t m2_pulses,
                                      GPIO_PinState m2_direction)
    {
        uint32_t m1_count;
        uint32_t m2_count;
        uint32_t m2_frequency;
        uint8_t m1_done;
        uint8_t m2_done;

        if ((m1_pulses == 0U) && (m2_pulses == 0U))
        {
            return;
        }

        if (m1_pulses > 0U)
        {
            HAL_GPIO_WritePin(M1_DIR_GPIO_Port,
                              M1_DIR_Pin,
                              m1_direction);
        }

        if (m2_pulses > 0U)
        {
            HAL_GPIO_WritePin(M2_DIR_GPIO_Port,
                              M2_DIR_Pin,
                              m2_direction);
        }

        /* Give DIR plenty of setup time before the first STEP edge. */
        HAL_Delay(2);

        StopAndResetTimer(&htim3, TIM_CHANNEL_1);
        StopAndResetTimer(&htim2, TIM_CHANNEL_1);

        m1_count = 0U;
        m2_count = 0U;
        m1_done = (m1_pulses == 0U) ? 1U : 0U;
        m2_done = (m2_pulses == 0U) ? 1U : 0U;

        /*
         * Motor 1 keeps its existing fixed TIM3 speed.
         * Starting PWM creates the first STEP rising edge.
         */
        if (m1_done == 0U)
        {
            if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
            {
                Error_Handler();
            }

            m1_count = 1U;
        }

        /*
         * Motor 2 always starts slowly at 50 Hz.
         * TIM2 ARR changes after STEP periods to create the ramp.
         */
        if (m2_done == 0U)
        {
            Motor2_SetStepFrequency(M2_START_FREQ_HZ);
            __HAL_TIM_SET_COUNTER(&htim2, 0U);
            __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

            if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
            {
                Error_Handler();
            }

            m2_count = 1U;
        }

        while ((m1_done == 0U) || (m2_done == 0U))
        {
            if (m1_done == 0U)
            {
                if (m1_count >= m1_pulses)
                {
                    /*
                     * Allow Motor 1's final high pulse to finish before
                     * stopping TIM3.
                     */
                    if (__HAL_TIM_GET_COUNTER(&htim3) >=
                        M1_STEP_PWM_PULSE_COUNT)
                    {
                        StopAndResetTimer(&htim3, TIM_CHANNEL_1);
                        m1_done = 1U;
                    }
                }
                else if (__HAL_TIM_GET_FLAG(&htim3,
                                            TIM_FLAG_UPDATE) != RESET)
                {
                    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
                    m1_count++;
                }
            }

            if (m2_done == 0U)
            {
                if (m2_count >= m2_pulses)
                {
                    /*
                     * Motor 2 reaches the final pulse at the slow end of the
                     * deceleration ramp. Let the 10 us STEP high pulse finish,
                     * then stop STEP pulses.
                     *
                     * M2_EN is NOT disabled here, so the TMC2209 continues
                     * holding the final position after the move.
                     */
                    if (__HAL_TIM_GET_COUNTER(&htim2) >=
                        M2_STEP_PWM_PULSE_COUNT)
                    {
                        StopAndResetTimer(&htim2, TIM_CHANNEL_1);
                        m2_done = 1U;
                    }
                }
                else if (__HAL_TIM_GET_FLAG(&htim2,
                                            TIM_FLAG_UPDATE) != RESET)
                {
                    /*
                     * The timer update event corresponds to the next STEP
                     * period. Count the pulse and select the frequency for
                     * the following interval.
                     */
                    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
                    m2_count++;

                    m2_frequency =
                        Motor2_GetRampFrequency(m2_count,
                                                m2_pulses);

                    Motor2_SetStepFrequency(m2_frequency);
                }
            }
        }
    }

    /**
    * @brief Move one selected motor by a relative angle.
    *
    * motor_id       = 1 or 2
    * direction_sign = +1 clockwise, -1 counterclockwise
    *
    * @return Exact number of STEP pulses sent.
    */
    static uint32_t Motor_MoveRelative(uint8_t motor_id,
                                       float move_angle_deg,
                                       int32_t direction_sign)
    {
        uint32_t pulses_to_move;
        GPIO_PinState direction;

        pulses_to_move = Stepper_AngleToPulses(move_angle_deg);
        direction = Motor_GetDirectionLevel(motor_id, direction_sign);

        if (motor_id == 1U)
        {
            Stepper_MovePulsePair(pulses_to_move,
                                  direction,
                                  0U,
                                  M2_DIR_POSITIVE);

            m1_position_pulses +=
                direction_sign * (int32_t)pulses_to_move;
        }
        else
        {
            Stepper_MovePulsePair(0U,
                                  M1_DIR_POSITIVE,
                                  pulses_to_move,
                                  direction);

            m2_position_pulses +=
                direction_sign * (int32_t)pulses_to_move;
        }

        return pulses_to_move;
    }


    /**
    * @brief Move both motors simultaneously.
    */
    static void Motors_MoveRelative(float m1_angle_deg,
                                    int32_t m1_direction_sign,
                                    float m2_angle_deg,
                                    int32_t m2_direction_sign,
                                    uint32_t *m1_pulses_sent,
                                    uint32_t *m2_pulses_sent)
    {
        uint32_t m1_pulses;
        uint32_t m2_pulses;
        GPIO_PinState m1_direction;
        GPIO_PinState m2_direction;

        m1_pulses = Stepper_AngleToPulses(m1_angle_deg);
        m2_pulses = Stepper_AngleToPulses(m2_angle_deg);

        m1_direction =
            Motor_GetDirectionLevel(1U, m1_direction_sign);
        m2_direction =
            Motor_GetDirectionLevel(2U, m2_direction_sign);

        Stepper_MovePulsePair(m1_pulses,
                              m1_direction,
                              m2_pulses,
                              m2_direction);

        m1_position_pulses +=
            m1_direction_sign * (int32_t)m1_pulses;
        m2_position_pulses +=
            m2_direction_sign * (int32_t)m2_pulses;

        if (m1_pulses_sent != NULL)
        {
            *m1_pulses_sent = m1_pulses;
        }

        if (m2_pulses_sent != NULL)
        {
            *m2_pulses_sent = m2_pulses;
        }
    }


    static uint32_t RunSingleMotorTest(uint8_t motor_id,
                                       float move_angle_deg,
                                       int32_t direction_sign)
    {
        uint32_t pulses_sent;

        HAL_GPIO_WritePin(MOTOR_MOVING_GPIO_Port,
                          MOTOR_MOVING_Pin,
                          GPIO_PIN_SET);

        HAL_Delay(20);

        pulses_sent = Motor_MoveRelative(motor_id,
                                         move_angle_deg,
                                         direction_sign);

        HAL_Delay(20);

        HAL_GPIO_WritePin(MOTOR_MOVING_GPIO_Port,
                          MOTOR_MOVING_Pin,
                          GPIO_PIN_RESET);

        HAL_Delay(MOTOR_SETTLE_TIME_MS);

        return pulses_sent;
    }


    static void RunDualMotorTest(float m1_angle_deg,
                                 int32_t m1_direction_sign,
                                 float m2_angle_deg,
                                 int32_t m2_direction_sign,
                                 uint32_t *m1_pulses_sent,
                                 uint32_t *m2_pulses_sent)
    {
        HAL_GPIO_WritePin(MOTOR_MOVING_GPIO_Port,
                          MOTOR_MOVING_Pin,
                          GPIO_PIN_SET);

        HAL_Delay(20);

        Motors_MoveRelative(m1_angle_deg,
                            m1_direction_sign,
                            m2_angle_deg,
                            m2_direction_sign,
                            m1_pulses_sent,
                            m2_pulses_sent);

        HAL_Delay(20);

        HAL_GPIO_WritePin(MOTOR_MOVING_GPIO_Port,
                          MOTOR_MOVING_Pin,
                          GPIO_PIN_RESET);

        HAL_Delay(MOTOR_SETTLE_TIME_MS);
    }


    static void UART_SendString(const char *text)
    {
        if (text == NULL)
        {
            return;
        }

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)text,
                          (uint16_t)strlen(text),
                          HAL_MAX_DELAY);
    }


    static void SplitAngleForMessage(float angle_deg,
                                     const char **sign_text,
                                     int32_t *whole,
                                     int32_t *fraction)
    {
        int32_t angle_cdeg;
        int32_t absolute_cdeg;

        angle_cdeg = RoundFloatToInt32(angle_deg * 100.0f);
        absolute_cdeg = (angle_cdeg < 0) ? -angle_cdeg : angle_cdeg;

        if (sign_text != NULL)
        {
            *sign_text = (angle_cdeg < 0) ? "-" : "";
        }

        if (whole != NULL)
        {
            *whole = absolute_cdeg / 100;
        }

        if (fraction != NULL)
        {
            *fraction = absolute_cdeg % 100;
        }
    }


    static void UART_SendStatus(void)
    {
        char tx_buffer[128];
        float m1_angle_deg;
        float m2_angle_deg;
        const char *m1_sign;
        const char *m2_sign;
        int32_t m1_whole;
        int32_t m1_fraction;
        int32_t m2_whole;
        int32_t m2_fraction;

        m1_angle_deg = Stepper_PulsesToAngle(m1_position_pulses);
        m2_angle_deg = Stepper_PulsesToAngle(m2_position_pulses);

        SplitAngleForMessage(m1_angle_deg,
                             &m1_sign,
                             &m1_whole,
                             &m1_fraction);
        SplitAngleForMessage(m2_angle_deg,
                             &m2_sign,
                             &m2_whole,
                             &m2_fraction);

        (void)snprintf(tx_buffer,
                       sizeof(tx_buffer),
                       "STATUS,M1,%s,%s%ld.%02ld,M2,%s,%s%ld.%02ld\r\n",
                       (m1_enabled != 0U) ? "ENABLED" : "DISABLED",
                       m1_sign,
                       (long)m1_whole,
                       (long)m1_fraction,
                       (m2_enabled != 0U) ? "ENABLED" : "DISABLED",
                       m2_sign,
                       (long)m2_whole,
                       (long)m2_fraction);

        UART_SendString(tx_buffer);
    }


    static void UART_SendSingleAck(uint8_t motor_id,
                                   int32_t direction_sign,
                                   float requested_angle_deg,
                                   uint32_t pulses)
    {
        char tx_buffer[96];
        const char *sign_text;
        int32_t whole;
        int32_t fraction;

        SplitAngleForMessage(requested_angle_deg,
                             &sign_text,
                             &whole,
                             &fraction);

        (void)sign_text;

        (void)snprintf(tx_buffer,
                       sizeof(tx_buffer),
                       "ACK,MOVE,%u,%+ld,%ld.%02ld,%lu\r\n",
                       (unsigned int)motor_id,
                       (long)direction_sign,
                       (long)whole,
                       (long)fraction,
                       (unsigned long)pulses);

        UART_SendString(tx_buffer);
    }


    static void UART_SendSingleDone(uint8_t motor_id,
                                    int32_t direction_sign,
                                    float requested_angle_deg,
                                    uint32_t pulses,
                                    int32_t cumulative_position_pulses)
    {
        char tx_buffer[112];
        const char *sign_text;
        int32_t whole;
        int32_t fraction;

        SplitAngleForMessage(requested_angle_deg,
                             &sign_text,
                             &whole,
                             &fraction);

        (void)sign_text;

        (void)snprintf(tx_buffer,
                       sizeof(tx_buffer),
                       "DONE,MOVE,%u,%+ld,%ld.%02ld,%lu,%ld\r\n",
                       (unsigned int)motor_id,
                       (long)direction_sign,
                       (long)whole,
                       (long)fraction,
                       (unsigned long)pulses,
                       (long)cumulative_position_pulses);

        UART_SendString(tx_buffer);
    }


    static void UART_SendDualAck(int32_t m1_direction_sign,
                                 float m1_angle_deg,
                                 uint32_t m1_pulses,
                                 int32_t m2_direction_sign,
                                 float m2_angle_deg,
                                 uint32_t m2_pulses)
    {
        char tx_buffer[144];
        const char *unused_sign;
        int32_t m1_whole;
        int32_t m1_fraction;
        int32_t m2_whole;
        int32_t m2_fraction;

        SplitAngleForMessage(m1_angle_deg,
                             &unused_sign,
                             &m1_whole,
                             &m1_fraction);
        SplitAngleForMessage(m2_angle_deg,
                             &unused_sign,
                             &m2_whole,
                             &m2_fraction);

        (void)snprintf(
            tx_buffer,
            sizeof(tx_buffer),
            "ACK,MOVE2,%+ld,%ld.%02ld,%lu,%+ld,%ld.%02ld,%lu\r\n",
            (long)m1_direction_sign,
            (long)m1_whole,
            (long)m1_fraction,
            (unsigned long)m1_pulses,
            (long)m2_direction_sign,
            (long)m2_whole,
            (long)m2_fraction,
            (unsigned long)m2_pulses);

        UART_SendString(tx_buffer);
    }


    static void UART_SendDualDone(int32_t m1_direction_sign,
                                  float m1_angle_deg,
                                  uint32_t m1_pulses,
                                  int32_t m2_direction_sign,
                                  float m2_angle_deg,
                                  uint32_t m2_pulses)
    {
        char tx_buffer[176];
        const char *unused_sign;
        int32_t m1_whole;
        int32_t m1_fraction;
        int32_t m2_whole;
        int32_t m2_fraction;

        SplitAngleForMessage(m1_angle_deg,
                             &unused_sign,
                             &m1_whole,
                             &m1_fraction);
        SplitAngleForMessage(m2_angle_deg,
                             &unused_sign,
                             &m2_whole,
                             &m2_fraction);

        (void)snprintf(
            tx_buffer,
            sizeof(tx_buffer),
            "DONE,MOVE2,%+ld,%ld.%02ld,%lu,%ld,%+ld,%ld.%02ld,%lu,%ld\r\n",
            (long)m1_direction_sign,
            (long)m1_whole,
            (long)m1_fraction,
            (unsigned long)m1_pulses,
            (long)m1_position_pulses,
            (long)m2_direction_sign,
            (long)m2_whole,
            (long)m2_fraction,
            (unsigned long)m2_pulses,
            (long)m2_position_pulses);

        UART_SendString(tx_buffer);
    }


    static uint8_t ParseMotorSelector(const char *token,
                                      uint8_t *motor_id,
                                      uint8_t allow_all)
    {
        if ((token == NULL) || (motor_id == NULL))
        {
            return 0U;
        }

        if (strcmp(token, "1") == 0)
        {
            *motor_id = 1U;
            return 1U;
        }

        if (strcmp(token, "2") == 0)
        {
            *motor_id = 2U;
            return 1U;
        }

        if ((allow_all != 0U) &&
            (strcmp(token, "ALL") == 0))
        {
            *motor_id = 0U;
            return 1U;
        }

        return 0U;
    }


    static void EnableSelectedMotor(uint8_t motor_id)
    {
        if ((motor_id == 0U) || (motor_id == 1U))
        {
            HAL_GPIO_WritePin(M1_EN_GPIO_Port,
                              M1_EN_Pin,
                              GPIO_PIN_RESET);
            m1_enabled = 1U;
        }

        if ((motor_id == 0U) || (motor_id == 2U))
        {
            HAL_GPIO_WritePin(M2_EN_GPIO_Port,
                              M2_EN_Pin,
                              GPIO_PIN_RESET);
            m2_enabled = 1U;
        }
    }


    static void DisableSelectedMotor(uint8_t motor_id)
    {
        if ((motor_id == 0U) || (motor_id == 1U))
        {
            StopAndResetTimer(&htim3, TIM_CHANNEL_1);

            HAL_GPIO_WritePin(M1_EN_GPIO_Port,
                              M1_EN_Pin,
                              GPIO_PIN_SET);
            m1_enabled = 0U;
        }

        if ((motor_id == 0U) || (motor_id == 2U))
        {
            StopAndResetTimer(&htim2, TIM_CHANNEL_1);

            HAL_GPIO_WritePin(M2_EN_GPIO_Port,
                              M2_EN_Pin,
                              GPIO_PIN_SET);
            m2_enabled = 0U;
        }

        HAL_GPIO_WritePin(MOTOR_MOVING_GPIO_Port,
                          MOTOR_MOVING_Pin,
                          GPIO_PIN_RESET);
    }


    static void ZeroSelectedMotor(uint8_t motor_id)
    {
        if ((motor_id == 0U) || (motor_id == 1U))
        {
            m1_position_pulses = 0;
        }

        if ((motor_id == 0U) || (motor_id == 2U))
        {
            m2_position_pulses = 0;
        }
    }


    /**
    * @brief Parse and execute one command received from the PC.
    *
    * Supported commands (terminate each command with \n):
    *
    *   PING
    *   ENABLE              (enable both)
    *   ENABLE,1
    *   ENABLE,2
    *   ENABLE,ALL
    *
    *   DISABLE             (disable both)
    *   DISABLE,1
    *   DISABLE,2
    *   DISABLE,ALL
    *
    *   ZERO                (zero both software positions; no movement)
    *   ZERO,1
    *   ZERO,2
    *   ZERO,ALL
    *
    *   STATUS
    *
    *   MOVE,1,+1,30        (Motor 1 clockwise 30 degrees)
    *   MOVE,2,-1,45        (Motor 2 counterclockwise 45 degrees)
    *
    *   MOVE2,+1,30,-1,45   (move both motors simultaneously)
    *
    * Backward-compatible Motor 1 form:
    *   MOVE,+1,30
    *   MOVE,-1,30
    */
    static void UART_ProcessCommand(char *command)
    {
        char *operation;
        char *token1;
        char *token2;
        char *token3;
        char *token4;
        char *extra_token;

        uint8_t motor_id;
        int32_t direction_sign;
        int32_t m1_direction_sign;
        int32_t m2_direction_sign;
        float move_angle_deg;
        float m1_angle_deg;
        float m2_angle_deg;
        uint32_t pulses_sent;
        uint32_t m1_pulses;
        uint32_t m2_pulses;

        if ((command == NULL) || (command[0] == '\0'))
        {
            return;
        }

        operation = strtok(command, ",");

        if (operation == NULL)
        {
            return;
        }

        if (strcmp(operation, "PING") == 0)
        {
            if (strtok(NULL, ",") != NULL)
            {
                UART_SendString("ERR,BAD_COMMAND\r\n");
                return;
            }

            UART_SendString("PONG\r\n");
            return;
        }

        if ((strcmp(operation, "ENABLE") == 0) ||
            (strcmp(operation, "DISABLE") == 0) ||
            (strcmp(operation, "ZERO") == 0))
        {
            token1 = strtok(NULL, ",");
            extra_token = strtok(NULL, ",");

            if (extra_token != NULL)
            {
                UART_SendString("ERR,BAD_COMMAND\r\n");
                return;
            }

            /* No selector means ALL, preserving the single-motor workflow. */
            if (token1 == NULL)
            {
                motor_id = 0U;
            }
            else if (ParseMotorSelector(token1,
                                        &motor_id,
                                        1U) == 0U)
            {
                UART_SendString("ERR,BAD_MOTOR\r\n");
                return;
            }

            if (strcmp(operation, "ENABLE") == 0)
            {
                EnableSelectedMotor(motor_id);
                UART_SendString("DONE,ENABLE\r\n");
            }
            else if (strcmp(operation, "DISABLE") == 0)
            {
                DisableSelectedMotor(motor_id);
                UART_SendString("DONE,DISABLE\r\n");
            }
            else
            {
                ZeroSelectedMotor(motor_id);
                UART_SendString("DONE,ZERO\r\n");
            }

            return;
        }

        if (strcmp(operation, "STATUS") == 0)
        {
            if (strtok(NULL, ",") != NULL)
            {
                UART_SendString("ERR,BAD_COMMAND\r\n");
                return;
            }

            UART_SendStatus();
            return;
        }

        if (strcmp(operation, "MOVE") == 0)
        {
            token1 = strtok(NULL, ",");
            token2 = strtok(NULL, ",");
            token3 = strtok(NULL, ",");
            extra_token = strtok(NULL, ",");

            if ((token1 == NULL) ||
                (token2 == NULL) ||
                (extra_token != NULL))
            {
                UART_SendString("ERR,BAD_MOVE_FORMAT\r\n");
                return;
            }

            if (token3 == NULL)
            {
                /*
                 * Backward-compatible syntax:
                 * MOVE,+1,30 or MOVE,-1,30 -> Motor 1
                 */
                motor_id = 1U;

                if ((ParseInt32Token(token1,
                                     &direction_sign) == 0U) ||
                    (ParseFloatToken(token2,
                                     &move_angle_deg) == 0U))
                {
                    UART_SendString("ERR,BAD_MOVE_FORMAT\r\n");
                    return;
                }
            }
            else
            {
                /*
                 * New dual-motor syntax:
                 * MOVE,motor_id,direction,angle
                 */
                if ((ParseMotorSelector(token1,
                                        &motor_id,
                                        0U) == 0U) ||
                    (ParseInt32Token(token2,
                                     &direction_sign) == 0U) ||
                    (ParseFloatToken(token3,
                                     &move_angle_deg) == 0U))
                {
                    UART_SendString("ERR,BAD_MOVE_FORMAT\r\n");
                    return;
                }
            }

            if ((direction_sign != 1) &&
                (direction_sign != -1))
            {
                UART_SendString("ERR,DIRECTION_RANGE\r\n");
                return;
            }

            if ((move_angle_deg < MOTOR_MIN_MOVE_DEG) ||
                (move_angle_deg > MOTOR_MAX_MOVE_DEG))
            {
                UART_SendString("ERR,ANGLE_RANGE\r\n");
                return;
            }

            if (((motor_id == 1U) && (m1_enabled == 0U)) ||
                ((motor_id == 2U) && (m2_enabled == 0U)))
            {
                UART_SendString("ERR,MOTOR_DISABLED\r\n");
                return;
            }

            pulses_sent = Stepper_AngleToPulses(move_angle_deg);

            UART_SendSingleAck(motor_id,
                               direction_sign,
                               move_angle_deg,
                               pulses_sent);

            pulses_sent = RunSingleMotorTest(motor_id,
                                             move_angle_deg,
                                             direction_sign);

            UART_SendSingleDone(
                motor_id,
                direction_sign,
                move_angle_deg,
                pulses_sent,
                (motor_id == 1U)
                    ? m1_position_pulses
                    : m2_position_pulses);
            return;
        }

        if (strcmp(operation, "MOVE2") == 0)
        {
            token1 = strtok(NULL, ",");
            token2 = strtok(NULL, ",");
            token3 = strtok(NULL, ",");
            token4 = strtok(NULL, ",");
            extra_token = strtok(NULL, ",");

            if ((token1 == NULL) ||
                (token2 == NULL) ||
                (token3 == NULL) ||
                (token4 == NULL) ||
                (extra_token != NULL) ||
                (ParseInt32Token(token1,
                                 &m1_direction_sign) == 0U) ||
                (ParseFloatToken(token2,
                                 &m1_angle_deg) == 0U) ||
                (ParseInt32Token(token3,
                                 &m2_direction_sign) == 0U) ||
                (ParseFloatToken(token4,
                                 &m2_angle_deg) == 0U))
            {
                UART_SendString("ERR,BAD_MOVE2_FORMAT\r\n");
                return;
            }

            if (((m1_direction_sign != 1) &&
                 (m1_direction_sign != -1)) ||
                ((m2_direction_sign != 1) &&
                 (m2_direction_sign != -1)))
            {
                UART_SendString("ERR,DIRECTION_RANGE\r\n");
                return;
            }

            if ((m1_angle_deg < MOTOR_MIN_MOVE_DEG) ||
                (m1_angle_deg > MOTOR_MAX_MOVE_DEG) ||
                (m2_angle_deg < MOTOR_MIN_MOVE_DEG) ||
                (m2_angle_deg > MOTOR_MAX_MOVE_DEG))
            {
                UART_SendString("ERR,ANGLE_RANGE\r\n");
                return;
            }

            if (((m1_angle_deg > 0.0f) && (m1_enabled == 0U)) ||
                ((m2_angle_deg > 0.0f) && (m2_enabled == 0U)))
            {
                UART_SendString("ERR,MOTOR_DISABLED\r\n");
                return;
            }

            m1_pulses = Stepper_AngleToPulses(m1_angle_deg);
            m2_pulses = Stepper_AngleToPulses(m2_angle_deg);

            UART_SendDualAck(m1_direction_sign,
                             m1_angle_deg,
                             m1_pulses,
                             m2_direction_sign,
                             m2_angle_deg,
                             m2_pulses);

            RunDualMotorTest(m1_angle_deg,
                             m1_direction_sign,
                             m2_angle_deg,
                             m2_direction_sign,
                             &m1_pulses,
                             &m2_pulses);

            UART_SendDualDone(m1_direction_sign,
                              m1_angle_deg,
                              m1_pulses,
                              m2_direction_sign,
                              m2_angle_deg,
                              m2_pulses);
            return;
        }

        UART_SendString("ERR,UNKNOWN_COMMAND\r\n");
    }


    /**
    * @brief Poll USART1 and build newline-terminated commands.
    */
    static void UART_Poll(void)
    {
        uint8_t received_byte;
        HAL_StatusTypeDef receive_status;

        receive_status = HAL_UART_Receive(&huart1,
                                          &received_byte,
                                          1U,
                                          10U);

        if (receive_status != HAL_OK)
        {
            return;
        }

        if ((received_byte == '\r') || (received_byte == '\n'))
        {
            if (uart_rx_index > 0U)
            {
                uart_rx_buffer[uart_rx_index] = '\0';
                UART_ProcessCommand(uart_rx_buffer);
                uart_rx_index = 0U;
            }

            return;
        }

        if (uart_rx_index < (UART_RX_BUFFER_SIZE - 1U))
        {
            uart_rx_buffer[uart_rx_index] = (char)received_byte;
            uart_rx_index++;
        }
        else
        {
            uart_rx_index = 0U;
            UART_SendString("ERR,LINE_TOO_LONG\r\n");
        }
    }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    /* Keep the ESP32 synchronization signal low initially. */
    HAL_GPIO_WritePin(MOTOR_MOVING_GPIO_Port,
                      MOTOR_MOVING_Pin,
                      GPIO_PIN_RESET);

    /* The physical power-up positions are initially software zero. */
    m1_position_pulses = 0;
    m2_position_pulses = 0;

    /*
     * Enable both TMC2209 drivers at startup.
     * EN is active-low.
     */
    HAL_GPIO_WritePin(M1_EN_GPIO_Port,
                      M1_EN_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_EN_GPIO_Port,
                      M2_EN_Pin,
                      GPIO_PIN_RESET);

    m1_enabled = 1U;
    m2_enabled = 1U;

    /* Wait for the USB-TTL connection and PC serial program. */
    HAL_Delay(500);
    UART_SendString("READY,DUAL_MOTOR\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        UART_Poll();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
//sdasdasd
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M2_EN_GPIO_Port, M2_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M1_DIR_Pin|MOTOR_MOVING_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M2_DIR_Pin M2_EN_Pin */
  GPIO_InitStruct.Pin = M2_DIR_Pin|M2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_DIR_Pin M1_EN_Pin */
  GPIO_InitStruct.Pin = M1_DIR_Pin|M1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_MOVING_Pin */
  GPIO_InitStruct.Pin = MOTOR_MOVING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_MOVING_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
