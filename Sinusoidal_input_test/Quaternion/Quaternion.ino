#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// ============================================================
// ESP32 pins
// ============================================================
constexpr int BNO_SDA_PIN = 21;
constexpr int BNO_SCL_PIN = 22;

/*
 * STM32 PB10 -> ESP32 GPIO27
 *
 * moving = 1: motor is moving
 * moving = 0: motor is stationary
 */
constexpr int MOTOR_MOVING_PIN = 27;

// No BNO085 reset pin connected
constexpr int BNO_RESET_PIN = -1;

// 10000 us = 100 Hz
constexpr uint32_t REPORT_INTERVAL_US = 10000;

// BNO085 object
Adafruit_BNO08x bno08x(BNO_RESET_PIN);

// Latest SH-2 sensor report
sh2_SensorValue_t sensorValue;


// ============================================================
// Enable Game Rotation Vector
// ============================================================
bool enableGameRotationVector()
{
  Serial.println("# Enabling SH2_GAME_ROTATION_VECTOR...");

  const bool success = bno08x.enableReport(
    SH2_GAME_ROTATION_VECTOR,
    REPORT_INTERVAL_US
  );

  if (!success) {
    Serial.println(
      "# ERROR: Could not enable Game Rotation Vector"
    );
    return false;
  }

  Serial.println(
    "# Game Rotation Vector enabled at 100 Hz"
  );

  return true;
}


// ============================================================
// Setup
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Synchronization input from STM32
  pinMode(MOTOR_MOVING_PIN, INPUT_PULLDOWN);

  Serial.println();
  Serial.println("# BNO085 Game Rotation Vector Test");
  Serial.println("# Communication: I2C");
  Serial.println("# Fusion: Accelerometer + Gyroscope");
  Serial.println("# Magnetometer: Not used");

  // Start ESP32 I2C
  Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN);

  // Initialize BNO085 at default I2C address 0x4A
  if (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("# ERROR: BNO085 not found");
    Serial.println("# Check:");
    Serial.println("# 1. VIN -> 3.3V");
    Serial.println("# 2. GND -> GND");
    Serial.println("# 3. SDA -> GPIO21");
    Serial.println("# 4. SCL -> GPIO22");
    Serial.println(
      "# 5. P0 is not connected to 3.3V"
    );

    while (true) {
      delay(1000);
    }
  }

  Serial.println("# BNO085 found");

  if (!enableGameRotationVector()) {
    while (true) {
      delay(1000);
    }
  }

  // Python recognizes this as the CSV header
  Serial.println(
    "time_us,qw,qx,qy,qz,accuracy,q_norm,moving"
  );
}


// ============================================================
// Main loop
// ============================================================
void loop()
{
  /*
   * BNO085 may reset internally.
   * Reports must be enabled again after a reset.
   */
  if (bno08x.wasReset()) {
    Serial.println("# BNO085 reset detected");

    if (!enableGameRotationVector()) {
      Serial.println(
        "# ERROR: Failed to restore report"
      );
    }
  }

  // Wait until a new sensor report is available
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  // Ignore other SH-2 report types
  if (
    sensorValue.sensorId !=
    SH2_GAME_ROTATION_VECTOR
  ) {
    return;
  }

  // Quaternion order: qw, qx, qy, qz
  const float qw =
    sensorValue.un.gameRotationVector.real;

  const float qx =
    sensorValue.un.gameRotationVector.i;

  const float qy =
    sensorValue.un.gameRotationVector.j;

  const float qz =
    sensorValue.un.gameRotationVector.k;

  // Quaternion norm should remain close to 1.0
  const float qNorm = sqrtf(
    qw * qw +
    qx * qx +
    qy * qy +
    qz * qz
  );

  /*
   * Accuracy status:
   * 0 = unreliable
   * 1 = low
   * 2 = medium
   * 3 = high
   */
  const uint8_t accuracy = sensorValue.status;

  // ESP32 timestamp
  const uint32_t timeUs = micros();

  // Read motor synchronization signal
  const int motorMoving =
    digitalRead(MOTOR_MOVING_PIN);

  // Output exactly one complete CSV row
  Serial.print(timeUs);
  Serial.print(",");

  Serial.print(qw, 6);
  Serial.print(",");

  Serial.print(qx, 6);
  Serial.print(",");

  Serial.print(qy, 6);
  Serial.print(",");

  Serial.print(qz, 6);
  Serial.print(",");

  Serial.print(accuracy);
  Serial.print(",");

  Serial.print(qNorm, 6);
  Serial.print(",");

  Serial.println(motorMoving);
}