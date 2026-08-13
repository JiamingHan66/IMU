from __future__ import annotations

import csv
import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import serial
from serial.tools import list_ports
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


BAUD_RATE = 115200
MOTOR_PULSES_PER_REV = 1600  # 1.8 deg motor, 1/8 microstepping
DEFAULT_AVERAGE_SAMPLES = 50
MAX_HISTORY = 3000
FINAL_COLLECTION_TIMEOUT_S = 3.0
FINAL_POLL_MS = 100

AXIS_KEY = {"Roll": "roll", "Pitch": "pitch", "Yaw": "yaw"}
AXIS_INDEX = {"Roll": 0, "Pitch": 1, "Yaw": 2}

# Change a value to -1.0 only if the measured IMU direction is reversed
# for that sensor axis after the mechanical assembly is complete.
IMU_AXIS_SIGN = {"Roll": -1.0, "Pitch": 1.0, "Yaw": 1.0}

ACCURACY_TEXT = {0: "Unreliable", 1: "Low", 2: "Medium", 3: "High"}
Quaternion = tuple[float, float, float, float]


def normalize_quaternion(q: Quaternion) -> Quaternion:
    qw, qx, qy, qz = q
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        raise ValueError("Zero quaternion")
    return qw / norm, qx / norm, qy / norm, qz / norm


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    qw, qx, qy, qz = q
    return qw, -qx, -qy, -qz


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quaternion_dot(a: Quaternion, b: Quaternion) -> float:
    return sum(x * y for x, y in zip(a, b))


def quaternion_negate(q: Quaternion) -> Quaternion:
    return tuple(-value for value in q)  # type: ignore[return-value]


def mean_quaternion(quaternions: list[Quaternion]) -> Quaternion:
    if not quaternions:
        raise ValueError("No quaternion samples")

    reference = normalize_quaternion(quaternions[0])
    total = [0.0, 0.0, 0.0, 0.0]

    for q in quaternions:
        current = normalize_quaternion(q)
        if quaternion_dot(reference, current) < 0.0:
            current = quaternion_negate(current)
        for index, value in enumerate(current):
            total[index] += value

    return normalize_quaternion(tuple(total))  # type: ignore[arg-type]


def quaternion_to_euler_deg(q: Quaternion) -> tuple[float, float, float]:
    qw, qx, qy, qz = normalize_quaternion(q)

    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )

    sin_pitch = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def circular_mean_deg(values: list[float]) -> float:
    if not values:
        raise ValueError("No angle samples")

    mean_sin = sum(math.sin(math.radians(v)) for v in values) / len(values)
    mean_cos = sum(math.cos(math.radians(v)) for v in values) / len(values)
    return math.degrees(math.atan2(mean_sin, mean_cos))


def angle_difference_deg(initial_deg: float, final_deg: float) -> float:
    """Return final - initial wrapped to [-180, 180)."""
    return (final_deg - initial_deg + 180.0) % 360.0 - 180.0


def relative_quaternion(initial: Quaternion, final: Quaternion) -> Quaternion:
    relative = quaternion_multiply(
        quaternion_conjugate(normalize_quaternion(initial)),
        normalize_quaternion(final),
    )
    relative = normalize_quaternion(relative)
    if relative[0] < 0.0:
        relative = quaternion_negate(relative)
    return relative


def quaternion_rotation_magnitude_deg(q_relative: Quaternion) -> float:
    qw, qx, qy, qz = normalize_quaternion(q_relative)
    vector_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    return math.degrees(2.0 * math.atan2(vector_norm, max(0.0, qw)))


def quaternion_axis_component(q_relative: Quaternion, axis_index: int) -> float:
    _, qx, qy, qz = normalize_quaternion(q_relative)
    vector = (qx, qy, qz)
    vector_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if vector_norm < 1e-12:
        return 0.0
    return vector[axis_index] / vector_norm


def incremental_axis_rotation_deg(
    previous: Quaternion,
    current: Quaternion,
    axis_index: int,
) -> float:
    delta = relative_quaternion(previous, current)
    return (
        quaternion_rotation_magnitude_deg(delta)
        * quaternion_axis_component(delta, axis_index)
    )


def quaternion_euler_axis_deg(q: Quaternion, axis_index: int) -> float:
    """Return one human-readable Euler component of a quaternion."""
    return quaternion_to_euler_deg(q)[axis_index]


def axis_angle_quaternion(axis_index: int, angle_deg: float) -> Quaternion:
    """Create a quaternion for a rotation about one BNO085 coordinate axis."""
    half_angle = math.radians(angle_deg) / 2.0
    vector = [0.0, 0.0, 0.0]
    vector[axis_index] = math.sin(half_angle)
    return math.cos(half_angle), vector[0], vector[1], vector[2]


def dual_axis_forward_kinematics(
    m1_axis_index: int,
    m1_motor_deg: float,
    m1_axis_sign: float,
    m2_axis_index: int,
    m2_motor_deg: float,
    m2_axis_sign: float,
) -> Quaternion:
    """Return the pose predicted by the two-axis motor forward kinematics.

    Motor positions are in the motor convention. ``IMU_AXIS_SIGN`` converts
    them to the BNO085 coordinate convention.  The mechanical stack is:
    ``q_motor = q_M2(axis2) * q_M1(axis1)``.
    """
    if m1_axis_index == m2_axis_index:
        raise ValueError("Dual-axis FK requires two different IMU axes")

    return normalize_quaternion(
        quaternion_multiply(
            axis_angle_quaternion(
                m2_axis_index,
                m2_motor_deg * m2_axis_sign,
            ),
            axis_angle_quaternion(
                m1_axis_index,
                m1_motor_deg * m1_axis_sign,
            ),
        )
    )


def quaternion_rotation_axis(q: Quaternion) -> Optional[tuple[float, float, float]]:
    """Return the unit rotation axis, or None when the rotation is zero."""
    _, qx, qy, qz = normalize_quaternion(q)
    vector_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if vector_norm < 1e-12:
        return None
    return qx / vector_norm, qy / vector_norm, qz / vector_norm


class SerialWorker:
    def __init__(self, name: str, port: str, output_queue: queue.Queue):
        self.name = name
        self.port = port
        self.output_queue = output_queue
        self.ser: Optional[serial.Serial] = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def start(self) -> None:
        try:
            self.ser = serial.Serial(
                self.port,
                BAUD_RATE,
                timeout=0.1,
                write_timeout=1.0,
            )
        except serial.SerialException as exc:
            raise RuntimeError(f"Cannot open {self.port}: {exc}") from exc

        self.stop_event.clear()
        threading.Thread(target=self._read_loop, daemon=True).start()

    def _read_loop(self) -> None:
        assert self.ser is not None
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.output_queue.put((self.name, "line", line))
            except Exception as exc:
                self.output_queue.put((self.name, "error", str(exc)))
                break

    def send(self, command: str) -> None:
        if not self.is_open:
            raise RuntimeError(f"{self.name} is not connected")

        data = (command.rstrip("\r\n") + "\n").encode("ascii")
        with self.lock:
            assert self.ser is not None
            self.ser.write(data)
            self.ser.flush()

    def close(self) -> None:
        self.stop_event.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None


@dataclass
class ImuSample:
    q: Quaternion
    roll: float
    pitch: float
    yaw: float
    accuracy: int
    moving: int


@dataclass
class ActiveTest:
    motor_id: int
    axis_name: str
    axis_key: str
    axis_index: int
    imu_axis_sign: float
    motor_direction: int
    requested_move_deg: float
    command_rotation_deg: float
    software_start_deg: float
    initial_euler_deg: float
    initial_q: Quaternion
    sample_count: int
    state: str = "WAITING_ACK"
    actual_pulses: Optional[int] = None
    actual_move_deg: Optional[float] = None
    saw_motion: bool = False
    previous_motion_q: Optional[Quaternion] = None
    integrated_sensor_rotation_deg: float = 0.0
    final_samples: list[ImuSample] = field(default_factory=list)
    final_started_at: Optional[float] = None


@dataclass
class ActiveDualTest:
    m1_axis_name: str
    m1_axis_sign: float
    m2_axis_name: str
    m2_axis_sign: float

    m1_direction: int
    m2_direction: int
    m1_requested_deg: float
    m2_requested_deg: float

    m1_start_deg: float
    m2_start_deg: float
    initial_q: Quaternion

    m1_sample_count: int
    m2_sample_count: int

    state: str = "WAITING_ACK"
    saw_motion: bool = False
    m1_pulses: Optional[int] = None
    m2_pulses: Optional[int] = None
    m1_command_deg: Optional[float] = None
    m2_command_deg: Optional[float] = None
    final_samples: list[ImuSample] = field(default_factory=list)
    final_started_at: Optional[float] = None


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Dual-Motor IMU Verification")
        self.root.geometry("1180x850")
        self.root.minsize(1040, 760)

        self.events: queue.Queue = queue.Queue()
        self.esp: Optional[SerialWorker] = None
        self.stm: Optional[SerialWorker] = None

        self.history: deque[ImuSample] = deque(maxlen=MAX_HISTORY)
        self.latest: Optional[ImuSample] = None
        self.motor_position_deg: dict[int, Optional[float]] = {1: None, 2: None}
        self.motor_enabled: dict[int, Optional[bool]] = {1: None, 2: None}

        self.active_test: Optional[ActiveTest] = None
        self.active_dual_test: Optional[ActiveDualTest] = None
        self.results: list[dict[str, object]] = []
        self.run_buttons: dict[int, ttk.Button] = {}

        self._variables()
        self._layout()
        self.refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(25, self.process_events)

    def _variables(self) -> None:
        self.esp_port = tk.StringVar()
        self.stm_port = tk.StringVar()
        self.esp_status = tk.StringVar(value="Disconnected")
        self.stm_status = tk.StringVar(value="Disconnected")

        self.motor_angle = {1: tk.StringVar(value="15"), 2: tk.StringVar(value="15")}
        self.motor_direction = {
            1: tk.StringVar(value="+1 (Clockwise)"),
            2: tk.StringVar(value="+1 (Clockwise)"),
        }
        self.motor_axis = {
            1: tk.StringVar(value="Pitch"),
            2: tk.StringVar(value="Roll"),
        }
        self.motor_samples = {
            1: tk.StringVar(value=str(DEFAULT_AVERAGE_SAMPLES)),
            2: tk.StringVar(value=str(DEFAULT_AVERAGE_SAMPLES)),
        }
        self.motor_status_text = {
            1: tk.StringVar(value="M1: unknown"),
            2: tk.StringVar(value="M2: unknown"),
        }

        self.dual_angle = {1: tk.StringVar(value="15"), 2: tk.StringVar(value="15")}
        self.dual_direction = {
            1: tk.StringVar(value="+1 (Clockwise)"),
            2: tk.StringVar(value="+1 (Clockwise)"),
        }

        self.live_roll = tk.StringVar(value="--")
        self.live_pitch = tk.StringVar(value="--")
        self.live_yaw = tk.StringVar(value="--")
        self.live_accuracy = tk.StringVar(value="--")
        self.live_moving = tk.StringVar(value="--")
        self.test_state = tk.StringVar(value="Idle")

        self.result_motor = tk.StringVar(value="--")
        self.software_start = tk.StringVar(value="--")
        self.software_end = tk.StringVar(value="--")
        self.command_direction = tk.StringVar(value="--")
        self.requested_rotation = tk.StringVar(value="--")
        self.commanded_rotation = tk.StringVar(value="--")
        self.initial_angle = tk.StringVar(value="--")
        self.final_angle = tk.StringVar(value="--")
        self.measured_rotation = tk.StringVar(value="--")
        self.signed_error = tk.StringVar(value="--")
        self.absolute_error = tk.StringVar(value="--")
        self.percent_error = tk.StringVar(value="--")

        self.status = tk.StringVar(value="Connect ESP32 and STM32.")

    def _layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(3, weight=1)

        self._connection_frame()
        self._motor_control_frame()
        self._data_frame()
        self._log_frame()

        ttk.Label(
            self.root,
            textvariable=self.status,
            relief="sunken",
            anchor="w",
            padding=(6, 3),
        ).grid(row=4, column=0, sticky="ew", padx=10, pady=(4, 10))

    def _connection_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text="1. Serial Connections", padding=10)
        frame.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(5, weight=1)

        ttk.Label(frame, text="ESP32 / BNO085").grid(row=0, column=0, padx=4)
        self.esp_combo = ttk.Combobox(
            frame, textvariable=self.esp_port, state="readonly", width=16
        )
        self.esp_combo.grid(row=0, column=1, sticky="ew", padx=4)
        self.esp_button = ttk.Button(frame, text="Connect ESP32", command=self.toggle_esp)
        self.esp_button.grid(row=0, column=2, padx=4)

        ttk.Separator(frame, orient="vertical").grid(
            row=0, column=3, rowspan=2, sticky="ns", padx=10
        )

        ttk.Label(frame, text="STM32 / Two Motors").grid(row=0, column=4, padx=4)
        self.stm_combo = ttk.Combobox(
            frame, textvariable=self.stm_port, state="readonly", width=16
        )
        self.stm_combo.grid(row=0, column=5, sticky="ew", padx=4)
        self.stm_button = ttk.Button(frame, text="Connect STM32", command=self.toggle_stm)
        self.stm_button.grid(row=0, column=6, padx=4)
        ttk.Button(frame, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=7, padx=4
        )

        ttk.Label(frame, textvariable=self.esp_status).grid(
            row=1, column=0, columnspan=3, sticky="w", padx=4, pady=(5, 0)
        )
        ttk.Label(frame, textvariable=self.stm_status).grid(
            row=1, column=4, columnspan=4, sticky="w", padx=4, pady=(5, 0)
        )

    def _motor_control_frame(self) -> None:
        outer = ttk.Frame(self.root)
        outer.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)
        outer.columnconfigure(2, weight=1)

        self._single_motor_panel(outer, 1, 0)
        self._single_motor_panel(outer, 2, 1)
        self._dual_motor_panel(outer, 2)

    def _single_motor_panel(self, parent: ttk.Frame, motor_id: int, column: int) -> None:
        panel = ttk.LabelFrame(
            parent,
            text=f"Motor {motor_id} Verification",
            padding=10,
        )
        panel.grid(
            row=0,
            column=column,
            sticky="nsew",
            padx=(0, 5) if column < 2 else 5,
        )
        panel.columnconfigure(1, weight=1)

        self._entry_row(panel, 0, "Rotation magnitude (0-180°)", self.motor_angle[motor_id])

        ttk.Label(panel, text="Direction").grid(row=1, column=0, sticky="w", padx=4, pady=4)
        ttk.Combobox(
            panel,
            textvariable=self.motor_direction[motor_id],
            values=("+1 (Clockwise)", "-1 (Counterclockwise)"),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", padx=4, pady=4)

        ttk.Label(panel, text="IMU comparison axis").grid(
            row=2, column=0, sticky="w", padx=4, pady=4
        )
        ttk.Combobox(
            panel,
            textvariable=self.motor_axis[motor_id],
            values=("Roll", "Pitch", "Yaw"),
            state="readonly",
        ).grid(row=2, column=1, sticky="ew", padx=4, pady=4)

        self._entry_row(panel, 3, "Average samples", self.motor_samples[motor_id])

        button = ttk.Button(
            panel,
            text=f"RUN MOTOR {motor_id} TEST",
            command=lambda mid=motor_id: self.start_single_test(mid),
        )
        button.grid(row=4, column=0, columnspan=2, sticky="ew", padx=4, pady=(10, 4))
        self.run_buttons[motor_id] = button

        ttk.Label(panel, textvariable=self.motor_status_text[motor_id]).grid(
            row=5, column=0, columnspan=2, sticky="w", padx=4, pady=(5, 2)
        )

        command_row = ttk.Frame(panel)
        command_row.grid(row=6, column=0, columnspan=2, sticky="ew", padx=2, pady=2)
        for col in range(3):
            command_row.columnconfigure(col, weight=1)

        ttk.Button(
            command_row,
            text="Enable",
            command=lambda mid=motor_id: self.send_manual(f"ENABLE,{mid}"),
        ).grid(row=0, column=0, sticky="ew", padx=2)
        ttk.Button(
            command_row,
            text="Disable",
            command=lambda mid=motor_id: self.send_manual(f"DISABLE,{mid}"),
        ).grid(row=0, column=1, sticky="ew", padx=2)
        ttk.Button(
            command_row,
            text="Zero",
            command=lambda mid=motor_id: self.send_manual(f"ZERO,{mid}"),
        ).grid(row=0, column=2, sticky="ew", padx=2)

    def _dual_motor_panel(self, parent: ttk.Frame, column: int) -> None:
        panel = ttk.LabelFrame(parent, text="Dual-Motor Verification", padding=10)
        panel.grid(row=0, column=column, sticky="nsew", padx=(5, 0))
        panel.columnconfigure(1, weight=1)

        self._entry_row(panel, 0, "M1 rotation (0-180°)", self.dual_angle[1])
        ttk.Label(panel, text="M1 direction").grid(row=1, column=0, sticky="w", padx=4, pady=4)
        ttk.Combobox(
            panel,
            textvariable=self.dual_direction[1],
            values=("+1 (Clockwise)", "-1 (Counterclockwise)"),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", padx=4, pady=4)

        self._entry_row(panel, 2, "M2 rotation (0-180°)", self.dual_angle[2])
        ttk.Label(panel, text="M2 direction").grid(row=3, column=0, sticky="w", padx=4, pady=4)
        ttk.Combobox(
            panel,
            textvariable=self.dual_direction[2],
            values=("+1 (Clockwise)", "-1 (Counterclockwise)"),
            state="readonly",
        ).grid(row=3, column=1, sticky="ew", padx=4, pady=4)

        self.dual_run_button = ttk.Button(
            panel,
            text="RUN DUAL-MOTOR TEST",
            command=self.run_dual_move,
        )
        self.dual_run_button.grid(
            row=4, column=0, columnspan=2, sticky="ew", padx=4, pady=(10, 4)
        )

        commands = ttk.Frame(panel)
        commands.grid(row=5, column=0, columnspan=2, sticky="ew", padx=2, pady=2)
        for col in range(3):
            commands.columnconfigure(col, weight=1)

        for col, (text, command) in enumerate(
            (("Enable All", "ENABLE"), ("Disable All", "DISABLE"), ("Zero All", "ZERO"))
        ):
            ttk.Button(
                commands,
                text=text,
                command=lambda c=command: self.send_manual(c),
            ).grid(row=0, column=col, sticky="ew", padx=2, pady=2)

        ttk.Button(panel, text="STATUS", command=lambda: self.send_manual("STATUS")).grid(
            row=6, column=0, sticky="ew", padx=4, pady=4
        )
        ttk.Button(panel, text="PING", command=lambda: self.send_manual("PING")).grid(
            row=6, column=1, sticky="ew", padx=4, pady=4
        )

        ttk.Label(
            panel,
            text="Runs both motors together, verifies both IMU axes, and opens a result window.",
            wraplength=300,
        ).grid(row=7, column=0, columnspan=2, sticky="w", padx=4, pady=(6, 0))

    def _data_frame(self) -> None:
        outer = ttk.Frame(self.root)
        outer.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)

        live = ttk.LabelFrame(outer, text="3. Live IMU Data", padding=10)
        live.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        live.columnconfigure(1, weight=1)

        live_fields = [
            ("Roll", self.live_roll),
            ("Pitch", self.live_pitch),
            ("Yaw", self.live_yaw),
            ("Accuracy", self.live_accuracy),
            ("Motor moving", self.live_moving),
            ("Test state", self.test_state),
        ]
        for row, (label, variable) in enumerate(live_fields):
            self._value_row(live, row, label, variable)

        result = ttk.LabelFrame(outer, text="4. Latest Verification Result", padding=10)
        result.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        result.columnconfigure(1, weight=1)

        result_fields = [
            ("Motor", self.result_motor),
            ("Software start", self.software_start),
            ("Software end", self.software_end),
            ("Motor direction", self.command_direction),
            ("Requested rotation", self.requested_rotation),
            ("Actual motor command", self.commanded_rotation),
            ("Initial IMU angle", self.initial_angle),
            ("Final IMU angle", self.final_angle),
            ("Actual IMU rotation", self.measured_rotation),
            ("Signed error", self.signed_error),
            ("Absolute error", self.absolute_error),
            ("Percent error", self.percent_error),
        ]
        for row, (label, variable) in enumerate(result_fields):
            self._value_row(result, row, label, variable)

        ttk.Button(result, text="Save Results CSV", command=self.save_results).grid(
            row=len(result_fields),
            column=0,
            columnspan=2,
            sticky="ew",
            padx=4,
            pady=(10, 4),
        )

    def _log_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Serial Log", padding=8)
        frame.grid(row=3, column=0, sticky="nsew", padx=10, pady=5)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.log = tk.Text(frame, height=10, state="disabled", wrap="word")
        self.log.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.log.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scrollbar.set)

    @staticmethod
    def _entry_row(parent, row: int, label: str, variable: tk.StringVar) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(parent, textvariable=variable).grid(
            row=row, column=1, sticky="ew", padx=4, pady=4
        )

    @staticmethod
    def _value_row(parent, row: int, label: str, variable: tk.StringVar) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=3)
        ttk.Label(parent, textvariable=variable, anchor="e").grid(
            row=row, column=1, sticky="ew", padx=4, pady=3
        )

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.esp_combo["values"] = ports
        self.stm_combo["values"] = ports

        if ports:
            if self.esp_port.get() not in ports:
                self.esp_port.set(ports[0])
            if self.stm_port.get() not in ports:
                self.stm_port.set(ports[1] if len(ports) > 1 else ports[0])

        self.status.set(f"Detected {len(ports)} serial port(s).")

    def toggle_esp(self) -> None:
        if self.esp is not None and self.esp.is_open:
            self.esp.close()
            self.esp = None
            self.esp_status.set("Disconnected")
            self.esp_button.configure(text="Connect ESP32")
            return

        port = self.esp_port.get().strip()
        if not port:
            messagebox.showerror("ESP32", "Select the ESP32 port.")
            return
        if self.stm is not None and self.stm.is_open and self.stm.port == port:
            messagebox.showerror("Ports", "ESP32 and STM32 need different ports.")
            return

        try:
            self.esp = SerialWorker("ESP32", port, self.events)
            self.esp.start()
        except RuntimeError as exc:
            self.esp = None
            messagebox.showerror("ESP32", str(exc))
            return

        self.esp_status.set(f"Connected: {port}")
        self.esp_button.configure(text="Disconnect ESP32")

    def toggle_stm(self) -> None:
        if self.stm is not None and self.stm.is_open:
            self.stm.close()
            self.stm = None
            self.motor_position_deg = {1: None, 2: None}
            self.motor_enabled = {1: None, 2: None}
            self.stm_status.set("Disconnected")
            self.motor_status_text[1].set("M1: unknown")
            self.motor_status_text[2].set("M2: unknown")
            self.stm_button.configure(text="Connect STM32")
            return

        port = self.stm_port.get().strip()
        if not port:
            messagebox.showerror("STM32", "Select the STM32 port.")
            return
        if self.esp is not None and self.esp.is_open and self.esp.port == port:
            messagebox.showerror("Ports", "ESP32 and STM32 need different ports.")
            return

        try:
            self.stm = SerialWorker("STM32", port, self.events)
            self.stm.start()
        except RuntimeError as exc:
            self.stm = None
            messagebox.showerror("STM32", str(exc))
            return

        self.stm_status.set(f"Connected: {port}")
        self.stm_button.configure(text="Disconnect STM32")
        self.root.after(700, lambda: self.send_stm("STATUS", show_error=False))

    def process_events(self) -> None:
        for _ in range(500):
            try:
                source, event, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if event == "line":
                if source == "ESP32":
                    self.handle_esp(payload)
                else:
                    self.handle_stm(payload)
            else:
                self.append_log(f"{source} ERROR: {payload}")
                self.status.set(f"{source} serial error.")

        self.root.after(25, self.process_events)

    def handle_esp(self, line: str) -> None:
        if line.startswith("#"):
            self.append_log(f"ESP32: {line}")
            return
        if line.startswith("time_us,"):
            self.append_log("ESP32: IMU CSV stream started")
            return

        parts = line.split(",")
        if len(parts) != 8:
            return

        try:
            qw, qx, qy, qz = map(float, parts[1:5])
            accuracy = int(parts[5])
            moving = int(parts[7])
            q = normalize_quaternion((qw, qx, qy, qz))
            roll, pitch, yaw = quaternion_to_euler_deg(q)
        except ValueError:
            return

        if moving not in (0, 1):
            return

        sample = ImuSample(q, roll, pitch, yaw, accuracy, moving)
        self.latest = sample
        self.history.append(sample)

        self.live_roll.set(f"{roll:+.3f}°")
        self.live_pitch.set(f"{pitch:+.3f}°")
        self.live_yaw.set(f"{yaw:+.3f}°")
        self.live_accuracy.set(f"{accuracy} - {ACCURACY_TEXT.get(accuracy, 'Unknown')}")
        self.live_moving.set("1 - Moving" if moving else "0 - Stationary")

        self.track_motion_and_collect(sample)
        self.track_dual_motion_and_collect(sample)

    def track_motion_and_collect(self, sample: ImuSample) -> None:
        test = self.active_test
        if test is None:
            return

        if sample.moving:
            if not test.saw_motion:
                test.saw_motion = True
                test.state = "MOVING"
                test.previous_motion_q = sample.q
                test.integrated_sensor_rotation_deg = 0.0
                self.test_state.set(f"Motor {test.motor_id} moving")
            else:
                if test.previous_motion_q is not None:
                    test.integrated_sensor_rotation_deg += incremental_axis_rotation_deg(
                        test.previous_motion_q,
                        sample.q,
                        test.axis_index,
                    )
                test.previous_motion_q = sample.q
            return

        if test.saw_motion and test.state == "MOVING":
            if test.previous_motion_q is not None:
                test.integrated_sensor_rotation_deg += incremental_axis_rotation_deg(
                    test.previous_motion_q,
                    sample.q,
                    test.axis_index,
                )
            test.previous_motion_q = sample.q
            test.state = "WAITING_DONE"
            self.test_state.set("Motor stopped; waiting for STM32 DONE")
            return

        if test.state != "FINAL":
            return

        test.final_samples.append(sample)
        self.test_state.set(
            f"Final average {len(test.final_samples)}/{test.sample_count}"
        )
        if len(test.final_samples) >= test.sample_count:
            self.finish_test()

    def track_dual_motion_and_collect(self, sample: ImuSample) -> None:
        test = self.active_dual_test
        if test is None:
            return

        if sample.moving:
            if not test.saw_motion:
                test.saw_motion = True
                test.state = "MOVING"
                self.test_state.set("Dual motors moving")
            return

        if test.saw_motion and test.state == "MOVING":
            test.state = "WAITING_DONE"
            self.test_state.set("Dual motors stopped; waiting for STM32 DONE")
            return

        if test.state != "FINAL":
            return

        test.final_samples.append(sample)
        target = max(test.m1_sample_count, test.m2_sample_count)
        self.test_state.set(
            f"Dual final average {len(test.final_samples)}/{target}"
        )
        if len(test.final_samples) >= target:
            self.finish_dual_test()

    def handle_stm(self, line: str) -> None:
        self.append_log(f"STM32 -> PC: {line}")

        if line.startswith("READY"):
            self.status.set("STM32 READY.")
            self.root.after(150, lambda: self.send_stm("STATUS", show_error=False))
            return

        if line == "PONG":
            self.status.set("STM32 communication is working.")
            return

        if line in ("DONE,ENABLE", "DONE,DISABLE", "DONE,ZERO"):
            self.status.set(line.replace(",", " "))
            self.root.after(100, lambda: self.send_stm("STATUS", show_error=False))
            return

        if line.startswith("STATUS,"):
            parts = line.split(",")
            # STATUS,M1,ENABLED,+0.00,M2,ENABLED,+0.00
            if len(parts) != 7 or parts[1] != "M1" or parts[4] != "M2":
                return
            try:
                m1_position = float(parts[3])
                m2_position = float(parts[6])
            except ValueError:
                return

            self.motor_enabled[1] = parts[2] == "ENABLED"
            self.motor_enabled[2] = parts[5] == "ENABLED"
            self.motor_position_deg[1] = m1_position
            self.motor_position_deg[2] = m2_position

            self.motor_status_text[1].set(
                f"M1: {parts[2]}, position {m1_position:+.2f}°"
            )
            self.motor_status_text[2].set(
                f"M2: {parts[5]}, position {m2_position:+.2f}°"
            )
            self.stm_status.set(
                f"M1 {parts[2]} {m1_position:+.2f}° | "
                f"M2 {parts[5]} {m2_position:+.2f}°"
            )
            return

        if line.startswith("ACK,MOVE,"):
            parts = line.split(",")
            # ACK,MOVE,motor_id,direction,requested,pulses
            if len(parts) != 6:
                return
            try:
                motor_id = int(parts[2])
                direction = int(parts[3])
                pulses = int(parts[5])
            except ValueError:
                return

            test = self.active_test
            if test is None or test.motor_id != motor_id:
                return

            test.actual_pulses = pulses
            test.actual_move_deg = pulses * 360.0 / MOTOR_PULSES_PER_REV
            test.command_rotation_deg = direction * test.actual_move_deg
            if not test.saw_motion:
                test.state = "WAITING_MOTION"

            self.commanded_rotation.set(
                f"{test.command_rotation_deg:+.3f}° ({pulses} pulses)"
            )
            self.test_state.set(f"Waiting for Motor {motor_id} movement")
            return

        if line.startswith("DONE,MOVE,"):
            parts = line.split(",")
            # DONE,MOVE,motor_id,direction,requested,pulses,cumulative_pulses
            if len(parts) != 7:
                return
            try:
                motor_id = int(parts[2])
                direction = int(parts[3])
                pulses = int(parts[5])
                cumulative_pulses = int(parts[6])
            except ValueError:
                return

            position = cumulative_pulses * 360.0 / MOTOR_PULSES_PER_REV
            self.motor_position_deg[motor_id] = position
            enabled_text = "ENABLED" if self.motor_enabled.get(motor_id) else "UNKNOWN"
            self.motor_status_text[motor_id].set(
                f"M{motor_id}: {enabled_text}, position {position:+.2f}°"
            )

            test = self.active_test
            if test is not None and test.motor_id == motor_id:
                test.actual_pulses = pulses
                test.actual_move_deg = pulses * 360.0 / MOTOR_PULSES_PER_REV
                test.command_rotation_deg = direction * test.actual_move_deg

                # IMPORTANT: STM32 DONE can arrive after several stationary IMU
                # packets have already been received. The old code cleared the
                # buffer here and then waited only for *future* packets, which
                # could leave the GUI stuck at "Collecting final IMU angle".
                # Start FINAL mode, recover any stationary packets that are
                # already in history, then keep polling until we have enough.
                test.state = "FINAL"
                test.final_started_at = time.monotonic()
                test.final_samples.clear()
                self._refresh_final_samples_from_history(test)

                self.test_state.set(
                    f"Final average {len(test.final_samples)}/{test.sample_count}"
                )
                self.status.set(
                    f"Motor {motor_id} DONE. Averaging final stationary data."
                )

                if len(test.final_samples) >= test.sample_count:
                    self.finish_test()
                else:
                    self.root.after(
                        FINAL_POLL_MS,
                        lambda t=test: self._poll_final_collection(t),
                    )
            return

        if line.startswith("ACK,MOVE2,"):
            test = self.active_dual_test
            if test is not None and not test.saw_motion:
                test.state = "WAITING_MOTION"
                self.test_state.set("Waiting for dual-motor movement")
            self.status.set("Both motors accepted the command and started.")
            return

        if line.startswith("DONE,MOVE2,"):
            parts = line.split(",")
            # DONE,MOVE2,d1,a1,p1,cum1,d2,a2,p2,cum2
            if len(parts) != 10:
                return

            try:
                d1 = int(parts[2])
                p1 = int(parts[4])
                m1_cumulative = int(parts[5])
                d2 = int(parts[6])
                p2 = int(parts[8])
                m2_cumulative = int(parts[9])
            except ValueError:
                return

            self.motor_position_deg[1] = (
                m1_cumulative * 360.0 / MOTOR_PULSES_PER_REV
            )
            self.motor_position_deg[2] = (
                m2_cumulative * 360.0 / MOTOR_PULSES_PER_REV
            )
            self.motor_status_text[1].set(
                f"M1: position {self.motor_position_deg[1]:+.2f}°"
            )
            self.motor_status_text[2].set(
                f"M2: position {self.motor_position_deg[2]:+.2f}°"
            )

            test = self.active_dual_test
            if test is not None:
                test.m1_pulses = p1
                test.m2_pulses = p2
                test.m1_command_deg = d1 * p1 * 360.0 / MOTOR_PULSES_PER_REV
                test.m2_command_deg = d2 * p2 * 360.0 / MOTOR_PULSES_PER_REV
                test.state = "FINAL"
                test.final_started_at = time.monotonic()
                test.final_samples.clear()

                self._refresh_dual_final_samples_from_history(test)
                target = max(test.m1_sample_count, test.m2_sample_count)
                self.test_state.set(
                    f"Dual final average {len(test.final_samples)}/{target}"
                )
                self.status.set(
                    "Dual-motor move complete. Averaging final stationary IMU data."
                )

                if len(test.final_samples) >= target:
                    self.finish_dual_test()
                else:
                    self.root.after(
                        FINAL_POLL_MS,
                        lambda t=test: self._poll_dual_final_collection(t),
                    )
            else:
                self._set_run_controls(True)

            self.root.after(100, lambda: self.send_stm("STATUS", show_error=False))
            return

        if line.startswith("ERR,"):
            if self.active_test is not None:
                self.abort_test(line)
            if self.active_dual_test is not None:
                self.abort_dual_test(line)
            self._set_run_controls(True)
            self.status.set(f"STM32 returned {line}")

    def send_stm(self, command: str, show_error: bool = True) -> bool:
        if self.stm is None or not self.stm.is_open:
            if show_error:
                messagebox.showerror("STM32", "Connect STM32 first.")
            return False

        try:
            self.stm.send(command)
        except Exception as exc:
            if show_error:
                messagebox.showerror("STM32", str(exc))
            return False

        self.append_log(f"PC -> STM32: {command}")
        return True

    def send_manual(self, command: str) -> None:
        if self.active_test is not None or self.active_dual_test is not None:
            messagebox.showwarning(
                "Test running",
                "Wait for the active verification test to finish.",
            )
            return
        self.send_stm(command)

    def latest_stationary(self, count: int) -> list[ImuSample]:
        samples: list[ImuSample] = []
        for sample in reversed(self.history):
            if sample.moving:
                break
            samples.append(sample)
            if len(samples) >= count:
                break
        samples.reverse()
        return samples

    @staticmethod
    def _direction_value(text: str) -> int:
        return 1 if text.startswith("+1") else -1

    def start_single_test(self, motor_id: int) -> None:
        if self.active_test is not None or self.active_dual_test is not None:
            messagebox.showwarning("Test", "A test is already running.")
            return
        if self.esp is None or not self.esp.is_open:
            messagebox.showerror("Test", "Connect ESP32 first.")
            return
        if self.stm is None or not self.stm.is_open:
            messagebox.showerror("Test", "Connect STM32 first.")
            return
        if self.motor_position_deg[motor_id] is None:
            messagebox.showerror("Test", "Press STATUS or ZERO first.")
            return
        if self.motor_enabled[motor_id] is False:
            messagebox.showerror("Test", f"Motor {motor_id} is disabled.")
            return
        if self.latest is None:
            messagebox.showerror("Test", "No IMU data received.")
            return
        if self.latest.moving:
            messagebox.showerror("Test", "A motor is currently moving.")
            return

        try:
            requested_move = float(self.motor_angle[motor_id].get())
            count = int(self.motor_samples[motor_id].get())
        except ValueError:
            messagebox.showerror(
                "Test", "Rotation magnitude and sample count must be numbers."
            )
            return

        if not 0.0 <= requested_move <= 180.0:
            messagebox.showerror("Test", "Rotation magnitude must be 0° to 180°.")
            return
        if not 5 <= count <= 500:
            messagebox.showerror("Test", "Average samples must be 5 to 500.")
            return

        samples = self.latest_stationary(count)
        if len(samples) < count:
            messagebox.showerror(
                "Test",
                f"Need {count} stationary samples; only {len(samples)} available.",
            )
            return

        direction = self._direction_value(self.motor_direction[motor_id].get())
        axis_name = self.motor_axis[motor_id].get()
        axis_key = AXIS_KEY[axis_name]
        axis_index = AXIS_INDEX[axis_name]
        imu_axis_sign = IMU_AXIS_SIGN[axis_name]

        initial_q = mean_quaternion([sample.q for sample in samples])
        initial_euler = circular_mean_deg(
            [getattr(sample, axis_key) for sample in samples]
        )
        software_start = self.motor_position_deg[motor_id]
        assert software_start is not None

        self.active_test = ActiveTest(
            motor_id=motor_id,
            axis_name=axis_name,
            axis_key=axis_key,
            axis_index=axis_index,
            imu_axis_sign=imu_axis_sign,
            motor_direction=direction,
            requested_move_deg=requested_move,
            command_rotation_deg=direction * requested_move,
            software_start_deg=software_start,
            initial_euler_deg=initial_euler,
            initial_q=initial_q,
            sample_count=count,
        )

        self.result_motor.set(f"Motor {motor_id}")
        self.software_start.set(f"{software_start:+.3f}°")
        self.software_end.set("--")
        self.command_direction.set(
            "+1 Clockwise" if direction > 0 else "-1 Counterclockwise"
        )
        self.requested_rotation.set(f"{direction * requested_move:+.3f}°")
        self.commanded_rotation.set("Waiting for ACK")
        self.initial_angle.set(f"{initial_euler:+.3f}° ({axis_name})")
        self.final_angle.set("--")
        self.measured_rotation.set("--")
        self.signed_error.set("--")
        self.absolute_error.set("--")
        self.percent_error.set("--")
        self.test_state.set("Waiting for ACK")
        self._set_run_controls(False)

        command = f"MOVE,{motor_id},{direction:+d},{requested_move:.4f}"
        if not self.send_stm(command):
            self.abort_test("Failed to send MOVE")
            return

        self.status.set(
            f"Motor {motor_id} test started: requested "
            f"{direction * requested_move:+.3f}°."
        )

    def run_dual_move(self) -> None:
        if self.active_test is not None or self.active_dual_test is not None:
            messagebox.showwarning("Dual test", "A verification test is already running.")
            return
        if self.esp is None or not self.esp.is_open:
            messagebox.showerror("Dual test", "Connect ESP32 first.")
            return
        if self.stm is None or not self.stm.is_open:
            messagebox.showerror("Dual test", "Connect STM32 first.")
            return
        if self.motor_position_deg[1] is None or self.motor_position_deg[2] is None:
            messagebox.showerror("Dual test", "Press STATUS or ZERO first.")
            return
        if self.motor_enabled[1] is False or self.motor_enabled[2] is False:
            messagebox.showerror("Dual test", "Enable both motors first.")
            return
        if self.latest is None:
            messagebox.showerror("Dual test", "No IMU data received.")
            return
        if self.latest.moving:
            messagebox.showerror("Dual test", "A motor is currently moving.")
            return

        try:
            angle1 = float(self.dual_angle[1].get())
            angle2 = float(self.dual_angle[2].get())
            count1 = int(self.motor_samples[1].get())
            count2 = int(self.motor_samples[2].get())
        except ValueError:
            messagebox.showerror(
                "Dual test",
                "Angles and Average samples must be valid numbers.",
            )
            return

        if not (0.0 <= angle1 <= 180.0 and 0.0 <= angle2 <= 180.0):
            messagebox.showerror("Dual test", "Both angles must be 0° to 180°.")
            return
        if not (5 <= count1 <= 500 and 5 <= count2 <= 500):
            messagebox.showerror(
                "Dual test",
                "Motor 1 and Motor 2 Average samples must be 5 to 500.",
            )
            return

        target = max(count1, count2)
        samples = self.latest_stationary(target)
        if len(samples) < target:
            messagebox.showerror(
                "Dual test",
                f"Need {target} stationary IMU samples; only {len(samples)} available.",
            )
            return

        direction1 = self._direction_value(self.dual_direction[1].get())
        direction2 = self._direction_value(self.dual_direction[2].get())

        axis1_name = self.motor_axis[1].get()
        axis2_name = self.motor_axis[2].get()
        if axis1_name == axis2_name:
            messagebox.showerror(
                "Dual test",
                "Dual-axis verification requires Motor 1 and Motor 2 to use "
                "different IMU axes.",
            )
            return

        # The initial IMU orientation is the common reference pose for the
        # quaternion comparison after both motors finish their move.
        initial_q = mean_quaternion([sample.q for sample in samples])

        start1 = self.motor_position_deg[1]
        start2 = self.motor_position_deg[2]
        assert start1 is not None and start2 is not None

        self.active_dual_test = ActiveDualTest(
            m1_axis_name=axis1_name,
            m1_axis_sign=IMU_AXIS_SIGN[axis1_name],
            m2_axis_name=axis2_name,
            m2_axis_sign=IMU_AXIS_SIGN[axis2_name],
            m1_direction=direction1,
            m2_direction=direction2,
            m1_requested_deg=angle1,
            m2_requested_deg=angle2,
            m1_start_deg=start1,
            m2_start_deg=start2,
            initial_q=initial_q,
            m1_sample_count=count1,
            m2_sample_count=count2,
        )

        command = (
            f"MOVE2,{direction1:+d},{angle1:.4f},"
            f"{direction2:+d},{angle2:.4f}"
        )

        self.test_state.set("Dual test: waiting for ACK")
        self._set_run_controls(False)

        if not self.send_stm(command):
            self.abort_dual_test("Failed to send MOVE2")
            return

        self.status.set(
            f"Dual verification started: "
            f"M1 {direction1 * angle1:+.2f}° ({axis1_name}), "
            f"M2 {direction2 * angle2:+.2f}° ({axis2_name})."
        )

    def _refresh_final_samples_from_history(self, test: ActiveTest) -> None:
        """Recover the newest stationary samples after the motor stopped.

        latest_stationary() walks backward and stops as soon as it sees a
        moving sample, so when the MOTOR_MOVING signal worked during the move
        this naturally excludes the pre-move stationary data.
        """
        if self.active_test is not test or test.state != "FINAL":
            return

        samples = self.latest_stationary(test.sample_count)
        if test.saw_motion:
            # Safe to replace the buffer: the most recent moving sample acts as
            # a boundary between pre-move and post-move stationary data.
            test.final_samples = samples
        elif not test.final_samples:
            # If the moving flag was never observed, do not immediately trust
            # a full history window because it may include pre-move samples.
            # Future stationary packets will still be appended normally.
            test.final_samples = []

    def _poll_final_collection(self, test: ActiveTest) -> None:
        """Watch final IMU averaging so a test cannot hang forever."""
        if self.active_test is not test or test.state != "FINAL":
            return

        self._refresh_final_samples_from_history(test)
        count = len(test.final_samples)
        self.test_state.set(f"Final average {count}/{test.sample_count}")

        if count >= test.sample_count:
            self.finish_test()
            return

        started = test.final_started_at or time.monotonic()
        if time.monotonic() - started >= FINAL_COLLECTION_TIMEOUT_S:
            reason = (
                f"Final IMU averaging timed out ({count}/{test.sample_count} "
                "stationary samples). Check that the ESP32 is still streaming "
                "IMU data and that MOTOR_MOVING returns to 0."
            )
            self.append_log(reason)
            self.abort_test(reason)
            messagebox.showerror("IMU averaging timeout", reason)
            return

        self.root.after(
            FINAL_POLL_MS,
            lambda t=test: self._poll_final_collection(t),
        )

    def _refresh_dual_final_samples_from_history(
        self,
        test: ActiveDualTest,
    ) -> None:
        if self.active_dual_test is not test or test.state != "FINAL":
            return

        target = max(test.m1_sample_count, test.m2_sample_count)
        samples = self.latest_stationary(target)

        if test.saw_motion:
            test.final_samples = samples
        elif not test.final_samples:
            # If MOTOR_MOVING was never observed, only future stationary
            # packets received after DONE are trusted.
            test.final_samples = []

    def _poll_dual_final_collection(self, test: ActiveDualTest) -> None:
        if self.active_dual_test is not test or test.state != "FINAL":
            return

        self._refresh_dual_final_samples_from_history(test)
        target = max(test.m1_sample_count, test.m2_sample_count)
        count = len(test.final_samples)
        self.test_state.set(f"Dual final average {count}/{target}")

        if count >= target:
            self.finish_dual_test()
            return

        started = test.final_started_at or time.monotonic()
        if time.monotonic() - started >= FINAL_COLLECTION_TIMEOUT_S:
            reason = (
                f"Dual final IMU averaging timed out ({count}/{target} "
                "stationary samples). Check that the ESP32 is still streaming "
                "and MOTOR_MOVING returns to 0."
            )
            self.append_log(reason)
            self.abort_dual_test(reason)
            messagebox.showerror("Dual IMU averaging timeout", reason)
            return

        self.root.after(
            FINAL_POLL_MS,
            lambda t=test: self._poll_dual_final_collection(t),
        )

    def finish_dual_test(self) -> None:
        test = self.active_dual_test
        if test is None or not test.final_samples:
            return

        # Use one common final pose, matching the common initial pose recorded
        # when the dual test was started.
        final_q = mean_quaternion([sample.q for sample in test.final_samples])

        m1_axis_index = AXIS_INDEX[test.m1_axis_name]
        m2_axis_index = AXIS_INDEX[test.m2_axis_name]

        m1_command = (
            test.m1_command_deg
            if test.m1_command_deg is not None
            else test.m1_direction * test.m1_requested_deg
        )
        m2_command = (
            test.m2_command_deg
            if test.m2_command_deg is not None
            else test.m2_direction * test.m2_requested_deg
        )

        end1 = self.motor_position_deg[1]
        end2 = self.motor_position_deg[2]
        if end1 is None:
            end1 = test.m1_start_deg + m1_command
        if end2 is None:
            end2 = test.m2_start_deg + m2_command

        # Primary dual-axis measurement: predict each complete motor pose
        # with FK, compare the resulting pose change with the BNO085 pose
        # change, then take the shortest quaternion error rotation.  No
        # inverse-kinematics or per-joint error is used here.
        q_motor_initial = dual_axis_forward_kinematics(
            m1_axis_index,
            test.m1_start_deg,
            test.m1_axis_sign,
            m2_axis_index,
            test.m2_start_deg,
            test.m2_axis_sign,
        )
        q_motor_final = dual_axis_forward_kinematics(
            m1_axis_index,
            end1,
            test.m1_axis_sign,
            m2_axis_index,
            end2,
            test.m2_axis_sign,
        )
        q_motor_relative = relative_quaternion(q_motor_initial, q_motor_final)
        q_imu_relative = relative_quaternion(test.initial_q, final_q)
        q_error = relative_quaternion(q_motor_relative, q_imu_relative)
        orientation_error_deg = quaternion_rotation_magnitude_deg(q_error)
        error_axis = quaternion_rotation_axis(q_error)
        imu_rotation_magnitude_deg = quaternion_rotation_magnitude_deg(
            q_imu_relative
        )

        now = time.strftime("%Y-%m-%d %H:%M:%S")
        common_number = len(self.results) + 1

        # A dual test produces one 3D orientation result, not one result per
        # motor.  The two motor commands are retained as metadata in this row.
        self.results.append(
            {
                "test_number": common_number,
                "time": now,
                "test_mode": "Dual FK quaternion",
                "motor_id": "M1+M2",
                "m1_axis": test.m1_axis_name,
                "m1_imu_axis_sign": test.m1_axis_sign,
                "m1_direction": test.m1_direction,
                "m1_requested_move_deg": test.m1_requested_deg,
                "m1_actual_step_pulses": test.m1_pulses,
                "m1_quantized_command_deg": m1_command,
                "m1_software_start_deg": test.m1_start_deg,
                "m1_software_end_deg": end1,
                "m2_axis": test.m2_axis_name,
                "m2_imu_axis_sign": test.m2_axis_sign,
                "m2_direction": test.m2_direction,
                "m2_requested_move_deg": test.m2_requested_deg,
                "m2_actual_step_pulses": test.m2_pulses,
                "m2_quantized_command_deg": m2_command,
                "m2_software_start_deg": test.m2_start_deg,
                "m2_software_end_deg": end2,
                "imu_relative_rotation_magnitude_deg": imu_rotation_magnitude_deg,
                "fk_predicted_relative_q_w": q_motor_relative[0],
                "fk_predicted_relative_q_x": q_motor_relative[1],
                "fk_predicted_relative_q_y": q_motor_relative[2],
                "fk_predicted_relative_q_z": q_motor_relative[3],
                "imu_relative_q_w": q_imu_relative[0],
                "imu_relative_q_x": q_imu_relative[1],
                "imu_relative_q_y": q_imu_relative[2],
                "imu_relative_q_z": q_imu_relative[3],
                "orientation_error_q_w": q_error[0],
                "orientation_error_q_x": q_error[1],
                "orientation_error_q_y": q_error[2],
                "orientation_error_q_z": q_error[3],
                "orientation_error_deg": orientation_error_deg,
                "orientation_error_axis_x": "" if error_axis is None else error_axis[0],
                "orientation_error_axis_y": "" if error_axis is None else error_axis[1],
                "orientation_error_axis_z": "" if error_axis is None else error_axis[2],
            }
        )

        result_data = {
            "motors": {
                1: {
                    "axis": test.m1_axis_name,
                    "start": test.m1_start_deg,
                    "end": end1,
                    "direction": test.m1_direction,
                    "requested": test.m1_requested_deg,
                    "pulses": test.m1_pulses,
                    "command": m1_command,
                },
                2: {
                    "axis": test.m2_axis_name,
                    "start": test.m2_start_deg,
                    "end": end2,
                    "direction": test.m2_direction,
                    "requested": test.m2_requested_deg,
                    "pulses": test.m2_pulses,
                    "command": m2_command,
                },
            },
            "motor_relative_q": q_motor_relative,
            "imu_relative_q": q_imu_relative,
            "error_q": q_error,
            "error_axis": error_axis,
            "orientation_error_deg": orientation_error_deg,
        }

        self.active_dual_test = None
        self._set_run_controls(True)
        self.test_state.set("Dual complete")
        self.status.set(
            f"Dual complete: 3D orientation error {orientation_error_deg:.3f}°."
        )
        self.show_dual_result_window(result_data)

    def show_dual_result_window(self, data: dict[str, object]) -> None:
        window = tk.Toplevel(self.root)
        window.title("Dual-Motor Verification Result")
        window.geometry("790x670")
        window.minsize(700, 600)
        window.transient(self.root)

        outer = ttk.Frame(window, padding=14)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)

        ttk.Label(
            outer,
            text="Dual-Axis FK Quaternion Verification",
            font=("TkDefaultFont", 13, "bold"),
        ).grid(row=0, column=0, columnspan=2, pady=(0, 10))

        motors = data["motors"]
        assert isinstance(motors, dict)
        for col, motor_id in enumerate((1, 2)):
            item = motors[motor_id]
            assert isinstance(item, dict)
            panel = ttk.LabelFrame(
                outer,
                text=f"Motor {motor_id}  ({item['axis']})",
                padding=10,
            )
            panel.grid(
                row=1,
                column=col,
                sticky="nsew",
                padx=(0, 6) if col == 0 else (6, 0),
            )
            panel.columnconfigure(1, weight=1)

            direction_text = (
                "+1 Clockwise"
                if int(item["direction"]) > 0
                else "-1 Counterclockwise"
            )
            pulses = item["pulses"]
            pulse_text = "--" if pulses is None else str(pulses)

            fields = [
                ("Software start", f"{float(item['start']):+.3f}°"),
                ("Software end", f"{float(item['end']):+.3f}°"),
                ("Direction", direction_text),
                (
                    "Requested rotation",
                    f"{int(item['direction']) * float(item['requested']):+.3f}°",
                ),
                (
                    "Actual motor command",
                    f"{float(item['command']):+.3f}° ({pulse_text} pulses)",
                ),
            ]

            for row, (label, value) in enumerate(fields):
                ttk.Label(panel, text=label).grid(
                    row=row, column=0, sticky="w", padx=4, pady=5
                )
                ttk.Label(panel, text=value, anchor="e").grid(
                    row=row, column=1, sticky="ew", padx=4, pady=5
                )

        def format_quaternion(q: Quaternion) -> str:
            return "[" + ", ".join(f"{value:+.6f}" for value in q) + "]"

        q_motor_relative = data["motor_relative_q"]
        q_imu_relative = data["imu_relative_q"]
        q_error = data["error_q"]
        error_axis = data["error_axis"]
        assert isinstance(q_motor_relative, tuple)
        assert isinstance(q_imu_relative, tuple)
        assert isinstance(q_error, tuple)
        assert error_axis is None or isinstance(error_axis, tuple)

        metric = ttk.LabelFrame(
            outer,
            text="Primary 3D Orientation Metric",
            padding=10,
        )
        metric.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(14, 0))
        metric.columnconfigure(1, weight=1)

        axis_text = (
            "N/A (zero rotation)"
            if error_axis is None
            else "[" + ", ".join(f"{value:+.6f}" for value in error_axis) + "]"
        )
        metric_fields = [
            ("Motor FK predicted Δq", format_quaternion(q_motor_relative)),
            ("BNO085 measured Δq", format_quaternion(q_imu_relative)),
            ("Quaternion error Δq", format_quaternion(q_error)),
            ("Error rotation axis", axis_text),
            (
                "3D orientation error",
                f"{float(data['orientation_error_deg']):.3f}°",
            ),
        ]
        for row, (label, value) in enumerate(metric_fields):
            ttk.Label(metric, text=label).grid(
                row=row, column=0, sticky="w", padx=4, pady=5
            )
            ttk.Label(metric, text=value, anchor="e").grid(
                row=row, column=1, sticky="ew", padx=4, pady=5
            )

        button_row = ttk.Frame(outer)
        button_row.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        button_row.columnconfigure(0, weight=1)
        button_row.columnconfigure(1, weight=1)

        ttk.Button(
            button_row,
            text="Save Results CSV",
            command=self.save_results,
        ).grid(row=0, column=0, sticky="ew", padx=(0, 5))

        ttk.Button(
            button_row,
            text="Close",
            command=window.destroy,
        ).grid(row=0, column=1, sticky="ew", padx=(5, 0))

        window.lift()
        window.focus_force()

    def abort_dual_test(self, reason: str) -> None:
        self.active_dual_test = None
        self._set_run_controls(True)
        self.test_state.set("Dual aborted")
        self.status.set(f"Dual test aborted: {reason}")

    def finish_test(self) -> None:
        test = self.active_test
        if test is None or not test.final_samples:
            return

        final_q = mean_quaternion([sample.q for sample in test.final_samples])
        final_euler = circular_mean_deg(
            [getattr(sample, test.axis_key) for sample in test.final_samples]
        )

        q_relative = relative_quaternion(test.initial_q, final_q)
        magnitude_deg = quaternion_rotation_magnitude_deg(q_relative)
        axis_component = quaternion_axis_component(q_relative, test.axis_index)
        quaternion_axis_signed = (
            magnitude_deg * axis_component * test.imu_axis_sign
        )
        relative_euler_signed = (
            quaternion_euler_axis_deg(q_relative, test.axis_index)
            * test.imu_axis_sign
        )
        integrated_motor_signed = (
            test.integrated_sensor_rotation_deg * test.imu_axis_sign
        )

        # The integrated value preserves direction over the complete movement.
        # The final quaternion projection is retained in CSV as a cross-check.
        measured = integrated_motor_signed if test.saw_motion else quaternion_axis_signed

        signed_error = measured - test.command_rotation_deg
        absolute_error = abs(signed_error)
        percent_error = (
            absolute_error / abs(test.command_rotation_deg) * 100.0
            if abs(test.command_rotation_deg) > 1e-9
            else math.nan
        )

        software_end = self.motor_position_deg[test.motor_id]
        if software_end is None:
            software_end = (
                test.software_start_deg
                + test.command_rotation_deg
            )

        self.software_end.set(f"{software_end:+.3f}°")
        self.final_angle.set(f"{final_euler:+.3f}° ({test.axis_name})")
        self.measured_rotation.set(f"{measured:+.3f}°")
        self.signed_error.set(f"{signed_error:+.3f}°")
        self.absolute_error.set(f"{absolute_error:.3f}°")
        self.percent_error.set(
            "N/A" if math.isnan(percent_error) else f"{percent_error:.3f}%"
        )

        self.results.append(
            {
                "test_number": len(self.results) + 1,
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "test_mode": "Single",
                "motor_id": test.motor_id,
                "axis": test.axis_name,
                "imu_axis_sign": test.imu_axis_sign,
                "motor_direction": test.motor_direction,
                "requested_move_deg": test.requested_move_deg,
                "actual_step_pulses": test.actual_pulses,
                "quantized_command_deg": test.command_rotation_deg,
                "software_start_deg": test.software_start_deg,
                "software_end_deg": software_end,
                "initial_euler_reference_deg": test.initial_euler_deg,
                "final_euler_reference_deg": final_euler,
                "quaternion_rotation_magnitude_deg": magnitude_deg,
                "quaternion_axis_projected_deg": quaternion_axis_signed,
                "relative_euler_axis_deg": relative_euler_signed,
                "integrated_sensor_axis_deg": test.integrated_sensor_rotation_deg,
                "integrated_motor_axis_deg": integrated_motor_signed,
                "measured_signed_rotation_deg": measured,
                "signed_error_deg": signed_error,
                "absolute_error_deg": absolute_error,
                "percent_error": percent_error,
            }
        )

        self.test_state.set("Complete")
        self.status.set(
            f"Motor {test.motor_id} complete: command "
            f"{test.command_rotation_deg:+.3f}°, IMU {measured:+.3f}°, "
            f"error {signed_error:+.3f}°."
        )
        self.active_test = None
        self._set_run_controls(True)

    def abort_test(self, reason: str) -> None:
        self.active_test = None
        self._set_run_controls(True)
        self.test_state.set("Aborted")
        self.status.set(f"Test aborted: {reason}")

    def _set_run_controls(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for button in self.run_buttons.values():
            button.configure(state=state)
        self.dual_run_button.configure(state=state)

    def save_results(self) -> None:
        if not self.results:
            messagebox.showinfo("Save", "No completed results.")
            return

        filename = filedialog.asksaveasfilename(
            title="Save results",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile="dual_motor_imu_results.csv",
        )
        if not filename:
            return

        try:
            with open(filename, "w", newline="", encoding="utf-8") as file:
                # Single- and dual-axis tests intentionally record different
                # metrics.  Keep every available column when both test modes
                # are exported in one CSV.
                fieldnames = list(
                    dict.fromkeys(
                        key for result in self.results for key in result
                    )
                )
                writer = csv.DictWriter(file, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.results)
        except OSError as exc:
            messagebox.showerror("Save", str(exc))
            return

        self.status.set(f"Saved results to {filename}")

    def append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def close(self) -> None:
        if self.esp is not None:
            self.esp.close()
        if self.stm is not None:
            self.stm.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
