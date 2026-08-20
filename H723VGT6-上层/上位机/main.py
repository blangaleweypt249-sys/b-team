# -*- coding: utf-8 -*-
"""H723VGT6 upper-control console."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # The UI can still open and explain the missing package.
    serial = None
    list_ports = None

from protocol import (
    COMMAND_J4310_STOP,
    ENABLE_CONVEYOR,
    ENABLE_J4310_ONLY,
    ENABLE_M3508_ONLY,
    ENABLE_GRIPPER,
    HANDSHAKE_MAGIC,
    MOTOR_ACTION_J4310_SAVE_ZERO,
    MOTOR_ACTION_J4310_AUTO_RETURN,
    MSG_ACK,
    MSG_AUX_CONTROL,
    MSG_ESTOP,
    MSG_FAULT,
    MSG_HEARTBEAT,
    MSG_MOTOR_ACTION,
    MSG_MOTOR_ACTION_RESULT,
    MSG_DJI_TELEMETRY,
    MSG_ROBOT_STATE,
    MSG_UPPER_POSITION_CMD,
    AUX_OUTPUT_ARM_CYLINDER,
    AUX_OUTPUT_PUSH_CYLINDER,
    AUX_OUTPUT_GRIPPER_CYLINDER,
    AUX_OUTPUT_ESTOP,
    Frame,
    FrameParser,
    build_motor_action_payload,
    build_aux_control_payload,
    build_handshake_frame,
    build_extended_position_payload,
    build_position_torque_payload,
    decode_motor_event,
    decode_motor_action_result,
    decode_dji_telemetry,
    decode_robot_state,
    encode_frame,
)


BAUD_RATES = (115200, 230400, 460800, 921600)
SEND_PERIOD_MS = 50
REMOTE_PD11_DELAY_MS = 500
REMOTE_PD8_FIRST_DELAY_MS = 500
REMOTE_PD9_ZERO_GATE_DEG = 0.0
HEARTBEAT_PERIOD_MS = 100
HANDSHAKE_PERIOD_MS = 150
HANDSHAKE_TIMEOUT_MS = 3000
HANDSHAKE_OPEN_DELAY_MS = 300
HANDSHAKE_SEQUENCE = 0x1234
J4310_POSITION_LIMIT = 12.5
J4310_VELOCITY_LIMIT = 30.0
J4310_KP_LIMIT = 49.0
J4310_KD_LIMIT = 0.95
J4310_TORQUE_LIMIT = 10.0
J4310_RAW_MIN_DEG = -math.degrees(J4310_POSITION_LIMIT)
J4310_RAW_MAX_DEG = math.degrees(J4310_POSITION_LIMIT)
J4310_POSITION_MIN_DEG = -90.0
J4310_POSITION_MAX_DEG = 270.0
POSITION_MIN_DEG = 0.0
POSITION_MAX_DEG = 1200.0
M3508_SYNC_POSITION_MIN_DEG = 0.0
M3508_SYNC_POSITION_MAX_DEG = 1200.0
M3508_TARGET_RANGES_DEG = (
    (
        min(POSITION_MIN_DEG, M3508_SYNC_POSITION_MIN_DEG),
        max(POSITION_MAX_DEG, M3508_SYNC_POSITION_MAX_DEG),
    ),
    (
        min(POSITION_MIN_DEG, M3508_SYNC_POSITION_MIN_DEG),
        max(POSITION_MAX_DEG, M3508_SYNC_POSITION_MAX_DEG),
    ),
)
M2006_POSITION_MIN_DEG = -360.0
M2006_POSITION_MAX_DEG = 360.0
GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG = 2.0
J4310_CAN_BUS = 1
J4310_NODE_ID = 0x06
PID_GAIN_MIN = 0.0
PID_KP_KI_MAX = 10000.0
PID_KD_MAX = 1000.0
PID_INTEGRAL_LIMIT_MAX = 100000.0
PID_OUTPUT_LIMIT_MIN = 0.1
PID_SPEED_OUTPUT_LIMIT_MAX = 32767.0
PID_POSITION_OUTPUT_LIMIT_MAX = 1000.0
GATE_M2006_ID = 1
GRIPPER_M2006_ID = 2
M2006_NODE_IDS = {
    "conveyor": GATE_M2006_ID,
    "gripper": GRIPPER_M2006_ID,
}
DJI_REDUCTION_RATIOS = {
    1: 3591.0 / 187.0,
    2: 36.0,
}
MOTOR_LOG_PATH = Path(__file__).resolve().parents[2] / "电机日志.log"
MOTOR_PARAMETER_PATH = Path(__file__).resolve().with_name("motor_parameters.json")
REMOTE_ACTION_PARAMETER_VERSION = 9
LEGACY_GATE_ACTION_KEY = "\u5f00\u5173\u95e8"

MOTOR_MODEL_NAMES = {
    0: "J4310",
    1: "M3508",
    2: "M2006",
}

J4310_FAULT_NAMES = {
    0x2: "电机侧编码器异常",
    0x3: "输出轴编码器异常",
    0x5: "电机侧编码器读取错误",
    0x7: "输出轴编码器读取错误",
    0x8: "过压",
    0x9: "欠压",
    0xA: "过流",
    0xB: "MOS 过温",
    0xC: "线圈过温",
    0xD: "通信丢失",
    0xE: "过载",
}

REFERENCE_PID_VALUES = {
    "M3508 / C620": (
        ("速度环", "120", "80", "0", "57.2958", "2458"),
        ("位置环", "100", "0", "0", "0", "150 rpm"),
    ),
    "M2006 / C610": (
        ("速度环", "350", "250", "0", "0.5", "10000"),
        ("位置环", "900", "500", "5", "0.002", "50 rpm"),
    ),
}

STATE_NAMES = {
    0: "初始化",
    1: "就绪",
    2: "运行",
    3: "停止",
    4: "错误",
}

def format_dji_feedback(
    name: str,
    zero_status: str,
    feedback_status: str,
    current_output_deg: float | None = None,
    relative_zero_deg: float | None = None,
) -> str:
    current_text = (
        "--" if current_output_deg is None else f"{current_output_deg:.2f}"
    )
    relative_text = (
        "--" if relative_zero_deg is None else f"{relative_zero_deg:.2f}"
    )
    return (
        f"{name}\n"
        f"当前输出轴角度: {current_text} deg\n"
        f"相对标定零点角度: {relative_text} deg\n"
        f"是否标零: {zero_status}\n"
        f"反馈状态: {feedback_status}"
    )


class SerialTransport:
    """Non-blocking serial worker with queues owned by the Tk main thread."""

    def __init__(self) -> None:
        self.rx_queue: queue.Queue[bytes] = queue.Queue()
        self.error_queue: queue.Queue[str] = queue.Queue()
        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        with self._state_lock:
            port = self._serial
        return port is not None and bool(port.is_open)

    def matches(self, port: str, baudrate: int) -> bool:
        with self._state_lock:
            current = self._serial
        return (
            current is not None
            and bool(current.is_open)
            and current.port == port
            and current.baudrate == baudrate
        )

    @staticmethod
    def available_ports() -> list[tuple[str, str]]:
        if list_ports is None:
            return []
        return sorted(
            [(item.device, item.description or "串口设备") for item in list_ports.comports()],
            key=lambda item: item[0],
        )

    def open(self, port: str, baudrate: int) -> None:
        if serial is None:
            raise RuntimeError("未安装 pyserial，请运行: python -m pip install -r requirements.txt")
        self.close()
        # Set modem-control lines before opening the CDC port. Constructing an
        # already-open Serial object uses pyserial's default DTR/RTS=True;
        # wireless DAP firmware may forward those controls to target reset.
        port_handle = serial.Serial()
        port_handle.port = port
        port_handle.baudrate = baudrate
        port_handle.bytesize = serial.EIGHTBITS
        port_handle.parity = serial.PARITY_NONE
        port_handle.stopbits = serial.STOPBITS_ONE
        port_handle.timeout = 0.1
        port_handle.write_timeout = 0.2
        port_handle.dtr = False
        port_handle.rts = False
        try:
            port_handle.open()
        except Exception:
            port_handle.close()
            raise
        with self._state_lock:
            self._serial = port_handle
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._reader,
            args=(port_handle,),
            daemon=True,
        )
        self._thread.start()

    def discard_input(self) -> None:
        with self._state_lock:
            port = self._serial
        if port is not None and port.is_open:
            try:
                port.reset_input_buffer()
            except Exception as exc:  # pragma: no cover - hardware-specific
                self._invalidate_port(port, f"串口清空接收缓冲失败: {exc}")
        while True:
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                break

    def _reader(self, port) -> None:
        while not self._stop_event.is_set():
            if not port.is_open:
                return
            try:
                data = port.read(port.in_waiting or 1)
                if data:
                    self.rx_queue.put(data)
            except Exception as exc:  # pragma: no cover - hardware-specific
                self._invalidate_port(port, f"串口接收失败: {exc}")
                return

    def write(self, data: bytes) -> bool:
        if not data:
            return False
        with self._state_lock:
            port = self._serial
        if port is None or not port.is_open:
            return False
        try:
            with self._write_lock:
                written = port.write(data)
        except Exception as exc:  # pragma: no cover - hardware-specific
            self._invalidate_port(port, f"串口发送失败: {exc}")
            return False
        if written != len(data):
            self._invalidate_port(
                port,
                f"串口发送不完整: 期望 {len(data)} B，实际 {written} B",
            )
            return False
        return True

    def _invalidate_port(self, port, message: str) -> None:
        """Release a device handle that failed because the USB CDC disappeared."""
        with self._state_lock:
            if self._serial is not port:
                return
            self._serial = None
        try:
            if port.is_open:
                port.close()
        except Exception:
            pass
        self.error_queue.put(message)

    def close(self) -> None:
        self._stop_event.set()
        with self._state_lock:
            port = self._serial
            self._serial = None
        if port is not None:
            try:
                if port.is_open:
                    port.close()
            except Exception:
                pass
        if (
            self._thread is not None
            and self._thread is not threading.current_thread()
            and self._thread.is_alive()
        ):
            self._thread.join(timeout=0.3)
        self._thread = None


class UpperConsole:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("H723VGT6 上层控制台")
        self.root.geometry("1280x1050")
        self.root.minsize(1180, 900)
        self.transport = SerialTransport()
        self.parser = FrameParser()
        self.sequence = 0
        self.tx_count = 0
        self.rx_count = 0
        self.active_page = "机械臂"
        self.remote_action_states: dict[str, int] = {}
        self.remote_pd9_zero_pending = False
        self.remote_pd11_after_id: str | None = None
        self.remote_pd8_first_after_id: str | None = None
        self.connection_requested = False
        self.last_heartbeat = 0.0
        self.last_board_response = 0.0
        self.last_handshake = 0.0
        self.handshake_started_at = 0.0
        self.handshake_sequence: int | None = None
        self.handshake_attempts = 0
        self.handshake_timeout_reported = False
        self.handshaken = False
        self.page_frames: dict[str, ttk.Frame] = {}
        self.page_tabs: dict[str, tk.Button] = {}
        self._make_vars()
        self._make_styles()
        self._build_ui()
        self.refresh_ports()
        self._poll_after_id = self.root.after(20, self._poll_io)
        self._send_after_id = self.root.after(SEND_PERIOD_MS, self._periodic_send)
        self.root.bind("<KeyPress-space>", self._on_space_estop, add="+")
        self.root.bind(
            "<KeyRelease-space>", self._on_space_estop_released, add="+"
        )
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _make_vars(self) -> None:
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(BAUD_RATES[0]))
        self.status_var = tk.StringVar(value="未连接")
        self.tx_var = tk.StringVar(value="TX 0")
        self.rx_var = tk.StringVar(value="RX 0")
        self.board_state_var = tk.StringVar(value="板端状态: --")
        self.remote_var = tk.StringVar(value="远程链路: --")
        self.j4310_output_position_var = tk.StringVar(value="当前输出轴角度: -- deg")
        self.dji_diagnostic_vars = {
            (1, 2, 1): tk.StringVar(
                value=format_dji_feedback(
                    "M3508 #1", "--", "等待主控反馈"
                )
            ),
            (1, 2, 2): tk.StringVar(
                value=format_dji_feedback(
                    "M3508 #2", "--", "等待主控反馈"
                )
            ),
            (2, 3, 1): tk.StringVar(
                value=format_dji_feedback(
                    "闸门 M2006", "--", "等待主控反馈"
                )
            ),
            (2, 3, 2): tk.StringVar(
                value=format_dji_feedback(
                    "夹爪 M2006", "--", "等待主控反馈"
                )
            ),
        }
        self.j_position = tk.DoubleVar(value=90.0)
        self.j_velocity = tk.DoubleVar(value=0.0)
        self.j_kp = tk.DoubleVar(value=30.0)
        self.j_kd = tk.DoubleVar(value=0.95)
        self.j_tau = tk.DoubleVar(value=0.0)
        self.j_torque_limit = tk.DoubleVar(value=10.0)
        self.m3508_position_1 = tk.DoubleVar(value=0.0)
        self.m3508_position_2 = tk.DoubleVar(value=0.0)
        self.m3508_sync_position = tk.DoubleVar(value=0.0)
        self.conveyor_position = tk.DoubleVar(value=0.0)
        self.gripper_position = tk.DoubleVar(value=0.0)
        self.m3508_speed_pid_vars = self._pid_vars(
            REFERENCE_PID_VALUES["M3508 / C620"][0]
        )
        self.m3508_position_pid_vars = self._pid_vars(
            REFERENCE_PID_VALUES["M3508 / C620"][1]
        )
        self.m2006_speed_pid_vars_by_motor = {
            "conveyor": self._pid_vars(REFERENCE_PID_VALUES["M2006 / C610"][0]),
            "gripper": self._pid_vars(REFERENCE_PID_VALUES["M2006 / C610"][0]),
        }
        self.m2006_position_pid_vars_by_motor = {
            "conveyor": self._pid_vars(REFERENCE_PID_VALUES["M2006 / C610"][1]),
            "gripper": self._pid_vars(REFERENCE_PID_VALUES["M2006 / C610"][1]),
        }
        # Keep the old attributes as aliases for callers/tests that inspect the
        # default M2006 (the conveyor) parameter set.
        self.m2006_speed_pid_vars = self.m2006_speed_pid_vars_by_motor["conveyor"]
        self.m2006_position_pid_vars = self.m2006_position_pid_vars_by_motor["conveyor"]
        self._saved_motor_parameters: dict[str, object] = {}
        self.arm_enable = tk.BooleanVar(value=True)
        self.m3508_enable = tk.BooleanVar(value=True)
        self.conveyor_enable = tk.BooleanVar(value=True)
        self.gripper_enable = tk.BooleanVar(value=True)
        # PE4/PB3 is active-low in the UART2 control frame: 1 means closed.
        self.aux_output_bits = AUX_OUTPUT_ARM_CYLINDER
        self.remote_action_angles: dict[str, dict[str, tk.DoubleVar]] = {
            "取地面块": {
                "m3508": tk.DoubleVar(value=500.0),
                "j4310": tk.DoubleVar(value=90.0),
                "m3508_second": tk.DoubleVar(value=1050.0),
                "j4310_second": tk.DoubleVar(value=90.0),
            },
            "取台阶块": {
                "m3508": tk.DoubleVar(value=0.0),
                "j4310": tk.DoubleVar(value=90.0),
                "m3508_second": tk.DoubleVar(value=850.0),
                "j4310_second": tk.DoubleVar(value=90.0),
            },
            "收块": {
                "m3508": tk.DoubleVar(value=0.0),
                "j4310": tk.DoubleVar(value=165.0),
                "m3508_second": tk.DoubleVar(value=0.0),
                "j4310_second": tk.DoubleVar(value=240.0),
            },
            "翻转/存块": {
                "m3508": tk.DoubleVar(value=0.0),
                "j4310": tk.DoubleVar(value=40.0),
                "m3508_second": tk.DoubleVar(value=0.0),
                "j4310_second": tk.DoubleVar(value=-20.0),
            },
            "闸门": {
                "angle_on": tk.DoubleVar(value=180.0),
                "angle_off": tk.DoubleVar(value=55.0),
            },
            "夹爪": {
                "angle_on": tk.DoubleVar(value=45.0),
                "angle_off": tk.DoubleVar(value=125.0),
            },
        }
        self._restore_motor_parameters()
        self.j4310_feedback_status = "未收到反馈"
        self.j4310_auto_return_status_received = False
        self.j4310_auto_return_enabled = False
        self.j4310_auto_return_available = False
        self.j4310_auto_return_active = False
        self.j4310_auto_return_stage = 0
        self.j4310_auto_return_pending: bool | None = None
        self.position_slider_controls: dict[str, tuple[tk.Widget, ...]] = {}
        self.page_action_buttons: dict[str, tuple[ttk.Button, ttk.Button]] = {}
        self.page_save_buttons: dict[str, ttk.Button] = {}
        self._space_estop_pressed = False
        self._drag_target: str | None = None
        self._active_arm_mask = 0
        self._sent_arm_values = self._values()
        self._sent_j_tau = float(self.j_tau.get())
        self._sent_j_torque_limit = float(self.j_torque_limit.get())
        self._sent_pid_values = self._pid_values()

    def _make_styles(self) -> None:
        self.root.configure(bg="#e8f6fb")
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background="#e8f6fb")
        style.configure("PageContainer.TFrame", background="#d7edf7")
        style.configure("Card.TFrame", background="#dff3fa")
        style.configure("TLabel", background="#e8f6fb", foreground="#1f3442", font=("Microsoft YaHei UI", 10))
        style.configure("Muted.TLabel", background="#e8f6fb", foreground="#4f6b79", font=("Microsoft YaHei UI", 9))
        style.configure("Card.TLabel", background="#dff3fa", foreground="#1f3442", font=("Microsoft YaHei UI", 10))
        style.configure("CardMuted.TLabel", background="#dff3fa", foreground="#4f6b79", font=("Microsoft YaHei UI", 9))
        style.configure("Title.TLabel", background="#e8f6fb", foreground="#143747", font=("Microsoft YaHei UI", 20, "bold"))
        style.configure("CardTitle.TLabel", background="#dff3fa", foreground="#1f5f6b", font=("Microsoft YaHei UI", 12, "bold"))
        style.configure("TButton", padding=(12, 7), background="#d6eff9", foreground="#143747", font=("Microsoft YaHei UI", 10))
        style.map("TButton", background=[("active", "#c7ebf9")])
        style.configure("Accent.TButton", background="#a9ddea", foreground="#124b5e", font=("Microsoft YaHei UI", 10, "bold"))
        style.map("Accent.TButton", background=[("active", "#91d2e2")])
        style.configure("Danger.TButton", background="#efb4bb", foreground="#6b1e2a", font=("Microsoft YaHei UI", 10, "bold"))
        style.map("Danger.TButton", background=[("active", "#e69aa4")])
        style.configure("AutoReturnOn.TButton", background="#bfe6c7", foreground="#176b2c", font=("Microsoft YaHei UI", 10, "bold"))
        style.map("AutoReturnOn.TButton", background=[("active", "#a9ddb5")])
        style.configure("AutoReturnOff.TButton", background="#e6e6e6", foreground="#555555", font=("Microsoft YaHei UI", 10, "bold"))
        style.map("AutoReturnOff.TButton", background=[("active", "#d4d4d4")])
        style.configure("TEntry", fieldbackground="#f7fdff", foreground="#1f3442", insertcolor="#143747")
        style.configure("TCombobox", fieldbackground="#f7fdff", foreground="#1f3442")

    def _build_ui(self) -> None:
        root = ttk.Frame(self.root, padding=18)
        root.pack(fill="both", expand=True)
        header = ttk.Frame(root)
        header.pack(fill="x", pady=(0, 14))
        ttk.Label(header, text="H723VGT6 上层控制台", style="Title.TLabel").pack(anchor="w")

        connection = ttk.Frame(root, style="Card.TFrame", padding=12)
        connection.pack(fill="x", pady=(0, 14))
        connection_controls = ttk.Frame(connection, style="Card.TFrame")
        connection_controls.pack(fill="x")
        ttk.Label(connection_controls, text="串口", style="CardTitle.TLabel").pack(side="left", padx=(0, 8))
        self.port_combo = ttk.Combobox(connection_controls, textvariable=self.port_var, state="readonly", width=28)
        self.port_combo.pack(side="left", padx=(0, 8))
        ttk.Button(connection_controls, text="刷新", command=self.refresh_ports).pack(side="left", padx=(0, 12))
        ttk.Label(connection_controls, text="波特率", style="CardTitle.TLabel").pack(side="left", padx=(0, 8))
        ttk.Combobox(connection_controls, textvariable=self.baud_var, values=[str(item) for item in BAUD_RATES], state="readonly", width=10).pack(side="left", padx=(0, 12))
        self.connect_button = ttk.Button(connection_controls, text="连接", style="Accent.TButton", command=self.toggle_connection)
        self.connect_button.pack(side="left")
        self.disconnect_button = ttk.Button(connection_controls, text="断连", command=self.disconnect)
        self.disconnect_button.pack(side="left", padx=(8, 0))
        connection_status = ttk.Frame(connection, style="Card.TFrame")
        connection_status.pack(fill="x", pady=(8, 0))
        ttk.Label(connection_status, textvariable=self.status_var, style="CardMuted.TLabel").pack(side="left")
        ttk.Label(connection_status, textvariable=self.tx_var, style="CardMuted.TLabel").pack(side="right", padx=(12, 0))
        ttk.Label(connection_status, textvariable=self.rx_var, style="CardMuted.TLabel").pack(side="right")
        ttk.Button(connection_status, text="急停", style="Danger.TButton", command=self.estop).pack(side="right", padx=(0, 22))
        self.disconnect_button.configure(state="disabled")

        status = ttk.Frame(root, style="Card.TFrame", padding=10)
        status.pack(fill="x", pady=(0, 14))
        status.columnconfigure(2, weight=1)
        ttk.Label(status, textvariable=self.board_state_var, style="CardTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.remote_var, style="CardMuted.TLabel").grid(row=0, column=1, sticky="w", padx=20)
        ttk.Label(status, text="通信与反馈超时自动停输出：已关闭", style="CardMuted.TLabel").grid(row=0, column=2, sticky="e")
        navigation = tk.Frame(root, bg="#e8f6fb")
        navigation.pack(fill="x", pady=(0, 2))
        for page in ("机械臂", "闸门", "夹爪", "遥控控制"):
            tab = tk.Button(
                navigation,
                text=page,
                width=11,
                height=1,
                padx=14,
                pady=6,
                relief="flat",
                bd=0,
                highlightthickness=0,
                font=("Microsoft YaHei UI", 10, "bold"),
                bg="#d6eff9",
                fg="#315465",
                activebackground="#c7ebf9",
                activeforeground="#143747",
                command=lambda selected_page=page: self._select_page(selected_page),
            )
            tab.pack(side="left", padx=(0, 2))
            self.page_tabs[page] = tab

        self.page_container = ttk.Frame(root, style="PageContainer.TFrame", padding=2)
        self._build_arm_page()
        self._build_single_motor_page("闸门", self.conveyor_position, "conveyor")
        self._build_single_motor_page("夹爪", self.gripper_position, "gripper")
        self._build_remote_control_page()
        self._select_page("机械臂", log_change=False)

        log_frame = ttk.Frame(root, style="Card.TFrame", padding=10, height=150)
        log_frame.pack_propagate(False)
        log_frame.pack(side="bottom", fill="x", pady=(14, 0))
        ttk.Label(log_frame, text="运行日志", style="CardTitle.TLabel").pack(anchor="w")
        self.log_text = tk.Text(log_frame, height=5, bg="#f4fcff", fg="#1e2a33", insertbackground="#143747", relief="flat", state="disabled", font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True, pady=(6, 0))
        self.page_container.pack(fill="both", expand=True)

    def _build_remote_control_page(self) -> None:
        frame, canvas = self._new_scrollable_page("遥控控制", 16)
        columns = ttk.Frame(frame)
        columns.pack(fill="both", expand=True)
        columns.columnconfigure(0, weight=1)
        columns.columnconfigure(1, weight=1)

        motor_card = ttk.Frame(columns, style="Card.TFrame", padding=14)
        motor_card.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 12))
        aux_card = ttk.Frame(columns, style="Card.TFrame", padding=14)
        aux_card.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=(0, 12))
        self._make_columns_responsive(canvas, columns, motor_card, aux_card)

        ttk.Label(
            motor_card,
            text="遥控电机动作",
            style="CardTitle.TLabel",
        ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 4))
        ttk.Label(
            motor_card,
            text="参数修改后点击“保存参数”，下次启动会自动恢复。每个动作沿用遥控器的电机目标。",
            style="CardMuted.TLabel",
            wraplength=520,
        ).grid(row=1, column=0, columnspan=6, sticky="w", pady=(0, 12))

        action_rows = (
            ("取地面块", "PD13", (("m3508", "M3508", POSITION_MIN_DEG, POSITION_MAX_DEG), ("j4310", "J4310", J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG), ("m3508_second", "M3508", POSITION_MIN_DEG, POSITION_MAX_DEG), ("j4310_second", "J4310", J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG))),
            ("取台阶块", "PD12", (("m3508", "M3508", POSITION_MIN_DEG, POSITION_MAX_DEG), ("j4310", "J4310", J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG), ("m3508_second", "M3508", POSITION_MIN_DEG, POSITION_MAX_DEG), ("j4310_second", "J4310", J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG))),
            ("收块", "PD11", (("m3508", "M3508", POSITION_MIN_DEG, POSITION_MAX_DEG), ("j4310", "J4310", J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG), ("m3508_second", "M3508", POSITION_MIN_DEG, POSITION_MAX_DEG), ("j4310_second", "J4310", J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG))),
            ("翻转/存块", "PD8", (("m3508", "M3508", POSITION_MIN_DEG, POSITION_MAX_DEG), ("j4310", "J4310", J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG), ("m3508_second", "M3508", POSITION_MIN_DEG, POSITION_MAX_DEG), ("j4310_second", "J4310", J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG))),
            ("闸门", "PD9", (("angle_on", "打开", M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG), (None, "", 0.0, 0.0), ("angle_off", "关闭", M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG), (None, "", 0.0, 0.0))),
            ("夹爪", "PD10", (("angle_on", "夹紧", M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG), (None, "", 0.0, 0.0), ("angle_off", "松开", M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG), (None, "", 0.0, 0.0))),
        )
        ttk.Label(motor_card, text="动作", style="CardMuted.TLabel").grid(row=2, column=0, sticky="w")
        ttk.Label(motor_card, text="第一次动作（角度 deg）", style="CardMuted.TLabel").grid(row=2, column=1, columnspan=2, sticky="w")
        ttk.Label(motor_card, text="第二次动作（角度 deg）", style="CardMuted.TLabel").grid(row=2, column=3, columnspan=2, sticky="w")
        ttk.Label(motor_card, text="执行", style="CardMuted.TLabel").grid(row=2, column=5, sticky="w")
        self.remote_action_buttons: dict[str, ttk.Button] = {}
        for row, (name, key_name, fields) in enumerate(action_rows, start=3):
            values = self.remote_action_angles[name]
            ttk.Label(motor_card, text=f"{name} ({key_name})", style="Card.TLabel").grid(
                row=row, column=0, sticky="w", pady=4
            )
            for column, (field_name, label, lower, upper) in zip(range(1, 5), fields):
                if field_name is None:
                    continue
                spin = self._bounded_spinbox(
                    motor_card, label, textvariable=values[field_name],
                    from_=lower, to=upper, increment=1.0, width=8,
                )
                spin.grid(row=row, column=column, sticky="ew", padx=(8 if column in (1, 3) else 4, 0), pady=4)
            button = ttk.Button(
                motor_card,
                text="执行",
                command=lambda action=name: self.execute_remote_action(action),
            )
            button.grid(row=row, column=5, sticky="ew", padx=(14, 0), pady=4)
            self.remote_action_buttons[name] = button
        for column in range(1, 5):
            motor_card.columnconfigure(column, weight=1)
        motor_card.columnconfigure(5, weight=1)
        self.remote_parameters_save_button = ttk.Button(
            motor_card, text="保存参数", style="Accent.TButton",
            command=self.save_remote_action_parameters,
        )
        self.remote_parameters_save_button.grid(
            row=9, column=0, columnspan=2, sticky="w", pady=(12, 0)
        )
        self.remote_zero_button = ttk.Button(
            motor_card, text="全部归零", command=self.return_all_motors_to_zero,
        )
        self.remote_zero_button.grid(
            row=9, column=5, sticky="ew", padx=(14, 0), pady=(12, 0),
        )

        ttk.Label(aux_card, text="气缸与电子急停", style="CardTitle.TLabel").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 4)
        )
        ttk.Label(
            aux_card,
            text="控制状态经 H723 UART5 发送完整控制包，由接收板 UART2 校验后控制输出。",
            style="CardMuted.TLabel", wraplength=430,
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 14))
        self.aux_buttons: dict[int, ttk.Button] = {}
        aux_items = (
            (AUX_OUTPUT_ARM_CYLINDER, "机械臂气缸", "PB3"),
            (AUX_OUTPUT_PUSH_CYLINDER, "推块气缸", "PB5"),
            (AUX_OUTPUT_GRIPPER_CYLINDER, "夹爪气缸", "PB7"),
            (AUX_OUTPUT_ESTOP, "电子急停", "PB8 / PB9"),
        )
        for row, (bit, label, pin) in enumerate(aux_items, start=2):
            ttk.Label(aux_card, text=f"{label} ({pin})", style="Card.TLabel").grid(
                row=row, column=0, sticky="w", pady=6
            )
            button = ttk.Button(
                aux_card, text="执行",
                command=lambda output_bit=bit: self.toggle_aux_output(output_bit),
                width=12,
            )
            button.grid(row=row, column=1, sticky="e", pady=6)
            self.aux_buttons[bit] = button
        self.aux_status_var = tk.StringVar(value="气缸/电子急停状态：全部关闭")
        ttk.Label(aux_card, textvariable=self.aux_status_var, style="CardMuted.TLabel").grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(14, 0)
        )
        aux_card.columnconfigure(0, weight=1)
        self._update_aux_buttons()
        self._bind_page_mousewheel(self.page_frames["遥控控制"], canvas)

    def save_remote_action_parameters(self) -> None:
        """Validate and persist the editable remote-action angle table."""

        ranges = {
            "m3508": (POSITION_MIN_DEG, POSITION_MAX_DEG),
            "m3508_second": (POSITION_MIN_DEG, POSITION_MAX_DEG),
            "j4310": (J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG),
            "j4310_second": (J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG),
            "angle_on": (M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG),
            "angle_off": (M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG),
        }
        errors: list[str] = []
        for action, values in self.remote_action_angles.items():
            for name, variable in values.items():
                lower, upper = ranges[name]
                value = self._read_numeric(variable)
                if not math.isfinite(value) or not lower <= value <= upper:
                    errors.append(
                        f"{action} {name}: {self._format_range(lower, upper, 'deg')}"
                    )
        if errors:
            messagebox.showerror(
                "遥控动作参数无效",
                "以下参数超出范围：\n" + "\n".join(errors),
                parent=self.root,
            )
            self.status_var.set("遥控动作参数未保存")
            return
        self._saved_motor_parameters["remote_actions"] = {
            action: {
                name: float(variable.get())
                for name, variable in variables.items()
            }
            for action, variables in self.remote_action_angles.items()
        }
        self._saved_motor_parameters["remote_actions_version"] = (
            REMOTE_ACTION_PARAMETER_VERSION
        )
        try:
            self._write_motor_parameters()
        except (OSError, TypeError, ValueError) as exc:
            self.status_var.set("遥控动作参数保存失败")
            self.log(f"遥控动作参数保存失败：{exc}")
            messagebox.showerror(
                "参数保存失败", f"无法写入参数文件：{exc}", parent=self.root
            )
            return
        self.status_var.set("遥控动作参数已保存，下次启动自动恢复")
        self.log("遥控电机动作角度参数已保存")

    def _update_aux_buttons(self) -> None:
        names = {
            AUX_OUTPUT_ARM_CYLINDER: "机械臂气缸",
            AUX_OUTPUT_PUSH_CYLINDER: "推块气缸",
            AUX_OUTPUT_GRIPPER_CYLINDER: "夹爪气缸",
            AUX_OUTPUT_ESTOP: "电子急停",
        }
        active_names = []
        for bit in self.aux_buttons:
            if bit == AUX_OUTPUT_ARM_CYLINDER:
                active = (self.aux_output_bits & bit) == 0
            else:
                active = (self.aux_output_bits & bit) != 0
            if active:
                active_names.append(names[bit])
        self.aux_status_var.set(
            "气缸/电子急停状态：" + ("、".join(active_names) if active_names else "全部关闭")
        )

    def _send_aux_control(self, output_bits: int) -> bool:
        if not self.transport.connected:
            self.status_var.set("未连接，气缸/电子急停未发送")
            return False
        if not self.handshaken:
            self.status_var.set("尚未握手，气缸/电子急停未发送")
            return False
        try:
            payload = build_aux_control_payload(output_bits)
        except ValueError:
            return False
        if self._send(MSG_AUX_CONTROL, payload):
            self.status_var.set("气缸/电子急停状态已发送")
            return True
        self.status_var.set("气缸/电子急停发送失败")
        return False

    def toggle_aux_output(self, output_bit: int) -> None:
        next_bits = self.aux_output_bits ^ output_bit
        if self._send_aux_control(next_bits):
            self.aux_output_bits = next_bits
            self._update_aux_buttons()
            if output_bit == AUX_OUTPUT_ARM_CYLINDER:
                active = (next_bits & output_bit) == 0
            else:
                active = (next_bits & output_bit) != 0
            self.log(f"辅助输出 {'打开' if active else '关闭'}: 0x{output_bit:02X}")

    def _reset_remote_action_state(self, action: str) -> None:
        self.remote_action_states[action] = 0
        if action == "翻转/存块":
            self.remote_pd9_zero_pending = False
        button = self.remote_action_buttons.get(action)
        if button is not None:
            button.configure(text="执行")

    def _cancel_remote_arm_delays(self) -> None:
        for attribute in (
            "remote_pd11_after_id",
            "remote_pd8_first_after_id",
        ):
            after_id = getattr(self, attribute)
            if after_id is None:
                continue
            try:
                self.root.after_cancel(after_id)
            except tk.TclError:
                pass
            setattr(self, attribute, None)

    def _complete_remote_arm_j4310_delay(
        self, action: str, state_text: str, values: tuple[float, ...]
    ) -> None:
        if action == "收块":
            self.remote_pd11_after_id = None
        else:
            self.remote_pd8_first_after_id = None
        if not self._send_position_values(
            "机械臂", values, True, enable_mask=ENABLE_J4310_ONLY
        ):
            self.status_var.set(f"{action} {state_text}：J4310 目标发送失败")
            self.log(f"{action} {state_text}：延时 J4310 目标发送失败")
            return
        self._sent_arm_values = values
        self._active_arm_mask = ENABLE_J4310_ONLY
        self.status_var.set(f"{action} {state_text}已完成")
        self.log(f"{action} {state_text}：M3508 动作 0.5 秒后，J4310 目标已发送")

    def _reset_remote_sequences_for(self, action: str) -> None:
        arm_actions = ("取地面块", "取台阶块", "收块", "翻转/存块")
        two_stage_actions = ("取地面块", "取台阶块", "收块", "翻转/存块")
        if action not in arm_actions:
            return
        for name in two_stage_actions:
            if name != action:
                self._reset_remote_action_state(name)

    def execute_remote_action(self, action: str) -> bool:
        arm_actions = ("取地面块", "取台阶块", "收块", "翻转/存块")
        two_stage_actions = ("取地面块", "取台阶块", "收块", "翻转/存块")
        if action in arm_actions:
            self._cancel_remote_arm_delays()
        values = list(self._values())
        params = self.remote_action_angles[action]
        updates_arm_snapshot = False
        delayed_arm_action: str | None = None
        delayed_arm_values: tuple[float, ...] | None = None
        pd9_zero_action = False
        if action in two_stage_actions:
            active = bool(self.remote_action_states.get(action, False))
            m3508_name = "m3508_second" if active else "m3508"
            j4310_name = "j4310_second" if active else "j4310"
            m3508_value = self._read_numeric(params[m3508_name])
            j4310_value = self._read_numeric(params[j4310_name])
            if action == "收块" or (action == "翻转/存块" and not active):
                values = list(self._sent_arm_values)
                values[4] = m3508_value
                values[5] = m3508_value
                delayed_values = list(values)
                delayed_values[0] = j4310_value
                delayed_arm_action = action
                delayed_arm_values = tuple(delayed_values)
                mask = ENABLE_M3508_ONLY
            else:
                values[0] = j4310_value
                values[4] = m3508_value
                values[5] = m3508_value
                mask = ENABLE_J4310_ONLY | ENABLE_M3508_ONLY
            page = "机械臂"
            next_state = int(not active)
            state_text = "第二次动作" if active else "第一次动作"
            updates_arm_snapshot = True
        elif action == "闸门":
            active = bool(self.remote_action_states.get(action, False))
            pd9_zero_action = self.remote_pd9_zero_pending
            values[6] = (
                REMOTE_PD9_ZERO_GATE_DEG
                if pd9_zero_action
                else self._read_numeric(
                    params["angle_on" if not active else "angle_off"]
                )
            )
            page = "闸门"
            mask = ENABLE_CONVEYOR
            next_state = 0 if pd9_zero_action else int(not active)
            state_text = (
                "PD8 第一次动作后的回零"
                if pd9_zero_action
                else ("第二段" if not active else "复位段")
            )
        else:
            active = bool(self.remote_action_states.get(action, False))
            values[7] = self._read_numeric(params["angle_on" if not active else "angle_off"])
            page = "夹爪"
            mask = ENABLE_GRIPPER
            next_state = int(not active)
            state_text = "第二段" if not active else "复位段"
        if not self._send_position_values(page, tuple(values), True, enable_mask=mask):
            return False
        self._reset_remote_sequences_for(action)
        self.remote_action_states[action] = next_state
        if action == "翻转/存块":
            self.remote_pd9_zero_pending = not active
            if active:
                self._reset_remote_action_state("闸门")
        elif action == "闸门" and pd9_zero_action:
            self.remote_pd9_zero_pending = False
        elif action not in ("翻转/存块", "闸门"):
            self.remote_pd9_zero_pending = False
        if updates_arm_snapshot:
            sent_values = tuple(values)
            self._sent_arm_values = sent_values
            self._active_arm_mask = mask
        button = self.remote_action_buttons.get(action)
        if button is not None:
            if action in two_stage_actions:
                button_text = "第二次动作" if next_state else "执行"
            else:
                button_text = "复位/第二次动作" if next_state else "执行"
            button.configure(text=button_text)
        if delayed_arm_action is not None and delayed_arm_values is not None:
            after_id = self.root.after(
                REMOTE_PD11_DELAY_MS
                if delayed_arm_action == "收块"
                else REMOTE_PD8_FIRST_DELAY_MS,
                self._complete_remote_arm_j4310_delay,
                delayed_arm_action,
                state_text,
                delayed_arm_values,
            )
            if delayed_arm_action == "收块":
                self.remote_pd11_after_id = after_id
            else:
                self.remote_pd8_first_after_id = after_id
            self.status_var.set(
                f"{action} {state_text}：M3508 已发送，0.5 秒后进行 J4310 动作"
            )
            self.log(f"{action} {state_text}：先发送 M3508，0.5 秒后发送 J4310")
        else:
            self.status_var.set(f"{action} 已执行")
            self.log(f"遥控动作 {action} 已执行，状态={state_text}")
        return True

    def return_all_motors_to_zero(self) -> bool:
        """Send one absolute-position snapshot that holds every motor at zero."""

        self._cancel_remote_arm_delays()
        values = (
            0.0,
            0.0,
            self._sent_arm_values[2],
            self._sent_arm_values[3],
            0.0,
            0.0,
            0.0,
            0.0,
        )
        enable_mask = (
            ENABLE_J4310_ONLY
            | ENABLE_M3508_ONLY
            | ENABLE_CONVEYOR
            | ENABLE_GRIPPER
        )
        if not self._send_position_values(
            "机械臂",
            values,
            True,
            enable_mask=enable_mask,
            j_tau=0.0,
            j_torque_limit=self._sent_j_torque_limit,
            pid_values=self._sent_pid_values,
            include_pid=False,
        ):
            return False

        self.j_position.set(0.0)
        self.j_velocity.set(0.0)
        self.m3508_position_1.set(0.0)
        self.m3508_position_2.set(0.0)
        self.m3508_sync_position.set(0.0)
        self.conveyor_position.set(0.0)
        self.gripper_position.set(0.0)
        self._sent_arm_values = values
        self._sent_j_tau = 0.0
        self._active_arm_mask = ENABLE_J4310_ONLY | ENABLE_M3508_ONLY
        for variable in (
            self.arm_enable,
            self.m3508_enable,
            self.conveyor_enable,
            self.gripper_enable,
        ):
            variable.set(True)
        self.remote_action_states.clear()
        self.remote_pd9_zero_pending = False
        for button in self.remote_action_buttons.values():
            button.configure(text="执行")
        self.status_var.set("全部电机归零目标已发送")
        self.log("遥控动作：全部电机已下发 0 deg 归零目标")
        return True

    def _new_scrollable_page(self, title: str, padding: int) -> tuple[ttk.Frame, tk.Canvas]:
        wrapper = ttk.Frame(self.page_container)
        canvas = tk.Canvas(
            wrapper,
            bg="#e8f6fb",
            highlightthickness=0,
            borderwidth=0,
        )
        scrollbar = ttk.Scrollbar(wrapper, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        content = ttk.Frame(canvas, padding=padding)
        window = canvas.create_window((0, 0), window=content, anchor="nw")
        content.bind(
            "<Configure>",
            lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.bind(
            "<Configure>",
            lambda event: canvas.itemconfigure(window, width=event.width),
        )
        self.page_frames[title] = wrapper
        return content, canvas

    @staticmethod
    def _bind_page_mousewheel(widget: tk.Misc, canvas: tk.Canvas) -> None:
        def scroll(event: tk.Event) -> str | None:
            delta = int(getattr(event, "delta", 0))
            if delta == 0:
                return None
            canvas.yview_scroll(-1 if delta > 0 else 1, "units")
            return "break"

        widget.bind("<MouseWheel>", scroll, add="+")
        for child in widget.winfo_children():
            UpperConsole._bind_page_mousewheel(child, canvas)

    @staticmethod
    def _make_columns_responsive(
        canvas: tk.Canvas,
        columns: ttk.Frame,
        left: ttk.Frame,
        right: ttk.Frame,
    ) -> None:
        def arrange(event: tk.Event) -> None:
            if event.width < 1450:
                columns.columnconfigure(0, weight=1)
                columns.columnconfigure(1, weight=0)
                left.grid_configure(row=0, column=0, padx=0, pady=(0, 12))
                right.grid_configure(row=1, column=0, padx=0, pady=0)
            else:
                columns.columnconfigure(0, weight=3)
                columns.columnconfigure(1, weight=4)
                left.grid_configure(row=0, column=0, padx=(0, 12), pady=0)
                right.grid_configure(row=0, column=1, padx=0, pady=0)

        canvas.bind("<Configure>", arrange, add="+")

    def _build_arm_page(self) -> None:
        frame, canvas = self._new_scrollable_page("机械臂", 12)
        columns = ttk.Frame(frame)
        columns.pack(fill="both", expand=True)
        columns.columnconfigure(0, weight=3)
        columns.columnconfigure(1, weight=4)
        left = ttk.Frame(columns, style="Card.TFrame", padding=14)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        left.columnconfigure(1, weight=1)
        left.columnconfigure(4, weight=1)
        right = ttk.Frame(columns, style="Card.TFrame", padding=14)
        right.grid(row=0, column=1, sticky="nsew")
        self._make_columns_responsive(canvas, columns, left, right)
        ttk.Label(left, text="J4310 · MIT 控制", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
        ttk.Button(
            left,
            text="参数范围",
            command=self.show_parameter_ranges,
        ).grid(row=0, column=3, columnspan=3, sticky="e", pady=(0, 10))
        self._field_at(left, 1, 0, "前馈 tau (t_ff)", self.j_tau, "Nm", -J4310_TORQUE_LIMIT, J4310_TORQUE_LIMIT, 0.05)
        self._field_at(left, 1, 3, "力矩限幅", self.j_torque_limit, "Nm", 0.1, J4310_TORQUE_LIMIT, 0.05)
        self._field_at(left, 2, 0, "刚度 Kp", self.j_kp, "", 0, J4310_KP_LIMIT, 1)
        self._field_at(left, 2, 3, "阻尼 Kd", self.j_kd, "", 0, J4310_KD_LIMIT, 0.05)
        self.j_position_controls = self._angle_field_at(
            left,
            3,
            0,
            "目标位置 p_des",
            self.j_position,
            J4310_POSITION_MIN_DEG,
            J4310_POSITION_MAX_DEG,
            "j4310",
        )
        self.position_slider_controls["j4310"] = self.j_position_controls
        self._field_at(left, 3, 3, "目标速度 v_des", self.j_velocity, "rad/s", -J4310_VELOCITY_LIMIT, J4310_VELOCITY_LIMIT, 0.1)
        self.j4310_bus_label = ttk.Label(
            left,
            text="反馈状态: 未收到反馈",
            style="CardMuted.TLabel",
            justify="left",
        )
        self.j4310_bus_label.grid(
            row=4,
            column=0,
            columnspan=6,
            sticky="w",
            pady=(6, 3),
        )
        actions = ttk.Frame(left, style="Card.TFrame")
        actions.grid(row=5, column=0, columnspan=6, sticky="ew", pady=(3, 8))
        self.j4310_send_button = ttk.Button(
            actions,
            text="发送目标",
            command=lambda: self.send_arm_group_now("j4310"),
        )
        self.j4310_send_button.pack(side="left", padx=(0, 8))
        self.j4310_stop_button = ttk.Button(
            actions,
            text="停止发送",
            command=lambda: self.stop_arm_group("j4310"),
        )
        self.j4310_stop_button.pack(side="left", padx=(0, 8))
        self.j4310_save_button = ttk.Button(
            actions,
            text="保存参数",
            command=lambda: self.save_motor_parameters("j4310"),
        )
        self.j4310_save_button.pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="永久标定", command=self.permanently_calibrate_j4310).pack(side="left", padx=(0, 8))
        self.j4310_auto_return_button = ttk.Button(
            actions,
            text="重启归零：--",
            command=self.toggle_j4310_auto_return,
            state="disabled",
            width=18,
        )
        self.j4310_auto_return_button.pack(side="left", padx=(0, 10))
        self._update_j4310_auto_return_button()
        ttk.Label(actions, textvariable=self.j4310_output_position_var, style="CardMuted.TLabel").pack(side="left")
        ttk.Label(left, text="M3508 × 2 · 位置目标", style="CardTitle.TLabel").grid(row=6, column=0, columnspan=6, sticky="w", pady=(4, 4))
        self.m3508_position_1_spinbox = self._angle_input_at(
            left,
            7,
            0,
            "M3508 #1",
            self.m3508_position_1,
            include_sync_input=True,
        )
        self.m3508_position_2_spinbox = self._angle_input_at(
            left, 8, 0, "M3508 #2", self.m3508_position_2
        )
        self.m3508_sync_slider = self._sync_angle_slider_at(
            left, 9, 0, "同步滑块"
        )
        self.m3508_position_controls = (
            self.m3508_position_1_spinbox,
            self.m3508_position_2_spinbox,
            self.m3508_sync_input_spinbox,
            self.m3508_sync_slider,
        )
        self.position_slider_controls["m3508"] = self.m3508_position_controls
        ttk.Label(left, text="FDCAN2 · ID 1 / ID 2", style="CardMuted.TLabel").grid(row=7, column=3, columnspan=3, rowspan=2, sticky="w", padx=(18, 0))
        m3508_actions = ttk.Frame(left, style="Card.TFrame")
        m3508_actions.grid(
            row=10, column=0, columnspan=6, sticky="w", pady=(7, 0)
        )
        self.m3508_send_button = ttk.Button(
            m3508_actions,
            text="发送目标",
            command=lambda: self.send_arm_group_now("m3508"),
        )
        self.m3508_send_button.pack(side="left", padx=(0, 8))
        self.m3508_stop_button = ttk.Button(
            m3508_actions,
            text="停止发送",
            command=lambda: self.stop_arm_group("m3508"),
        )
        self.m3508_stop_button.pack(side="left", padx=(0, 8))
        self.m3508_save_button = ttk.Button(
            m3508_actions,
            text="保存参数",
            command=lambda: self.save_motor_parameters("m3508"),
        )
        self.m3508_save_button.pack(side="left")
        m3508_feedback = ttk.Frame(left, style="Card.TFrame")
        m3508_feedback.grid(
            row=11, column=0, columnspan=6, sticky="ew", pady=(8, 0)
        )
        m3508_feedback.columnconfigure(0, weight=1, uniform="m3508_feedback")
        m3508_feedback.columnconfigure(1, weight=1, uniform="m3508_feedback")
        self.m3508_feedback_labels = (
            ttk.Label(
                m3508_feedback,
                textvariable=self.dji_diagnostic_vars[(1, 2, 1)],
                style="CardMuted.TLabel",
                justify="left",
            ),
            ttk.Label(
                m3508_feedback,
                textvariable=self.dji_diagnostic_vars[(1, 2, 2)],
                style="CardMuted.TLabel",
                justify="left",
            ),
        )
        self.m3508_feedback_labels[0].grid(
            row=0, column=0, sticky="nw", padx=(0, 12)
        )
        self.m3508_feedback_labels[1].grid(
            row=0, column=1, sticky="nw", padx=(12, 0)
        )
        self._build_pid_editor(
            right,
            "M3508 / C620 · PID 参数",
            self.m3508_speed_pid_vars,
            self.m3508_position_pid_vars,
        )
        self._bind_page_mousewheel(self.page_frames["机械臂"], canvas)

    def _build_single_motor_page(self, title: str, position_var: tk.DoubleVar, key: str) -> None:
        frame, canvas = self._new_scrollable_page(title, 16)
        columns = ttk.Frame(frame)
        columns.pack(fill="both", expand=True)
        columns.columnconfigure(0, weight=3)
        columns.columnconfigure(1, weight=4)
        left = ttk.Frame(columns, style="Card.TFrame", padding=14)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        right = ttk.Frame(columns, style="Card.TFrame", padding=14)
        right.grid(row=0, column=1, sticky="nsew")
        self._make_columns_responsive(canvas, columns, left, right)
        node_id = M2006_NODE_IDS[key]
        ttk.Label(left, text="M2006 · 单电机位置控制", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))
        self.position_slider_controls[key] = self._angle_field(
            left,
            1,
            "输出轴目标位置",
            position_var,
            M2006_POSITION_MIN_DEG,
            M2006_POSITION_MAX_DEG,
            key,
        )
        ttk.Label(left, text=f"FDCAN3 · ID {node_id} · {title}", style="CardMuted.TLabel").grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 12))
        buttons = ttk.Frame(left, style="Card.TFrame")
        buttons.grid(row=3, column=0, columnspan=3, sticky="w", pady=(4, 0))
        self._add_page_buttons(buttons, title, key, "M2006")
        diagnostic_key = (2, 3, node_id)
        ttk.Label(
            left,
            textvariable=self.dji_diagnostic_vars[diagnostic_key],
            style="CardMuted.TLabel",
            wraplength=440,
        ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(10, 0))
        self._build_pid_editor(
            right,
            "M2006 / C610 · PID 参数",
            self.m2006_speed_pid_vars_by_motor[key],
            self.m2006_position_pid_vars_by_motor[key],
        )
        self._bind_page_mousewheel(self.page_frames[title], canvas)

    def _field(self, parent: ttk.Frame, row: int, label: str, variable: tk.DoubleVar, unit: str, lower: float, upper: float, increment: float) -> None:
        self._field_at(parent, row, 0, label, variable, unit, lower, upper, increment)

    def _angle_field(
        self,
        parent: ttk.Frame,
        row: int,
        label: str,
        variable: tk.DoubleVar,
        lower: float,
        upper: float,
        drag_target: str,
    ) -> tuple[ttk.Spinbox, ttk.Scale]:
        return self._angle_field_at(
            parent, row, 0, label, variable, lower, upper, drag_target
        )

    def _angle_field_at(
        self,
        parent: ttk.Frame,
        row: int,
        column: int,
        label: str,
        variable: tk.DoubleVar,
        lower: float,
        upper: float,
        drag_target: str,
    ) -> tuple[ttk.Spinbox, ttk.Scale]:
        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row,
            column=column,
            sticky="w",
            padx=(18 if column > 0 else 0, 0),
            pady=4,
        )
        controls = ttk.Frame(parent, style="Card.TFrame")
        controls.grid(row=row, column=column + 1, sticky="ew", padx=8, pady=4)
        spin = self._bounded_spinbox(
            controls,
            "目标位置",
            textvariable=variable,
            from_=lower,
            to=upper,
            increment=1.0,
            width=7,
        )
        spin.pack(side="left")
        scale = ttk.Scale(
            controls,
            variable=variable,
            from_=lower,
            to=upper,
            orient="horizontal",
            length=140,
            command=lambda value, target=variable, drag=drag_target:
                self._on_slider_value(drag, target, value),
        )
        scale.pack(side="left", fill="x", expand=True, padx=(8, 0))
        scale.bind(
            "<ButtonPress-1>",
            lambda _event, target=drag_target: self._start_slider_drag(target),
        )
        scale.bind(
            "<ButtonRelease-1>",
            lambda _event, target=drag_target: self._finish_slider_drag(target),
        )
        ttk.Label(parent, text="deg", style="CardMuted.TLabel").grid(
            row=row,
            column=column + 2,
            sticky="w",
            padx=(0, 18),
            pady=4,
        )
        return spin, scale

    def _on_slider_value(
        self, drag_target: str, variable: tk.DoubleVar, value: str
    ) -> None:
        variable.set(round(float(value), 1))

    def _angle_input_at(
        self,
        parent: ttk.Frame,
        row: int,
        column: int,
        label: str,
        variable: tk.DoubleVar,
        include_sync_input: bool = False,
    ) -> ttk.Spinbox:
        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row, column=column, sticky="w", pady=4
        )
        controls = ttk.Frame(parent, style="Card.TFrame")
        controls.grid(
            row=row,
            column=column + 1,
            sticky="ew",
            padx=8,
            pady=4,
        )
        spinbox = self._bounded_spinbox(
            controls,
            label,
            textvariable=variable,
            from_=POSITION_MIN_DEG,
            to=POSITION_MAX_DEG,
            increment=1.0,
            width=7,
        )
        spinbox.pack(side="left")
        if include_sync_input:
            ttk.Label(
                controls,
                text="同步输入",
                style="CardMuted.TLabel",
            ).pack(side="left", padx=(24, 8))
            self.m3508_sync_input_spinbox = self._bounded_spinbox(
                controls,
                "M3508 同步输入",
                textvariable=self.m3508_sync_position,
                from_=M3508_SYNC_POSITION_MIN_DEG,
                to=M3508_SYNC_POSITION_MAX_DEG,
                increment=1.0,
                width=7,
            )
            self.m3508_sync_input_spinbox.configure(
                command=self._on_m3508_sync_input_value
            )
            for event_name in ("<KeyRelease>", "<Return>", "<FocusOut>"):
                self.m3508_sync_input_spinbox.bind(
                    event_name,
                    self._on_m3508_sync_input_value,
                    add="+",
                )
            self.m3508_sync_input_spinbox.pack(side="left")
        ttk.Label(parent, text="deg", style="CardMuted.TLabel").grid(
            row=row, column=column + 2, sticky="w", padx=(0, 18), pady=4
        )
        return spinbox

    def _sync_angle_slider_at(
        self, parent: ttk.Frame, row: int, column: int, label: str
    ) -> ttk.Scale:
        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row, column=column, sticky="w", pady=4
        )
        scale = ttk.Scale(
            parent,
            variable=self.m3508_sync_position,
            from_=M3508_SYNC_POSITION_MIN_DEG,
            to=M3508_SYNC_POSITION_MAX_DEG,
            orient="horizontal",
            length=224,
            command=self._on_m3508_sync_slider_value,
        )
        scale.grid(
            row=row,
            column=column + 1,
            columnspan=2,
            sticky="ew",
            padx=(8, 18),
            pady=4,
        )
        scale.bind(
            "<ButtonPress-1>",
            lambda _event: self._start_slider_drag("m3508"),
        )
        scale.bind(
            "<ButtonRelease-1>",
            lambda _event: self._finish_slider_drag("m3508"),
        )
        return scale

    def _on_m3508_sync_slider_value(self, value: str) -> None:
        position = round(float(value), 1)
        self.m3508_sync_position.set(position)
        self._set_m3508_sync_targets(position)

    def _on_m3508_sync_input_value(self, _event: tk.Event | None = None) -> None:
        position = self._read_numeric(self.m3508_sync_position)
        if (
            math.isfinite(position)
            and M3508_SYNC_POSITION_MIN_DEG
            <= position
            <= M3508_SYNC_POSITION_MAX_DEG
        ):
            self._set_m3508_sync_targets(position)

    def _set_m3508_sync_targets(self, position: float) -> None:
        self.m3508_position_1.set(position)
        self.m3508_position_2.set(position)

    def _field_at(self, parent: ttk.Frame, row: int, column: int, label: str, variable: tk.DoubleVar, unit: str, lower: float, upper: float, increment: float) -> None:
        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row,
            column=column,
            sticky="w",
            padx=(18 if column > 0 else 0, 0),
            pady=4,
        )
        spin = self._bounded_spinbox(
            parent,
            label,
            textvariable=variable,
            from_=lower,
            to=upper,
            increment=increment,
            width=16,
        )
        spin.grid(row=row, column=column + 1, sticky="ew", padx=8, pady=4)
        ttk.Label(parent, text=unit, style="CardMuted.TLabel").grid(
            row=row,
            column=column + 2,
            sticky="w",
            padx=(0, 18),
            pady=4,
        )

    def _pid_vars(
        self, values: tuple[str, str, str, str, str, str]
    ) -> dict[str, tk.DoubleVar]:
        return {
            "kp": tk.DoubleVar(value=float(values[1])),
            "ki": tk.DoubleVar(value=float(values[2])),
            "kd": tk.DoubleVar(value=float(values[3])),
            "integral_limit": tk.DoubleVar(value=float(values[4])),
            "output_limit": tk.DoubleVar(value=float(values[5].split()[0])),
        }

    @staticmethod
    def _pid_snapshot(variables: dict[str, tk.DoubleVar]) -> dict[str, float]:
        return {
            name: float(variables[name].get())
            for name in ("kp", "ki", "kd", "integral_limit", "output_limit")
        }

    @staticmethod
    def _restore_number(
        section: object,
        name: str,
        variable: tk.DoubleVar,
        lower: float,
        upper: float,
    ) -> None:
        if not isinstance(section, dict) or name not in section:
            return
        try:
            value = float(section[name])
        except (TypeError, ValueError):
            return
        if math.isfinite(value) and lower <= value <= upper:
            variable.set(value)

    def _restore_pid_group(
        self,
        section: object,
        variables: dict[str, tk.DoubleVar],
        position: bool,
    ) -> None:
        upper_output = (
            PID_POSITION_OUTPUT_LIMIT_MAX
            if position
            else PID_SPEED_OUTPUT_LIMIT_MAX
        )
        for name, lower, upper in (
            ("kp", PID_GAIN_MIN, PID_KP_KI_MAX),
            ("ki", PID_GAIN_MIN, PID_KP_KI_MAX),
            ("kd", PID_GAIN_MIN, PID_KD_MAX),
            ("integral_limit", PID_GAIN_MIN, PID_INTEGRAL_LIMIT_MAX),
            ("output_limit", PID_OUTPUT_LIMIT_MIN, upper_output),
        ):
            self._restore_number(section, name, variables[name], lower, upper)

    def _restore_motor_parameters(self) -> None:
        try:
            with MOTOR_PARAMETER_PATH.open("r", encoding="utf-8") as stream:
                data = json.load(stream)
        except (OSError, ValueError, TypeError):
            data = {}
        if not isinstance(data, dict):
            data = {}
        self._saved_motor_parameters = data

        j4310 = data.get("j4310")
        self._restore_number(j4310, "position_deg", self.j_position, J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG)
        self._restore_number(j4310, "velocity_rad_s", self.j_velocity, -J4310_VELOCITY_LIMIT, J4310_VELOCITY_LIMIT)
        self._restore_number(j4310, "kp", self.j_kp, 0.0, J4310_KP_LIMIT)
        self._restore_number(j4310, "kd", self.j_kd, 0.0, J4310_KD_LIMIT)
        self._restore_number(j4310, "tau_nm", self.j_tau, -J4310_TORQUE_LIMIT, J4310_TORQUE_LIMIT)
        self._restore_number(j4310, "torque_limit_nm", self.j_torque_limit, 0.1, J4310_TORQUE_LIMIT)

        remote_actions = data.get("remote_actions")
        if isinstance(remote_actions, dict):
            remote_actions_version = data.get("remote_actions_version", 1)
            for action, variables in self.remote_action_angles.items():
                if remote_actions_version != REMOTE_ACTION_PARAMETER_VERSION:
                    if action == "闸门":
                        continue
                    if action == "夹爪" and (
                        not isinstance(remote_actions_version, int)
                        or remote_actions_version < 9
                    ):
                        continue
                    if action == "收块" and (
                        not isinstance(remote_actions_version, int)
                        or remote_actions_version < 8
                    ):
                        continue
                    if (not isinstance(remote_actions_version, int) or
                            remote_actions_version < 4) and action != "夹爪":
                        continue
                section = remote_actions.get(action)
                if (action == "闸门" and
                        remote_actions_version == REMOTE_ACTION_PARAMETER_VERSION and
                        not isinstance(section, dict)):
                    section = remote_actions.get(LEGACY_GATE_ACTION_KEY)
                if action == "收块" and not isinstance(section, dict):
                    section = remote_actions.get("保留块")
                if not isinstance(section, dict):
                    continue
                for name, variable in variables.items():
                    if name.startswith("j4310"):
                        lower, upper = J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG
                    elif name.startswith("m3508"):
                        lower, upper = POSITION_MIN_DEG, POSITION_MAX_DEG
                    else:
                        lower, upper = M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG
                    self._restore_number(section, name, variable, lower, upper)

        m3508 = data.get("m3508")
        self._restore_number(m3508, "position_1_deg", self.m3508_position_1, *M3508_TARGET_RANGES_DEG[0])
        self._restore_number(m3508, "position_2_deg", self.m3508_position_2, *M3508_TARGET_RANGES_DEG[1])
        self._restore_number(m3508, "sync_position_deg", self.m3508_sync_position, M3508_SYNC_POSITION_MIN_DEG, M3508_SYNC_POSITION_MAX_DEG)
        self._restore_pid_group(m3508.get("speed_pid") if isinstance(m3508, dict) else None, self.m3508_speed_pid_vars, False)
        self._restore_pid_group(m3508.get("position_pid") if isinstance(m3508, dict) else None, self.m3508_position_pid_vars, True)

        for key, position_var in (
            ("conveyor", self.conveyor_position),
            ("gripper", self.gripper_position),
        ):
            section = data.get(key)
            self._restore_number(section, "position_deg", position_var, M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG)
            self._restore_pid_group(
                section.get("speed_pid") if isinstance(section, dict) else None,
                self.m2006_speed_pid_vars_by_motor[key],
                False,
            )
            self._restore_pid_group(
                section.get("position_pid") if isinstance(section, dict) else None,
                self.m2006_position_pid_vars_by_motor[key],
                True,
            )

    def _motor_parameter_snapshot(self, target: str) -> dict[str, object]:
        if target == "j4310":
            return {
                "position_deg": float(self.j_position.get()),
                "velocity_rad_s": float(self.j_velocity.get()),
                "kp": float(self.j_kp.get()),
                "kd": float(self.j_kd.get()),
                "tau_nm": float(self.j_tau.get()),
                "torque_limit_nm": float(self.j_torque_limit.get()),
            }
        if target == "m3508":
            return {
                "position_1_deg": float(self.m3508_position_1.get()),
                "position_2_deg": float(self.m3508_position_2.get()),
                "sync_position_deg": float(self.m3508_sync_position.get()),
                "speed_pid": self._pid_snapshot(self.m3508_speed_pid_vars),
                "position_pid": self._pid_snapshot(self.m3508_position_pid_vars),
            }
        if target not in self.m2006_speed_pid_vars_by_motor:
            raise ValueError(f"unknown motor parameter target: {target}")
        position_var = self.conveyor_position if target == "conveyor" else self.gripper_position
        return {
            "position_deg": float(position_var.get()),
            "speed_pid": self._pid_snapshot(self.m2006_speed_pid_vars_by_motor[target]),
            "position_pid": self._pid_snapshot(self.m2006_position_pid_vars_by_motor[target]),
        }

    def _write_motor_parameters(self) -> None:
        with MOTOR_PARAMETER_PATH.open("w", encoding="utf-8") as stream:
            json.dump(self._saved_motor_parameters, stream, indent=2, ensure_ascii=False)

    def save_motor_parameters(self, target: str) -> bool:
        page = "机械臂" if target in ("j4310", "m3508") else ("闸门" if target == "conveyor" else "夹爪")
        values = self._values()
        options: dict[str, object] = {"pid_values": self._pid_values(page)}
        if target == "j4310":
            options.update(
                enable_mask=ENABLE_J4310_ONLY,
                j_tau=self._read_numeric(self.j_tau),
                j_torque_limit=self._read_numeric(self.j_torque_limit),
            )
        elif target == "m3508":
            options["enable_mask"] = ENABLE_M3508_ONLY
        if not self._validate_send_parameters(page, values, True, options):
            return False
        try:
            self._saved_motor_parameters[target] = self._motor_parameter_snapshot(target)
            self._write_motor_parameters()
        except (OSError, TypeError, ValueError) as exc:
            self.status_var.set("参数保存失败")
            self.log(f"{target} 参数保存失败：{exc}")
            messagebox.showerror("参数保存失败", f"无法写入参数文件：{exc}", parent=self.root)
            return False

        applied = (
            self.send_arm_group_now(target)
            if target in ("j4310", "m3508")
            else self.send_page_now(page)
        )
        if applied:
            self.status_var.set(f"{page} 参数已保存并立即应用")
            self.log(f"{page} 参数已保存并立即应用")
        else:
            self.status_var.set(f"{page} 参数已保存，连接后发送目标时应用")
            self.log(f"{page} 参数已保存；当前未连接或尚未握手，暂未下发")
        return True

    def _build_pid_editor(
        self,
        parent: ttk.Frame,
        title: str,
        speed_vars: dict[str, tk.DoubleVar],
        position_vars: dict[str, tk.DoubleVar],
    ) -> None:
        ttk.Label(parent, text=title, style="CardTitle.TLabel").grid(
            row=0, column=0, columnspan=6, sticky="w", pady=(0, 4)
        )
        ttk.Label(
            parent,
            text="参数只在显式发送目标时下发，上电和复位不会自动应用。",
            style="CardMuted.TLabel",
        ).grid(row=1, column=0, columnspan=6, sticky="w", pady=(0, 12))
        for column, header in enumerate(
            ("环路", "Kp", "Ki", "Kd", "积分限幅", "输出限幅")
        ):
            ttk.Label(parent, text=header, style="CardMuted.TLabel").grid(
                row=2, column=column, sticky="w", padx=(0, 8), pady=(0, 4)
            )
        self._pid_row(parent, 3, "速度环", speed_vars, position=False)
        self._pid_row(parent, 4, "位置环", position_vars, position=True)

    def _pid_row(
        self,
        parent: ttk.Frame,
        row: int,
        title: str,
        variables: dict[str, tk.DoubleVar],
        position: bool,
    ) -> None:
        ttk.Label(parent, text=title, style="Card.TLabel").grid(
            row=row, column=0, sticky="w", pady=4
        )
        specs = (
            ("kp", PID_GAIN_MIN, PID_KP_KI_MAX, 1.0),
            ("ki", PID_GAIN_MIN, PID_KP_KI_MAX, 1.0),
            ("kd", PID_GAIN_MIN, PID_KD_MAX, 0.1),
            ("integral_limit", PID_GAIN_MIN, PID_INTEGRAL_LIMIT_MAX, 0.01),
            ("output_limit", PID_OUTPUT_LIMIT_MIN,
             PID_POSITION_OUTPUT_LIMIT_MAX if position else
             PID_SPEED_OUTPUT_LIMIT_MAX,
             0.1 if position else 1.0),
        )
        for column, (name, lower, upper, increment) in enumerate(
            specs, start=1
        ):
            spin = self._bounded_spinbox(
                parent,
                f"{title} {name}",
                textvariable=variables[name],
                from_=lower,
                to=upper,
                increment=increment,
                width=9,
            )
            spin.grid(row=row, column=column, sticky="w", padx=(0, 8), pady=4)

    def _bounded_spinbox(
        self,
        parent: ttk.Frame,
        label: str,
        *,
        textvariable: tk.DoubleVar,
        from_: float,
        to: float,
        increment: float,
        width: int,
    ) -> ttk.Spinbox:
        range_text = self._format_range(from_, to)
        validate_command = (
            self.root.register(self._validate_numeric_edit),
            "%P",
            str(from_),
            str(to),
        )
        invalid_command = (
            self.root.register(self._reject_numeric_edit),
            label,
            range_text,
        )
        return ttk.Spinbox(
            parent,
            textvariable=textvariable,
            from_=from_,
            to=to,
            increment=increment,
            width=width,
            validate="key",
            validatecommand=validate_command,
            invalidcommand=invalid_command,
        )

    @staticmethod
    def _validate_numeric_edit(proposed: str, lower: str, upper: str) -> bool:
        if proposed in ("", "-", ".", "-."):
            return True
        try:
            value = float(proposed)
        except ValueError:
            return False
        return math.isfinite(value) and float(lower) <= value <= float(upper)

    def _reject_numeric_edit(self, label: str, range_text: str) -> None:
        self.status_var.set(f"{label} 允许范围：{range_text}")

    @staticmethod
    def _read_numeric(variable: tk.DoubleVar) -> float:
        try:
            return float(variable.get())
        except (tk.TclError, TypeError, ValueError):
            return math.nan

    @staticmethod
    def _format_number(value: float) -> str:
        return f"{value:g}"

    @classmethod
    def _format_range(cls, lower: float, upper: float, unit: str = "") -> str:
        suffix = f" {unit}" if unit else ""
        return (
            f"{cls._format_number(lower)} .. "
            f"{cls._format_number(upper)}{suffix}"
        )

    @classmethod
    def _parameter_range_text(cls) -> str:
        return "\n".join(
            (
                "J4310 MIT：",
                f"  p_des：{cls._format_range(J4310_POSITION_MIN_DEG, J4310_POSITION_MAX_DEG, 'deg')}",
                f"  v_des：{cls._format_range(-J4310_VELOCITY_LIMIT, J4310_VELOCITY_LIMIT, 'rad/s')}",
                f"  Kp：{cls._format_range(0.0, J4310_KP_LIMIT)}",
                f"  Kd：{cls._format_range(0.0, J4310_KD_LIMIT)}",
                f"  tau：{cls._format_range(-J4310_TORQUE_LIMIT, J4310_TORQUE_LIMIT, 'Nm')}，且绝对值不能超过力矩限幅",
                f"  力矩限幅：{cls._format_range(0.1, J4310_TORQUE_LIMIT, 'Nm')}",
                "位置目标：",
                f"  M3508 独立输入：{cls._format_range(POSITION_MIN_DEG, POSITION_MAX_DEG, 'deg')}",
                f"  M3508 同步输入 / 滑块：{cls._format_range(M3508_SYNC_POSITION_MIN_DEG, M3508_SYNC_POSITION_MAX_DEG, 'deg')}",
                f"  M2006：{cls._format_range(M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG, 'deg')}",
                "PID 输入：",
                f"  Kp / Ki：{cls._format_range(PID_GAIN_MIN, PID_KP_KI_MAX)}",
                f"  Kd：{cls._format_range(PID_GAIN_MIN, PID_KD_MAX)}",
                f"  积分限幅：{cls._format_range(PID_GAIN_MIN, PID_INTEGRAL_LIMIT_MAX)}",
                f"  速度环输出限幅：{cls._format_range(PID_OUTPUT_LIMIT_MIN, PID_SPEED_OUTPUT_LIMIT_MAX)}",
                f"  位置环输出限幅：{cls._format_range(PID_OUTPUT_LIMIT_MIN, PID_POSITION_OUTPUT_LIMIT_MAX)}",
            )
        )

    def show_parameter_ranges(self) -> None:
        messagebox.showinfo(
            "控制参数范围",
            self._parameter_range_text(),
            parent=self.root,
        )
    def _add_action_buttons(self, parent: ttk.Frame, page: str) -> None:
        send_button = ttk.Button(
            parent, text="发送目标", command=lambda: self.send_page_now(page)
        )
        send_button.pack(side="left", padx=(0, 8))
        stop_button = ttk.Button(
            parent, text="停止发送", command=lambda: self.stop_page(page)
        )
        stop_button.pack(side="left", padx=(0, 8))
        self.page_action_buttons[page] = (send_button, stop_button)

    def _add_page_buttons(self, parent: ttk.Frame, page: str, target: str, target_name: str) -> None:
        self._add_action_buttons(parent, page)
        save_button = ttk.Button(
            parent,
            text="保存参数",
            command=lambda: self.save_motor_parameters(target),
        )
        save_button.pack(side="left", padx=(0, 8))
        self.page_save_buttons[page] = save_button

    def _select_page(self, page: str, log_change: bool = True) -> None:
        if page not in self.page_frames:
            return
        self.active_page = page
        for name, frame in self.page_frames.items():
            frame.pack_forget()
            if name == page:
                frame.pack(fill="both", expand=True)
        for name, tab in self.page_tabs.items():
            selected = name == page
            tab.configure(
                bg="#a9ddea" if selected else "#d6eff9",
                fg="#124b5e" if selected else "#315465",
            )
        if log_change:
            self.log(f"切换页面: {self.active_page}")

    def refresh_ports(self) -> None:
        ports = SerialTransport.available_ports()
        values = [f"{device} | {description}" for device, description in ports]
        self._port_lookup = {item: device for item, (device, _desc) in zip(values, ports)}
        self.port_combo["values"] = values
        if values and self.port_var.get() not in values:
            self.port_var.set(values[0])
        if not values:
            self.port_var.set("未发现串口")

    def toggle_connection(self) -> None:
        if self.connection_requested:
            return
        if serial is None:
            self.status_var.set("缺少 pyserial")
            self.log("未安装 pyserial，请先安装 requirements.txt")
            return
        selected = self.port_var.get()
        port = getattr(self, "_port_lookup", {}).get(selected)
        if port is None:
            self.status_var.set("请选择串口")
            return
        baudrate = int(self.baud_var.get())
        reused_cdc = self.transport.matches(port, baudrate)
        if not reused_cdc:
            try:
                self.transport.open(port, baudrate)
            except Exception as exc:
                self.status_var.set("连接失败")
                self.log(str(exc))
                return
        self.connection_requested = True
        self.transport.discard_input()
        self._begin_handshake(time.monotonic())
        self.connect_button.configure(text="握手中...", state="disabled")
        self.disconnect_button.configure(state="normal")
        self.status_var.set("握手中...")
        action = "复用已打开的无线 DAP 串口" if reused_cdc else "串口已打开"
        self.log(f"{action}，等待单片机握手: {port} @ {baudrate} 8N1")

    def disconnect(self) -> None:
        self._disable_all_outputs()
        was_requested = self.connection_requested
        self._reset_connection_state()
        if was_requested:
            self.log("控制链路已断开；无线 DAP 串口保持打开，便于主控重新上电后恢复")

    def _begin_handshake(self, now: float, *, reset_feedback: bool = True) -> None:
        self.parser = FrameParser()
        self.handshaken = False
        if reset_feedback:
            self._reset_feedback_display("等待握手后反馈")
        self.handshake_started_at = now
        self.last_handshake = 0.0
        self.handshake_sequence = None
        self.handshake_attempts = 0
        self.handshake_timeout_reported = False
        self.last_heartbeat = 0.0
        self.last_board_response = 0.0
        self._update_j4310_auto_return_button()

    def _reset_feedback_display(self, status: str) -> None:
        self.j4310_output_position_var.set("当前输出轴角度: -- deg")
        self.j4310_feedback_status = status
        self.j4310_auto_return_status_received = False
        self._refresh_j4310_status()
        names = {
            (1, 2, 1): "M3508 #1",
            (1, 2, 2): "M3508 #2",
            (2, 3, 1): "闸门 M2006",
            (2, 3, 2): "夹爪 M2006",
        }
        for key, variable in self.dji_diagnostic_vars.items():
            variable.set(format_dji_feedback(names[key], "--", status))

    def _reset_connection_state(self) -> None:
        self._cancel_remote_arm_delays()
        self._active_arm_mask = 0
        self.remote_action_states.clear()
        self.remote_pd9_zero_pending = False
        for button in getattr(self, "remote_action_buttons", {}).values():
            button.configure(text="执行")
        self.connection_requested = False
        self.handshaken = False
        self.handshake_sequence = None
        self.handshake_attempts = 0
        self.handshake_timeout_reported = False
        self.handshake_started_at = 0.0
        self.last_handshake = 0.0
        self.last_board_response = 0.0
        self.parser = FrameParser()
        self._reset_feedback_display("未连接")
        self.status_var.set("未连接")
        self.connect_button.configure(text="连接", state="normal")
        self.disconnect_button.configure(state="disabled")
        self.board_state_var.set("板端状态: --")
        self.remote_var.set("远程链路: --")
        self.j4310_auto_return_pending = None
        self._update_j4310_auto_return_button()

    def _send_handshake(self) -> None:
        if not self.transport.connected:
            return
        if self.handshake_sequence is None:
            # Use the same non-zero sequence as the byte stream independently
            # verified with VOFA. Subsequent retries must keep this sequence.
            self.handshake_sequence = HANDSHAKE_SEQUENCE
        sequence = self.handshake_sequence
        frame = build_handshake_frame(sequence)
        if not self.transport.write(frame):
            return
        self.handshake_attempts += 1
        self.tx_count += 1
        self.tx_var.set(f"TX {self.tx_count}")
        self.last_handshake = time.monotonic()
        if self.handshake_attempts == 1:
            self.log(
                f"握手请求已写入串口: seq=0x{sequence:04X}, "
                f"{len(frame)} B, {frame.hex(' ').upper()}"
            )

    def _next_sequence(self) -> int:
        value = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFF
        return value

    def _send(self, msg_type: int, payload: bytes = b"") -> bool:
        if not self.transport.connected or not self.handshaken:
            return False
        if self.transport.write(
            encode_frame(msg_type, self._next_sequence(), payload)
        ):
            self.tx_count += 1
            self.tx_var.set(f"TX {self.tx_count}")
            return True
        return False

    def _send_estop_frame(self) -> None:
        self._send(MSG_ESTOP, b"\x01")

    def _on_space_estop(self, _event: tk.Event) -> str:
        if not self._space_estop_pressed:
            self._space_estop_pressed = True
            self.estop()
        return "break"

    def _on_space_estop_released(self, _event: tk.Event) -> str:
        self._space_estop_pressed = False
        return "break"

    def estop(self) -> None:
        self._cancel_remote_arm_delays()
        self.remote_pd9_zero_pending = False
        self._disable_all_outputs()
        self._send_estop_frame()
        self.status_var.set("已发送急停")
        self.log("急停：全部电机已停止发送；重新发送目标即可恢复控制")

    def _disable_all_outputs(self) -> None:
        self._drag_target = None
        self._active_arm_mask = 0
        for variable in (
            self.arm_enable,
            self.m3508_enable,
            self.conveyor_enable,
            self.gripper_enable,
        ):
            variable.set(False)

    def _values(self) -> tuple[float, ...]:
        return (
            self._read_numeric(self.j_position),
            self._read_numeric(self.j_velocity),
            self._read_numeric(self.j_kp),
            self._read_numeric(self.j_kd),
            self._read_numeric(self.m3508_position_1),
            self._read_numeric(self.m3508_position_2),
            self._read_numeric(self.conveyor_position),
            self._read_numeric(self.gripper_position),
        )

    def _mask_for_page(self, page: str) -> int:
        if page == "机械臂":
            mask = ENABLE_J4310_ONLY if self.arm_enable.get() else 0
            if self.m3508_enable.get():
                mask |= ENABLE_M3508_ONLY
            return mask
        if page == "闸门":
            return ENABLE_CONVEYOR if self.conveyor_enable.get() else 0
        if page == "夹爪":
            return ENABLE_GRIPPER if self.gripper_enable.get() else 0
        return 0

    def _update_j4310_position_range(self) -> None:
        lower = J4310_POSITION_MIN_DEG
        upper = J4310_POSITION_MAX_DEG
        value = float(self.j_position.get())
        self.j_position.set(max(lower, min(upper, value)))
        for control in self.j_position_controls:
            control.configure(from_=lower, to=upper)

    def permanently_calibrate_j4310(self) -> None:
        if not self.transport.connected:
            self.status_var.set("未连接，永久标定未发送")
            self.log("J4310 永久标定未发送：串口未连接")
            return
        if not self.handshaken:
            self.status_var.set("尚未握手，永久标定未发送")
            self.log("J4310 永久标定未发送：尚未与单片机握手")
            return
        payload = build_motor_action_payload(
            MOTOR_ACTION_J4310_SAVE_ZERO,
            1,
            0x06,
        )
        self._send(MSG_MOTOR_ACTION, payload)
        self._active_arm_mask = 0
        self.arm_enable.set(False)
        self.m3508_enable.set(False)
        self.status_var.set("J4310 永久标定请求已发送")
        self.log("J4310 永久标定请求已发送，等待板端回执")

    def _update_j4310_auto_return_button(self) -> None:
        pending = self.j4310_auto_return_pending
        if pending is not None:
            text = "重启归零：设置中"
            style = (
                "AutoReturnOn.TButton" if pending
                else "AutoReturnOff.TButton"
            )
        elif not self.j4310_auto_return_status_received:
            text = "重启归零：状态未知"
            style = "AutoReturnOff.TButton"
        elif not self.j4310_auto_return_available:
            text = "重启归零：不可用"
            style = "AutoReturnOff.TButton"
        else:
            text = (
                f"重启归零："
                f"{'开启' if self.j4310_auto_return_enabled else '关闭'}"
            )
            style = (
                "AutoReturnOn.TButton"
                if self.j4310_auto_return_enabled
                else "AutoReturnOff.TButton"
            )
        interactive = (
            self.handshaken
            and pending is None
            and self.j4310_auto_return_status_received
            and self.j4310_auto_return_available
        )
        self.j4310_auto_return_button.configure(
            text=text,
            style=style,
            state="normal" if interactive else "disabled",
        )

    def toggle_j4310_auto_return(self) -> None:
        if not self.transport.connected or not self.handshaken:
            self.status_var.set("尚未连接，重启归零设置未发送")
            return
        if not self.j4310_auto_return_status_received:
            self.status_var.set("板端未上报重启归零状态，请烧录新固件")
            self.log("J4310 重启归零设置未发送：板端状态协议不支持")
            return
        if not self.j4310_auto_return_available:
            self.status_var.set("J4310 重启归零不可用：板端不支持")
            self.log("J4310 重启归零设置未发送：板端功能不可用")
            return
        requested = not self.j4310_auto_return_enabled
        payload = build_motor_action_payload(
            MOTOR_ACTION_J4310_AUTO_RETURN,
            1,
            0x06,
            1 if requested else 0,
        )
        if not self._send(MSG_MOTOR_ACTION, payload):
            self.status_var.set("J4310 重启归零设置发送失败")
            return
        self.j4310_auto_return_pending = requested
        self._update_j4310_auto_return_button()
        self.status_var.set("J4310 重启归零设置已发送")
        self.log(
            f"J4310 重启归零请求已发送："
            f"{'开启' if requested else '关闭'}，等待板端回执"
        )

    def _apply_j4310_auto_return_state(self, status: object) -> None:
        if not isinstance(status, dict):
            self.j4310_auto_return_status_received = False
            self.j4310_auto_return_available = False
            self.j4310_auto_return_active = False
            self.j4310_auto_return_stage = 0
            self._refresh_j4310_status()
            self._update_j4310_auto_return_button()
            return
        self.j4310_auto_return_status_received = True
        self.j4310_auto_return_available = bool(status["available"])
        self.j4310_auto_return_active = bool(status["active"])
        self.j4310_auto_return_stage = int(status["stage"])
        if self.j4310_auto_return_pending is None:
            self.j4310_auto_return_enabled = bool(status["enabled"])
        self._refresh_j4310_status()
        self._update_j4310_auto_return_button()

    def _pid_values(self, page: str | None = None) -> tuple[float, ...]:
        m2006_key = {
            "闸门": "conveyor",
            "夹爪": "gripper",
        }.get(page, "conveyor")
        groups = (
            self.m3508_speed_pid_vars,
            self.m3508_position_pid_vars,
            self.m2006_speed_pid_vars_by_motor[m2006_key],
            self.m2006_position_pid_vars_by_motor[m2006_key],
        )
        return tuple(
            self._read_numeric(group[name])
            for group in groups
            for name in ("kp", "ki", "kd", "integral_limit", "output_limit")
        )

    @staticmethod
    def _parameter_error(
        label: str,
        value: float,
        lower: float,
        upper: float,
        unit: str = "",
    ) -> str | None:
        range_text = UpperConsole._format_range(lower, upper, unit)
        if not math.isfinite(value):
            return f"{label}：不是有效数字；允许范围 {range_text}"
        if value < lower or value > upper:
            suffix = f" {unit}" if unit else ""
            return (
                f"{label}：当前 {UpperConsole._format_number(value)}{suffix}；"
                f"允许范围 {range_text}"
            )
        return None

    def _pid_parameter_errors(
        self,
        pid_values: tuple[float, ...],
        group_offset: int,
        group_name: str,
    ) -> list[str]:
        errors: list[str] = []
        fields = (
            ("Kp", PID_GAIN_MIN, PID_KP_KI_MAX),
            ("Ki", PID_GAIN_MIN, PID_KP_KI_MAX),
            ("Kd", PID_GAIN_MIN, PID_KD_MAX),
            ("积分限幅", PID_GAIN_MIN, PID_INTEGRAL_LIMIT_MAX),
            ("输出限幅", PID_OUTPUT_LIMIT_MIN, PID_SPEED_OUTPUT_LIMIT_MAX),
            ("Kp", PID_GAIN_MIN, PID_KP_KI_MAX),
            ("Ki", PID_GAIN_MIN, PID_KP_KI_MAX),
            ("Kd", PID_GAIN_MIN, PID_KD_MAX),
            ("积分限幅", PID_GAIN_MIN, PID_INTEGRAL_LIMIT_MAX),
            ("输出限幅", PID_OUTPUT_LIMIT_MIN, PID_POSITION_OUTPUT_LIMIT_MAX),
        )
        for index, (label, lower, upper) in enumerate(fields):
            loop = "速度环" if index < 5 else "位置环"
            error = self._parameter_error(
                f"{group_name} {loop} {label}",
                pid_values[group_offset + index],
                lower,
                upper,
            )
            if error is not None:
                errors.append(error)
        return errors

    def _send_parameter_errors(
        self,
        page: str,
        values: tuple[float, ...],
        enabled: bool,
        frame_options: dict[str, object],
    ) -> list[str]:
        if not enabled:
            return []

        enable_mask = int(
            frame_options.get("enable_mask", self._mask_for_page(page))
        )
        pid_values = tuple(
            frame_options.get("pid_values", self._pid_values(page))
        )
        errors: list[str] = []

        if enable_mask & ENABLE_J4310_ONLY:
            j_tau = float(
                frame_options.get("j_tau", self._read_numeric(self.j_tau))
            )
            torque_limit = float(
                frame_options.get(
                    "j_torque_limit",
                    self._read_numeric(self.j_torque_limit),
                )
            )
            j4310_specs = (
                ("J4310 p_des", values[0], J4310_POSITION_MIN_DEG,
                 J4310_POSITION_MAX_DEG, "deg"),
                ("J4310 v_des", values[1], -J4310_VELOCITY_LIMIT,
                 J4310_VELOCITY_LIMIT, "rad/s"),
                ("J4310 Kp", values[2], 0.0, J4310_KP_LIMIT, ""),
                ("J4310 Kd", values[3], 0.0, J4310_KD_LIMIT, ""),
                ("J4310 tau", j_tau, -J4310_TORQUE_LIMIT,
                 J4310_TORQUE_LIMIT, "Nm"),
                ("J4310 力矩限幅", torque_limit, 0.1,
                 J4310_TORQUE_LIMIT, "Nm"),
            )
            for spec in j4310_specs:
                error = self._parameter_error(*spec)
                if error is not None:
                    errors.append(error)
            if (
                math.isfinite(j_tau)
                and math.isfinite(torque_limit)
                and torque_limit > 0.0
                and abs(j_tau) > torque_limit
            ):
                errors.append(
                    f"J4310 tau：绝对值 {self._format_number(abs(j_tau))} Nm "
                    f"超过当前力矩限幅 {self._format_number(torque_limit)} Nm"
                )

        if enable_mask & ENABLE_M3508_ONLY:
            for index, (value, target_range) in enumerate(
                zip(
                    values[4:6],
                    (
                        (M3508_SYNC_POSITION_MIN_DEG,
                         M3508_SYNC_POSITION_MAX_DEG),
                        (M3508_SYNC_POSITION_MIN_DEG,
                         M3508_SYNC_POSITION_MAX_DEG),
                    ),
                    strict=True,
                ),
                start=1,
            ):
                error = self._parameter_error(
                    f"M3508 #{index} 目标位置",
                    value,
                    *target_range,
                    "deg",
                )
                if error is not None:
                    errors.append(error)
            errors.extend(
                self._pid_parameter_errors(pid_values, 0, "M3508 / C620")
            )

        if page in ("闸门", "夹爪"):
            value_index = 6 if page == "闸门" else 7
            error = self._parameter_error(
                f"{page} M2006 目标位置",
                values[value_index],
                M2006_POSITION_MIN_DEG,
                M2006_POSITION_MAX_DEG,
                "deg",
            )
            if error is not None:
                errors.append(error)
            errors.extend(
                self._pid_parameter_errors(pid_values, 10, "M2006 / C610")
            )
        return errors

    def _validate_send_parameters(
        self,
        page: str,
        values: tuple[float, ...],
        enabled: bool,
        frame_options: dict[str, object],
    ) -> bool:
        errors = self._send_parameter_errors(
            page, values, enabled, frame_options
        )
        if not errors:
            return True
        details = "\n".join(f"- {error}" for error in errors)
        self.status_var.set("参数超出范围，目标未发送")
        self.log(f"目标未发送：参数校验失败：{'；'.join(errors)}")
        messagebox.showerror(
            "参数超出范围",
            f"以下参数无效，已阻止发送：\n{details}\n\n"
            f"全部允许范围：\n{self._parameter_range_text()}",
            parent=self.root,
        )
        return False

    def _build_position_frame(
        self,
        page: str,
        values: tuple[float, ...],
        enabled: bool = True,
        *,
        enable_mask: int | None = None,
        j_tau: float | None = None,
        j_torque_limit: float | None = None,
        pid_values: tuple[float, ...] | None = None,
        j4310_stop: bool = False,
        include_pid: bool = True,
    ) -> bytes:
        mask = (
            self._mask_for_page(page) if enabled else 0
        ) if enable_mask is None else enable_mask
        if j4310_stop:
            mask |= COMMAND_J4310_STOP
        lower = J4310_RAW_MIN_DEG
        upper = J4310_RAW_MAX_DEG
        j_position_deg = max(lower, min(upper, values[0]))
        common_values = (
            mask,
            math.radians(j_position_deg),
            values[1],
            values[2],
            values[3],
            self._read_numeric(self.j_tau) if j_tau is None else j_tau,
            self._read_numeric(self.j_torque_limit) if j_torque_limit is None else j_torque_limit,
            math.radians(values[4]),
            math.radians(values[5]),
            math.radians(values[6]),
            math.radians(values[7] * GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG),
        )
        if include_pid:
            payload = build_extended_position_payload(
                *common_values,
                *(self._pid_values(page) if pid_values is None else pid_values),
            )
        else:
            payload = build_position_torque_payload(*common_values)
        return encode_frame(MSG_UPPER_POSITION_CMD, self._next_sequence(), payload)

    def send_page_now(self, page: str) -> bool:
        if page == "闸门":
            self.conveyor_enable.set(True)
        elif page == "夹爪":
            self.gripper_enable.set(True)
        values = self._values()
        if not self._send_position_values(page, values, True):
            return False
        if page == "机械臂":
            self._active_arm_mask = self._mask_for_page(page)
            self._sent_arm_values = values
            self._sent_j_tau = self._read_numeric(self.j_tau)
            self._sent_j_torque_limit = self._read_numeric(
                self.j_torque_limit
            )
            self._sent_pid_values = self._pid_values()
        self.status_var.set(f"{page} 目标已发送")
        self.log(f"{page} 目标已发送")
        return True

    def _start_slider_drag(self, target: str) -> None:
        if self._drag_target is not None and self._drag_target != target:
            self._stop_slider_target(self._drag_target)
        enable_var = self._slider_target_enable_var(target)
        enable_var.set(True)
        if self._send_slider_target(target):
            self._drag_target = target
            self.status_var.set(f"{self._slider_target_name(target)} 滑块控制中")

    def _finish_slider_drag(self, target: str) -> None:
        if self._drag_target != target:
            return
        self._send_slider_target(target)
        self._drag_target = None
        self.status_var.set(f"{self._slider_target_name(target)} 保持当前目标")

    @staticmethod
    def _slider_target_name(target: str) -> str:
        return {
            "j4310": "J4310",
            "m3508": "M3508",
            "conveyor": "闸门 M2006",
            "gripper": "夹爪 M2006",
        }[target]

    def _slider_target_enabled(self, target: str) -> bool:
        return self._slider_target_enable_var(target).get()

    def _slider_target_enable_var(self, target: str) -> tk.BooleanVar:
        return {
            "j4310": self.arm_enable,
            "m3508": self.m3508_enable,
            "conveyor": self.conveyor_enable,
            "gripper": self.gripper_enable,
        }[target]

    def _send_slider_target(self, target: str) -> bool:
        if not self._slider_target_enabled(target):
            return False
        if target in ("j4310", "m3508"):
            bit, _enable_var, value_indices, _display_name = (
                self._arm_group_controls(target)
            )
            current_values = self._values()
            sent_values = list(self._sent_arm_values)
            for index in value_indices:
                sent_values[index] = current_values[index]

            j_tau = self._sent_j_tau
            j_torque_limit = self._sent_j_torque_limit
            pid_values = list(self._sent_pid_values)
            if target == "j4310":
                j_tau = self._read_numeric(self.j_tau)
                j_torque_limit = self._read_numeric(self.j_torque_limit)
            else:
                pid_values[:10] = self._pid_values()[:10]

            enable_mask = (
                self._active_arm_mask & self._mask_for_page("机械臂")
            ) | bit
            sent = self._send_position_values(
                "机械臂",
                tuple(sent_values),
                True,
                enable_mask=enable_mask,
                j_tau=j_tau,
                j_torque_limit=j_torque_limit,
                pid_values=tuple(pid_values),
            )
            if sent:
                self._active_arm_mask = enable_mask
                self._sent_arm_values = tuple(sent_values)
                self._sent_j_tau = j_tau
                self._sent_j_torque_limit = j_torque_limit
                self._sent_pid_values = tuple(pid_values)
            return sent

        page = "闸门" if target == "conveyor" else "夹爪"
        return self._send_position_values(page, self._values(), True)

    def _stop_slider_target(self, target: str) -> None:
        if target in ("j4310", "m3508"):
            bit, enable_var, _value_indices, _display_name = (
                self._arm_group_controls(target)
            )
            enable_var.set(False)
            enable_mask = self._active_arm_mask & ~bit
            self._send_position_values(
                "机械臂",
                self._sent_arm_values,
                True,
                enable_mask=enable_mask,
                j_tau=self._sent_j_tau,
                j_torque_limit=self._sent_j_torque_limit,
                pid_values=self._sent_pid_values,
                j4310_stop=target == "j4310",
            )
            self._active_arm_mask = enable_mask
            return

        page, enable_var = {
            "conveyor": ("闸门", self.conveyor_enable),
            "gripper": ("夹爪", self.gripper_enable),
        }[target]
        enable_var.set(False)
        self._send_position_values(page, self._values(), False)

    def _update_j4310_diagnostic(
        self, diagnostic: object, tx_diagnostic: object = None
    ) -> None:
        state_text = "状态未知"
        if isinstance(tx_diagnostic, dict):
            feedback_state = int(tx_diagnostic["feedback_state"])
            if feedback_state == 0xFF:
                state_text = "未收到反馈"
            elif feedback_state == 0:
                state_text = "电机未使能"
            elif feedback_state == 1:
                state_text = "电机已使能"
            else:
                state_text = J4310_FAULT_NAMES.get(
                    feedback_state, f"状态 0x{feedback_state:X}"
                )
        elif (
            isinstance(diagnostic, dict)
            and int(diagnostic["accepted_frames"]) > 0
            and int(diagnostic["last_result"]) == 1
        ):
            feedback_state = (int(diagnostic["last_data0"]) >> 4) & 0x0F
            if feedback_state == 0:
                state_text = "电机未使能"
            elif feedback_state == 1:
                state_text = "电机已使能"
            else:
                state_text = J4310_FAULT_NAMES.get(
                    feedback_state, f"状态 0x{feedback_state:X}"
                )
        self.j4310_feedback_status = state_text
        self._refresh_j4310_status()

    def _apply_j4310_position_feedback(
        self, valid: bool, position_rad: float
    ) -> None:
        if valid:
            self.j4310_output_position_var.set(
                f"当前输出轴角度: {math.degrees(position_rad):.2f} deg"
            )

    def _refresh_j4310_status(self) -> None:
        self.j4310_bus_label.configure(
            text=f"反馈状态: {self.j4310_feedback_status}"
        )

    def _arm_group_controls(
        self, group: str
    ) -> tuple[int, tk.BooleanVar, tuple[int, ...], str]:
        if group == "j4310":
            return ENABLE_J4310_ONLY, self.arm_enable, (0, 1, 2, 3), "J4310"
        if group == "m3508":
            return ENABLE_M3508_ONLY, self.m3508_enable, (4, 5), "M3508"
        raise ValueError(f"unknown arm group: {group}")

    def send_arm_group_now(self, group: str) -> bool:
        bit, enable_var, value_indices, display_name = self._arm_group_controls(group)
        enable_var.set(True)

        current_values = self._values()
        sent_values = list(self._sent_arm_values)
        for index in value_indices:
            sent_values[index] = current_values[index]
        j_tau = self._sent_j_tau
        j_torque_limit = self._sent_j_torque_limit
        pid_values = list(self._sent_pid_values)
        if group == "j4310":
            j_tau = self._read_numeric(self.j_tau)
            j_torque_limit = self._read_numeric(self.j_torque_limit)
        else:
            pid_values[:10] = self._pid_values()[:10]

        enable_mask = (
            self._active_arm_mask & self._mask_for_page("机械臂")
        ) | bit
        if self._send_position_values(
            "机械臂",
            tuple(sent_values),
            True,
            enable_mask=enable_mask,
            j_tau=j_tau,
            j_torque_limit=j_torque_limit,
            pid_values=tuple(pid_values),
        ):
            self._active_arm_mask = enable_mask
            self._sent_arm_values = tuple(sent_values)
            self._sent_j_tau = j_tau
            self._sent_j_torque_limit = j_torque_limit
            self._sent_pid_values = tuple(pid_values)
            self.status_var.set(f"{display_name} 目标已发送")
            self.log(f"{display_name} 目标已发送")
            return True
        return False

    def stop_arm_group(self, group: str) -> None:
        bit, enable_var, _value_indices, display_name = self._arm_group_controls(group)
        enable_var.set(False)
        enable_mask = self._active_arm_mask & ~bit
        if not self.transport.connected:
            self.status_var.set(f"{display_name} 已取消输出，串口未连接")
            self.log(f"{display_name} 已取消输出；串口未连接，未下发停止帧")
            return
        if not self.handshaken:
            self.status_var.set(f"{display_name} 已取消输出，尚未握手")
            self.log(f"{display_name} 已取消输出；尚未握手，未下发停止帧")
            return
        if self._send_position_values(
            "机械臂",
            self._sent_arm_values,
            True,
            enable_mask=enable_mask,
            j_tau=self._sent_j_tau,
            j_torque_limit=self._sent_j_torque_limit,
            pid_values=self._sent_pid_values,
            j4310_stop=group == "j4310",
        ):
            self._active_arm_mask = enable_mask
            self.status_var.set(f"{display_name} 已停止发送")
            self.log(f"{display_name} 已下发停止帧")

    def stop_page(self, page: str) -> None:
        if page == "机械臂":
            self.arm_enable.set(False)
            self.m3508_enable.set(False)
        elif page == "闸门":
            self.conveyor_enable.set(False)
        elif page == "夹爪":
            self.gripper_enable.set(False)
        else:
            return
        if not self.transport.connected:
            self.status_var.set(f"{page} 已取消输出，串口未连接")
            self.log(f"{page} 已取消输出；串口未连接，未下发停止帧")
            return
        if not self.handshaken:
            self.status_var.set(f"{page} 已取消输出，尚未握手")
            self.log(f"{page} 已取消输出；尚未握手，未下发停止帧")
            return
        if self._send_position_values(
            page,
            self._values(),
            False,
            j4310_stop=page == "机械臂",
        ):
            if page == "机械臂":
                self._active_arm_mask = 0
        self.status_var.set(f"{page} 已停止发送")
        self.log(f"{page} 已下发停止帧")

    def _send_position_values(
        self,
        page: str,
        values: tuple[float, ...],
        enabled: bool,
        **frame_options,
    ) -> bool:
        if frame_options.get("j4310_stop"):
            frame_options["j_tau"] = 0.0
            frame_options["j_torque_limit"] = J4310_TORQUE_LIMIT
        if not self._validate_send_parameters(
            page, values, enabled, frame_options
        ):
            return False
        if not self.transport.connected:
            self.status_var.set("未连接，目标未发送")
            return False
        if not self.handshaken:
            self.status_var.set("尚未握手，目标未发送")
            return False
        frame = self._build_position_frame(
            page, values, enabled, **frame_options
        )
        if self.transport.write(frame):
            self.tx_count += 1
            self.tx_var.set(f"TX {self.tx_count}")
            return True
        return False

    def _accept_handshake_ack(self, frame: Frame) -> None:
        if self.handshaken:
            return
        if self.handshake_sequence is None:
            self.log(
                f"收到 ACK seq=0x{frame.sequence:04X}，但当前没有握手请求"
            )
            return
        if frame.sequence != self.handshake_sequence:
            self.log(
                f"收到 ACK 但序号不匹配: 收到 0x{frame.sequence:04X}，"
                f"等待 0x{self.handshake_sequence:04X}"
            )
            return
        if frame.payload != HANDSHAKE_MAGIC:
            self.log(
                f"收到 ACK seq=0x{frame.sequence:04X}，但 payload 为 "
                f"{frame.payload.hex(' ').upper()}，期望 48 37 32 33"
            )
            return
        self.handshaken = True
        self.last_heartbeat = 0.0
        self.last_board_response = time.monotonic()
        self.status_var.set("已握手")
        self.connect_button.configure(text="已握手", state="disabled")
        self.disconnect_button.configure(state="normal")
        self._update_j4310_auto_return_button()
        self.log("已收到单片机握手确认，控制链路就绪")

    def _update_dji_diagnostics(
        self,
        diagnostics: list[dict],
        _fdcan_rx_counts: tuple[int, int, int] | None = None,
        *,
        clear_missing: bool = True,
    ) -> None:
        names = {
            (1, 2, 1): "M3508 #1",
            (1, 2, 2): "M3508 #2",
            (2, 3, 1): "闸门 M2006",
            (2, 3, 2): "夹爪 M2006",
        }
        received_keys = set()
        for diagnostic in diagnostics:
            key = (
                int(diagnostic["model"]),
                int(diagnostic["can_bus"]),
                int(diagnostic["node_id"]),
            )
            variable = self.dji_diagnostic_vars.get(key)
            if variable is None:
                continue
            received_keys.add(key)
            name = names[key]
            if not diagnostic["feedback_received"]:
                variable.set(format_dji_feedback(name, "否", "未收到反馈"))
                continue
            reduction_ratio = DJI_REDUCTION_RATIOS[key[0]]
            current_output_rad = (
                float(diagnostic["rotor_position_rad"]) / reduction_ratio
            )
            freshness = "正常" if diagnostic["feedback_fresh"] else "反馈超时"
            if not diagnostic["zero_valid"]:
                variable.set(
                    format_dji_feedback(
                        name,
                        "否",
                        f"{freshness}，等待稳定标零",
                        math.degrees(current_output_rad),
                    )
                )
                continue
            relative_output_rad = float(
                diagnostic["relative_output_position_rad"]
            )
            current_output_rad = (
                float(diagnostic["zero_rotor_position_rad"]) /
                reduction_ratio + relative_output_rad
            )
            variable.set(
                format_dji_feedback(
                    name,
                    "是",
                    freshness,
                    math.degrees(current_output_rad),
                    math.degrees(relative_output_rad),
                )
            )
        for key, variable in self.dji_diagnostic_vars.items():
            if clear_missing and key not in received_keys:
                variable.set(
                    format_dji_feedback(
                        names[key], "--", "未收到诊断，请烧录新固件"
                    )
                )

    def _periodic_send(self) -> None:
        if self.connection_requested and self.transport.connected:
            now = time.monotonic()
            if not self.handshaken:
                if (not self.handshake_timeout_reported and
                        now - self.handshake_started_at >=
                        HANDSHAKE_TIMEOUT_MS / 1000.0):
                    self.handshake_timeout_reported = True
                    self.status_var.set("等待无线串口恢复")
                    self.log(
                        f"握手仍未响应: 已成功写入 {self.handshake_attempts} 次，"
                        f"等待 ACK seq=0x{self.handshake_sequence:04X}；"
                        f"串口收到 {self.rx_count} B，解析有效帧 "
                        f"{self.parser.valid_count}，CRC 错误 "
                        f"{self.parser.crc_error_count}，长度错误 "
                        f"{self.parser.length_error_count}。保持无线 DAP 串口打开继续等待"
                    )
                if (
                    now - self.handshake_started_at >=
                    HANDSHAKE_OPEN_DELAY_MS / 1000.0
                    and now - self.last_handshake >=
                    HANDSHAKE_PERIOD_MS / 1000.0
                ):
                    self._send_handshake()
            else:
                if now - self.last_heartbeat >= HEARTBEAT_PERIOD_MS / 1000.0:
                    self._send(MSG_HEARTBEAT)
                    self.last_heartbeat = now
                if self._drag_target is not None:
                    self._send_slider_target(self._drag_target)
        self._send_after_id = self.root.after(SEND_PERIOD_MS, self._periodic_send)

    def _poll_io(self) -> None:
        while True:
            try:
                data = self.transport.rx_queue.get_nowait()
            except queue.Empty:
                break
            self.rx_count += len(data)
            self.rx_var.set(f"RX {self.rx_count} B")
            for frame in self.parser.feed(data):
                if not self.connection_requested:
                    continue
                if frame.msg_type == MSG_ACK:
                    self._accept_handshake_ack(frame)
                elif frame.msg_type == MSG_ROBOT_STATE:
                    try:
                        state = decode_robot_state(frame.payload)
                    except ValueError:
                        continue
                    self.last_board_response = time.monotonic()
                    self.board_state_var.set(f"板端状态: {STATE_NAMES.get(state['state'], str(state['state']))}")
                    fdcan_counts = state["fdcan_rx_counts"]
                    fdcan_text = (
                        f" · FDCAN2 RX {fdcan_counts[1]}"
                        if fdcan_counts is not None
                        else ""
                    )
                    self.remote_var.set(
                        f"远程链路: {'活动' if state['remote_active'] else '空闲'} · "
                        f"RX序号 {state['last_rx_sequence']}{fdcan_text}"
                    )
                    self._apply_j4310_position_feedback(
                        bool(state["j4310_position_valid"]),
                        float(state["j4310_position_rad"]),
                    )
                    self._update_j4310_diagnostic(
                        state["j4310_rx_diagnostic"],
                        state["j4310_tx_diagnostic"],
                    )
                    self._apply_j4310_auto_return_state(
                        state["j4310_auto_return"]
                    )
                    if state["dji_diagnostics"]:
                        self._update_dji_diagnostics(
                            state["dji_diagnostics"], state["fdcan_rx_counts"]
                        )
                elif frame.msg_type == MSG_DJI_TELEMETRY:
                    try:
                        telemetry = decode_dji_telemetry(frame.payload)
                    except ValueError:
                        continue
                    self.last_board_response = time.monotonic()
                    self._update_dji_diagnostics(
                        [telemetry["diagnostic"]],
                        telemetry["fdcan_rx_counts"],
                        clear_missing=False,
                    )
                elif frame.msg_type == MSG_FAULT:
                    self._handle_motor_fault(frame.payload)
                elif frame.msg_type == MSG_MOTOR_ACTION_RESULT:
                    self._handle_motor_action_result(frame.payload)
        while True:
            try:
                message = self.transport.error_queue.get_nowait()
            except queue.Empty:
                break
            if not self.transport.connected:
                self._disable_all_outputs()
                self._reset_connection_state()
                self.status_var.set("无线 DAP 已断开，请重新插入后连接")
                self.log(f"{message}；旧串口句柄已释放")
            else:
                self.status_var.set(message)
                self.log(message)
        self._poll_after_id = self.root.after(20, self._poll_io)

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{timestamp}  {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _record_motor_error(
        self,
        *,
        model: str,
        can_bus: int,
        node_id: int,
        error: str,
        detail: str,
        board_tick_ms: int,
    ) -> None:
        timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        line = (
            f"{timestamp} | {model} | FDCAN{can_bus} | "
            f"ID=0x{node_id:02X} | {error} ({detail}) | "
            f"board_tick_ms={board_tick_ms}"
        )
        try:
            with MOTOR_LOG_PATH.open("a", encoding="utf-8") as log_file:
                log_file.write(line + "\n")
        except OSError as exc:
            self.log(f"电机故障日志写入失败: {exc}")
            return
        self.log(f"电机报错已写入 {MOTOR_LOG_PATH}: {detail}")

    def _handle_motor_fault(self, payload: bytes) -> None:
        try:
            event = decode_motor_event(payload)
        except ValueError:
            return
        model = MOTOR_MODEL_NAMES.get(event["value"], f"型号 {event['value']}")
        if event["value"] == 0:
            detail = (
                "反馈超时/离线"
                if event["code"] == 0xF0
                else J4310_FAULT_NAMES.get(event["code"], "未知故障")
            )
        else:
            detail = (
                "反馈超时/离线"
                if event["code"] == 0xF0
                else "电机反馈异常"
            )
        self.status_var.set(f"{model} 报错: {detail}")
        self._record_motor_error(
            model=model,
            can_bus=event["can_bus"],
            node_id=event["node_id"],
            error=f"error=0x{event['code']:02X}",
            detail=detail,
            board_tick_ms=event["tick_ms"],
        )

    def _handle_motor_action_result(self, payload: bytes) -> None:
        try:
            result = decode_motor_action_result(payload)
        except ValueError:
            return
        if result["action"] == MOTOR_ACTION_J4310_AUTO_RETURN:
            requested = self.j4310_auto_return_pending
            self.j4310_auto_return_pending = None
            if result["status"] == 0 and requested is not None:
                self.j4310_auto_return_available = True
                self.j4310_auto_return_enabled = requested
                state_name = "开启" if requested else "关闭"
                self.status_var.set(f"J4310 重启归零已{state_name}")
                self.log(
                    f"J4310 重启归零已{state_name}；配置仅在本次 MCU 运行期间有效"
                )
            elif result["status"] != 0:
                self.status_var.set("J4310 重启归零设置失败")
                self.log("J4310 重启归零设置失败：板端拒绝了运行时配置")
            self._update_j4310_auto_return_button()
            return
        if result["action"] != MOTOR_ACTION_J4310_SAVE_ZERO:
            return
        if result["status"] == 0:
            self.status_var.set("J4310 永久标定成功")
            self.log("J4310 永久标定成功；当前位置已保存为电机永久零位")
        else:
            self.status_var.set("J4310 永久标定失败")
            detail = "永久标定失败：请确认反馈在线、无故障后重试"
            self._record_motor_error(
                model="J4310",
                can_bus=result["can_bus"],
                node_id=result["node_id"],
                error=(
                    f"action=0x{result['action']:02X}, "
                    f"status=0x{result['status']:02X}"
                ),
                detail=detail,
                board_tick_ms=result["tick_ms"],
            )

    def close(self) -> None:
        self.disconnect()
        self.transport.close()
        for after_id in (self._poll_after_id, self._send_after_id):
            try:
                self.root.after_cancel(after_id)
            except tk.TclError:
                pass
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    UpperConsole(root)
    root.mainloop()


if __name__ == "__main__":
    main()
