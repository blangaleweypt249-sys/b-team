# -*- coding: utf-8 -*-
"""H723VGT6 upper-control console.

This is intentionally a small desktop application: Tkinter is included with
Python on Windows and pyserial is the only external dependency. The position
test sends three +90/-90 degree cycles while refreshing the command faster
than the firmware's 200 ms remote timeout.
"""

from __future__ import annotations

import math
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk
from typing import Callable

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # The UI can still open and explain the missing package.
    serial = None
    list_ports = None

from protocol import (
    ENABLE_ARM,
    ENABLE_CONVEYOR,
    ENABLE_GRIPPER,
    HANDSHAKE_MAGIC,
    MSG_ACK,
    MSG_ESTOP,
    MSG_HEARTBEAT,
    MSG_ROBOT_STATE,
    MSG_UPPER_POSITION_CMD,
    Frame,
    FrameParser,
    build_handshake_frame,
    build_extended_position_payload,
    decode_robot_state,
    encode_frame,
)


BAUD_RATES = (115200, 230400, 460800, 921600)
SEND_PERIOD_MS = 50
HEARTBEAT_PERIOD_MS = 100
HANDSHAKE_PERIOD_MS = 150
HANDSHAKE_TIMEOUT_MS = 3000
TEST_HOLD_MS = 900
TEST_CYCLES = 3
TEST_ANGLE_RAD = math.pi / 2.0
J4310_POSITION_LIMIT = 12.5
J4310_VELOCITY_LIMIT = 30.0
J4310_KP_LIMIT = 500.0
J4310_KD_LIMIT = 5.0
J4310_TORQUE_LIMIT = 10.0

REFERENCE_PID_VALUES = {
    "M3508 / C620": (
        ("速度环", "80", "40", "0", "100", "16384"),
        ("位置环", "60", "0", "0", "0", "300 rpm"),
    ),
    "M2006 / C610": (
        ("速度环", "350", "250", "0", "0.5", "10000"),
        ("位置环", "220", "500", "5", "0.002", "50 rpm"),
    ),
}


STATE_NAMES = {
    0: "初始化",
    1: "就绪",
    2: "运行",
    3: "停止",
    4: "错误",
}


@dataclass
class TestPhase:
    page: str
    target: str
    target_name: str
    baseline: tuple[float, ...]
    direction: int
    phase_index: int
    started_at: float


class SerialTransport:
    """Non-blocking serial worker with queues owned by the Tk main thread."""

    def __init__(self) -> None:
        self.rx_queue: queue.Queue[bytes] = queue.Queue()
        self.error_queue: queue.Queue[str] = queue.Queue()
        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._serial is not None and bool(self._serial.is_open)

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
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.2,
        )
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self) -> None:
        while not self._stop_event.is_set():
            port = self._serial
            if port is None or not port.is_open:
                return
            try:
                data = port.read(port.in_waiting or 1)
                if data:
                    self.rx_queue.put(data)
            except Exception as exc:  # pragma: no cover - hardware-specific
                self.error_queue.put(f"串口接收失败: {exc}")
                return

    def write(self, data: bytes) -> None:
        if not data or not self.connected:
            return
        try:
            with self._write_lock:
                self._serial.write(data)
        except Exception as exc:  # pragma: no cover - hardware-specific
            self.error_queue.put(f"串口发送失败: {exc}")

    def close(self) -> None:
        self._stop_event.set()
        port = self._serial
        self._serial = None
        if port is not None:
            try:
                if port.is_open:
                    port.close()
            except Exception:
                pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.3)
        self._thread = None


class TestRunner:
    """Runs +90/-90 degree position phases for one page."""

    def __init__(self, app: "UpperConsole") -> None:
        self.app = app
        self.phase: TestPhase | None = None
        self._after_id: str | None = None

    @property
    def running(self) -> bool:
        return self.phase is not None

    def start(self, page: str, target: str, target_name: str, baseline: tuple[float, ...]) -> None:
        self.stop(log=False)
        self.phase = TestPhase(page, target, target_name, baseline, 1, 1, time.monotonic())
        self.app.set_test_button_state(True)
        self.app.log(f"开始 {target_name} 输出轴测试：正反 90°，共 {TEST_CYCLES} 次")
        self.app.set_test_target(self.phase)
        self._schedule()

    def _schedule(self) -> None:
        self._after_id = self.app.root.after(50, self._tick)

    def _tick(self) -> None:
        if self.phase is None:
            return
        self.app.set_test_target(self.phase)
        elapsed_ms = (time.monotonic() - self.phase.started_at) * 1000.0
        if elapsed_ms < TEST_HOLD_MS:
            self._schedule()
            return
        if self.phase.phase_index < TEST_CYCLES * 2:
            self.phase = TestPhase(
                self.phase.page,
                self.phase.target,
                self.phase.target_name,
                self.phase.baseline,
                -self.phase.direction,
                self.phase.phase_index + 1,
                time.monotonic(),
            )
            self.app.log(
                f"{self.phase.target_name}: 第 {(self.phase.phase_index + 1) // 2} 次"
                f" {'正转' if self.phase.direction > 0 else '反转'} 90°"
            )
            self._schedule()
            return
        self.app.set_test_target(
            TestPhase(
                self.phase.page,
                self.phase.target,
                self.phase.target_name,
                self.phase.baseline,
                0,
                self.phase.phase_index,
                time.monotonic(),
            )
        )
        self.app.log(f"{self.phase.target_name} 输出轴测试完成，回到基准位置")
        self.stop(log=False)

    def stop(self, log: bool = True) -> None:
        if self._after_id is not None:
            self.app.root.after_cancel(self._after_id)
            self._after_id = None
        if self.phase is not None and log:
            self.app.log("测试已停止")
        self.phase = None
        self.app.set_test_button_state(False)


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
        self.last_heartbeat = 0.0
        self.last_handshake = 0.0
        self.handshake_started_at = 0.0
        self.handshake_sequence: int | None = None
        self.handshaken = False
        self.test_runner = TestRunner(self)
        self.page_enable_vars: dict[str, tk.BooleanVar] = {}
        self.page_test_buttons: dict[str, ttk.Button] = {}
        self.page_frames: dict[str, ttk.Frame] = {}
        self.page_tabs: dict[str, tk.Button] = {}
        self._make_vars()
        self._make_styles()
        self._build_ui()
        self.refresh_ports()
        self.root.after(20, self._poll_io)
        self.root.after(SEND_PERIOD_MS, self._periodic_send)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _make_vars(self) -> None:
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(BAUD_RATES[0]))
        self.status_var = tk.StringVar(value="未连接")
        self.tx_var = tk.StringVar(value="TX 0")
        self.rx_var = tk.StringVar(value="RX 0")
        self.board_state_var = tk.StringVar(value="板端状态: --")
        self.remote_var = tk.StringVar(value="远程链路: --")
        self.j_position = tk.DoubleVar(value=0.0)
        self.j_velocity = tk.DoubleVar(value=2.0)
        self.j_kp = tk.DoubleVar(value=20.0)
        self.j_kd = tk.DoubleVar(value=0.5)
        self.j_tau = tk.DoubleVar(value=0.0)
        self.j_torque_limit = tk.DoubleVar(value=10.0)
        self.m3508_position_1 = tk.DoubleVar(value=0.0)
        self.m3508_position_2 = tk.DoubleVar(value=0.0)
        self.conveyor_position = tk.DoubleVar(value=0.0)
        self.gripper_position = tk.DoubleVar(value=0.0)
        self.m3508_speed_pid_vars = self._pid_vars(REFERENCE_PID_VALUES["M3508 / C620"][0])
        self.m3508_position_pid_vars = self._pid_vars(REFERENCE_PID_VALUES["M3508 / C620"][1])
        self.m2006_speed_pid_vars = self._pid_vars(REFERENCE_PID_VALUES["M2006 / C610"][0])
        self.m2006_position_pid_vars = self._pid_vars(REFERENCE_PID_VALUES["M2006 / C610"][1])
        self.conveyor_enable = tk.BooleanVar(value=False)
        self.gripper_enable = tk.BooleanVar(value=False)
        self.arm_enable = tk.BooleanVar(value=False)

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
        style.configure("TCheckbutton", background="#dff3fa", foreground="#1f3442", font=("Microsoft YaHei UI", 10))
        style.map("TCheckbutton", background=[("active", "#dff3fa")])
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
        ttk.Label(connection, text="串口", style="CardTitle.TLabel").pack(side="left", padx=(0, 8))
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, state="readonly", width=28)
        self.port_combo.pack(side="left", padx=(0, 8))
        ttk.Button(connection, text="刷新", command=self.refresh_ports).pack(side="left", padx=(0, 12))
        ttk.Label(connection, text="波特率", style="CardTitle.TLabel").pack(side="left", padx=(0, 8))
        ttk.Combobox(connection, textvariable=self.baud_var, values=[str(item) for item in BAUD_RATES], state="readonly", width=10).pack(side="left", padx=(0, 12))
        self.connect_button = ttk.Button(connection, text="连接", style="Accent.TButton", command=self.toggle_connection)
        self.connect_button.pack(side="left")
        self.disconnect_button = ttk.Button(connection, text="断连", command=self.disconnect)
        self.disconnect_button.pack(side="left", padx=(8, 0))
        ttk.Label(connection, textvariable=self.status_var, style="Muted.TLabel").pack(side="left", padx=16)
        ttk.Label(connection, textvariable=self.tx_var, style="Muted.TLabel").pack(side="right", padx=(12, 0))
        ttk.Label(connection, textvariable=self.rx_var, style="Muted.TLabel").pack(side="right")
        ttk.Button(connection, text="急停", style="Danger.TButton", command=self.estop).pack(side="right", padx=(0, 22))
        self.disconnect_button.configure(state="disabled")

        status = ttk.Frame(root, style="Card.TFrame", padding=10)
        status.pack(fill="x", pady=(0, 14))
        ttk.Label(status, textvariable=self.board_state_var, style="CardTitle.TLabel").pack(side="left")
        ttk.Label(status, textvariable=self.remote_var, style="CardMuted.TLabel").pack(side="left", padx=20)
        ttk.Label(status, text="安全策略：断开或 200 ms 无有效帧后板端停止输出", style="CardMuted.TLabel").pack(side="right")

        navigation = tk.Frame(root, bg="#e8f6fb")
        navigation.pack(fill="x", pady=(0, 2))
        for page in ("机械臂", "开关门", "夹爪"):
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
        self._build_single_motor_page("开关门", ENABLE_CONVEYOR, self.conveyor_position, self.conveyor_enable, "conveyor")
        self._build_single_motor_page("夹爪", ENABLE_GRIPPER, self.gripper_position, self.gripper_enable, "gripper")
        self._select_page("机械臂", log_change=False)

        log_frame = ttk.Frame(root, style="Card.TFrame", padding=10, height=330)
        log_frame.pack_propagate(False)
        log_frame.pack(side="bottom", fill="x", pady=(14, 0))
        ttk.Label(log_frame, text="运行日志", style="CardTitle.TLabel").pack(anchor="w")
        self.log_text = tk.Text(log_frame, height=11, bg="#f4fcff", fg="#1e2a33", insertbackground="#143747", relief="flat", state="disabled", font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True, pady=(6, 0))
        self.page_container.pack(fill="both", expand=True)

    def _build_arm_page(self) -> None:
        frame = ttk.Frame(self.page_container, padding=12)
        self.page_frames["机械臂"] = frame
        self.page_enable_vars["机械臂"] = self.arm_enable
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
        ttk.Label(left, text="J4310 · MIT 参数（p_des：输出轴角度）", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 10))
        self._field_at(left, 1, 0, "前馈 tau (t_ff)", self.j_tau, "Nm", -J4310_TORQUE_LIMIT, J4310_TORQUE_LIMIT, 0.05)
        self._field_at(left, 1, 3, "力矩限幅", self.j_torque_limit, "Nm", 0.1, J4310_TORQUE_LIMIT, 0.05)
        self._field_at(left, 2, 0, "刚度 Kp", self.j_kp, "", 0, J4310_KP_LIMIT, 1)
        self._field_at(left, 2, 3, "阻尼 Kd", self.j_kd, "", 0, J4310_KD_LIMIT, 0.05)
        self._field_at(left, 3, 0, "目标位置 p_des", self.j_position, "rad", -J4310_POSITION_LIMIT, J4310_POSITION_LIMIT, 0.05)
        self._field_at(left, 3, 3, "目标速度 v_des", self.j_velocity, "rad/s", -J4310_VELOCITY_LIMIT, J4310_VELOCITY_LIMIT, 0.1)
        j4310_test = ttk.Frame(left, style="Card.TFrame")
        j4310_test.grid(row=4, column=0, columnspan=6, sticky="w", pady=(6, 8))
        self._add_test_button(j4310_test, "机械臂", "j4310", "J4310")
        ttk.Label(left, text="M3508 · 位置目标（输出轴角度）", style="CardTitle.TLabel").grid(row=5, column=0, columnspan=6, sticky="w", pady=(0, 4))
        self._field_at(left, 6, 0, "M3508 #1", self.m3508_position_1, "rad", -100, 100, 0.05)
        self._field_at(left, 6, 3, "M3508 #2", self.m3508_position_2, "rad", -100, 100, 0.05)
        m3508_test = ttk.Frame(left, style="Card.TFrame")
        m3508_test.grid(row=7, column=0, columnspan=6, sticky="w", pady=(5, 3))
        self._add_test_button(m3508_test, "机械臂", "m3508", "M3508")
        bottom = ttk.Frame(left, style="Card.TFrame")
        bottom.grid(row=8, column=0, columnspan=6, sticky="w", pady=(3, 0))
        ttk.Checkbutton(bottom, text="允许机械臂输出", variable=self.arm_enable).pack(side="left", padx=(0, 12))
        self._add_action_buttons(bottom, "机械臂")
        self._build_pid_editor(right, "M3508 / C620 · PID 参数", self.m3508_speed_pid_vars, self.m3508_position_pid_vars)

    def _build_single_motor_page(self, title: str, mask: int, position_var: tk.DoubleVar, enable_var: tk.BooleanVar, key: str) -> None:
        frame = ttk.Frame(self.page_container, padding=16)
        self.page_frames[title] = frame
        self.page_enable_vars[title] = enable_var
        columns = ttk.Frame(frame)
        columns.pack(fill="both", expand=True)
        columns.columnconfigure(0, weight=3)
        columns.columnconfigure(1, weight=4)
        left = ttk.Frame(columns, style="Card.TFrame", padding=14)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        right = ttk.Frame(columns, style="Card.TFrame", padding=14)
        right.grid(row=0, column=1, sticky="nsew")
        model = "传送带 M2006" if key == "conveyor" else "夹爪 M2006"
        ttk.Label(left, text=f"{model} · 位置控制（输出轴角度）", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))
        self._field(left, 1, "输出轴目标位置", position_var, "rad", -100, 100, 0.05)
        ttk.Label(left, text="转子角度按减速比换算为输出轴角度（rad）。", style="CardMuted.TLabel").grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 12))
        ttk.Checkbutton(left, text=f"允许{title}输出", variable=enable_var).grid(row=3, column=0, columnspan=3, sticky="w", pady=(4, 4))
        buttons = ttk.Frame(left, style="Card.TFrame")
        buttons.grid(row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))
        self._add_page_buttons(buttons, title, key, "M2006")
        self._build_pid_editor(right, "M2006 / C610 · PID 参数", self.m2006_speed_pid_vars, self.m2006_position_pid_vars)

    def _field(self, parent: ttk.Frame, row: int, label: str, variable: tk.DoubleVar, unit: str, lower: float, upper: float, increment: float) -> None:
        self._field_at(parent, row, 0, label, variable, unit, lower, upper, increment)

    def _field_at(self, parent: ttk.Frame, row: int, column: int, label: str, variable: tk.DoubleVar, unit: str, lower: float, upper: float, increment: float) -> None:
        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row,
            column=column,
            sticky="w",
            padx=(18 if column > 0 else 0, 0),
            pady=4,
        )
        spin = ttk.Spinbox(parent, textvariable=variable, from_=lower, to=upper, increment=increment, width=16)
        spin.grid(row=row, column=column + 1, sticky="ew", padx=8, pady=4)
        ttk.Label(parent, text=unit, style="CardMuted.TLabel").grid(
            row=row,
            column=column + 2,
            sticky="w",
            padx=(0, 18),
            pady=4,
        )

    def _pid_vars(self, values: tuple[str, str, str, str, str, str]) -> dict[str, tk.DoubleVar]:
        return {
            "kp": tk.DoubleVar(value=float(values[1])),
            "ki": tk.DoubleVar(value=float(values[2])),
            "kd": tk.DoubleVar(value=float(values[3])),
            "integral_limit": tk.DoubleVar(value=float(values[4])),
            "output_limit": tk.DoubleVar(value=float(values[5].split()[0])),
        }

    def _build_pid_editor(
        self,
        parent: ttk.Frame,
        title: str,
        speed_vars: dict[str, tk.DoubleVar],
        position_vars: dict[str, tk.DoubleVar],
    ) -> None:
        ttk.Label(parent, text=title, style="CardTitle.TLabel").grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 4))
        ttk.Label(parent, text="每一项均可修改；速度环输出限幅为原始电流命令，位置环输出限幅为 rpm。", style="CardMuted.TLabel").grid(row=1, column=0, columnspan=6, sticky="w", pady=(0, 12))
        for column, header in enumerate(("环路", "Kp", "Ki", "Kd", "积分限幅", "输出限幅")):
            ttk.Label(parent, text=header, style="CardMuted.TLabel").grid(row=2, column=column, sticky="w", padx=(0, 8), pady=(0, 4))
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
        ttk.Label(parent, text=title, style="Card.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        specs = (
            ("kp", 0.0, 10000.0, 1.0),
            ("ki", 0.0, 10000.0, 1.0),
            ("kd", 0.0, 1000.0, 0.1),
            ("integral_limit", 0.0, 100000.0, 0.01),
            ("output_limit", 0.1, 32767.0 if not position else 1000.0, 1.0 if not position else 0.1),
        )
        for column, (name, lower, upper, increment) in enumerate(specs, start=1):
            spin = ttk.Spinbox(parent, textvariable=variables[name], from_=lower, to=upper, increment=increment, width=9)
            spin.grid(row=row, column=column, sticky="w", padx=(0, 8), pady=4)

    def _add_test_button(self, parent: ttk.Frame, page: str, target: str, target_name: str) -> None:
        test = ttk.Button(
            parent,
            text=f"测试 {target_name}",
            style="Accent.TButton",
            command=lambda: self.start_test(page, target, target_name),
        )
        test.pack(side="left", padx=(0, 8))
        self.page_test_buttons[target] = test

    def _add_action_buttons(self, parent: ttk.Frame, page: str) -> None:
        ttk.Button(parent, text="发送目标", command=lambda: self.send_page_now(page)).pack(side="left", padx=(0, 8))
        ttk.Button(parent, text="停止本页", command=lambda: self.stop_page(page)).pack(side="left")

    def _add_page_buttons(self, parent: ttk.Frame, page: str, target: str, target_name: str) -> None:
        self._add_test_button(parent, page, target, target_name)
        self._add_action_buttons(parent, page)

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
        if self.transport.connected:
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
        try:
            self.transport.open(port, int(self.baud_var.get()))
        except Exception as exc:
            self.status_var.set("连接失败")
            self.log(str(exc))
            return
        self.parser = FrameParser()
        self.handshaken = False
        self.handshake_started_at = time.monotonic()
        self.last_handshake = 0.0
        self.handshake_sequence = None
        self.connect_button.configure(text="握手中...", state="disabled")
        self.disconnect_button.configure(state="normal")
        self.status_var.set("握手中...")
        self.log(f"串口已打开，等待单片机握手: {port} @ {self.baud_var.get()} 8N1")
        self._send_handshake()

    def disconnect(self) -> None:
        self.test_runner.stop(log=False)
        for variable in self.page_enable_vars.values():
            variable.set(False)
        if not self.transport.connected:
            self._reset_connection_state()
            return
        if self.handshaken:
            self._send_estop_frame()
        self.transport.close()
        self._reset_connection_state()
        self.log("串口已断开")

    def _reset_connection_state(self) -> None:
        self.handshaken = False
        self.handshake_sequence = None
        self.handshake_started_at = 0.0
        self.last_handshake = 0.0
        self.parser = FrameParser()
        self.status_var.set("未连接")
        self.connect_button.configure(text="连接", state="normal")
        self.disconnect_button.configure(state="disabled")
        self.board_state_var.set("板端状态: --")
        self.remote_var.set("远程链路: --")

    def _send_handshake(self) -> None:
        if not self.transport.connected:
            return
        sequence = self._next_sequence()
        self.handshake_sequence = sequence
        self.transport.write(build_handshake_frame(sequence))
        self.tx_count += 1
        self.tx_var.set(f"TX {self.tx_count}")
        self.last_handshake = time.monotonic()

    def _next_sequence(self) -> int:
        value = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFF
        return value

    def _send(self, msg_type: int, payload: bytes = b"") -> None:
        if not self.transport.connected:
            return
        self.transport.write(encode_frame(msg_type, self._next_sequence(), payload))
        self.tx_count += 1
        self.tx_var.set(f"TX {self.tx_count}")

    def _send_estop_frame(self) -> None:
        self._send(MSG_ESTOP, b"\x01")

    def estop(self) -> None:
        self.test_runner.stop(log=False)
        for variable in self.page_enable_vars.values():
            variable.set(False)
        self._send_estop_frame()
        self.status_var.set("已发送急停")
        self.log("急停：板端进入错误状态，需重新上电或按固件流程清错")

    def _values(self) -> tuple[float, ...]:
        return (
            float(self.j_position.get()),
            float(self.j_velocity.get()),
            float(self.j_kp.get()),
            float(self.j_kd.get()),
            float(self.m3508_position_1.get()),
            float(self.m3508_position_2.get()),
            float(self.conveyor_position.get()),
            float(self.gripper_position.get()),
        )

    def _mask_for_page(self, page: str) -> int:
        return {"机械臂": ENABLE_ARM, "开关门": ENABLE_CONVEYOR, "夹爪": ENABLE_GRIPPER}[page]

    def _pid_values(self) -> tuple[float, ...]:
        groups = (
            self.m3508_speed_pid_vars,
            self.m3508_position_pid_vars,
            self.m2006_speed_pid_vars,
            self.m2006_position_pid_vars,
        )
        return tuple(float(group[name].get()) for group in groups for name in ("kp", "ki", "kd", "integral_limit", "output_limit"))

    def _build_position_frame(self, page: str, values: tuple[float, ...], enabled: bool = True) -> bytes:
        mask = self._mask_for_page(page) if enabled else 0
        payload = build_extended_position_payload(
            mask,
            values[0],
            values[1],
            values[2],
            values[3],
            float(self.j_tau.get()),
            float(self.j_torque_limit.get()),
            values[4],
            values[5],
            values[6],
            values[7],
            *self._pid_values(),
        )
        return encode_frame(MSG_UPPER_POSITION_CMD, self._next_sequence(), payload)

    def send_page_now(self, page: str) -> None:
        self._send_position_values(page, self._values(), self.page_enable_vars[page].get())

    def stop_page(self, page: str) -> None:
        self.page_enable_vars[page].set(False)
        self._send_position_values(page, self._values(), False)
        if self.test_runner.phase and self.test_runner.phase.page == page:
            self.test_runner.stop()
        self.log(f"{page} 已停止")

    def _send_position_values(self, page: str, values: tuple[float, ...], enabled: bool) -> None:
        if not self.transport.connected:
            self.status_var.set("未连接，目标未发送")
            return
        if not self.handshaken:
            self.status_var.set("尚未握手，目标未发送")
            return
        frame = self._build_position_frame(page, values, enabled)
        self.transport.write(frame)
        self.tx_count += 1
        self.tx_var.set(f"TX {self.tx_count}")

    def start_test(self, page: str, target: str, target_name: str) -> None:
        if self.test_runner.running:
            self.test_runner.stop()
            return
        if not self.transport.connected:
            self.status_var.set("请先连接串口")
            self.log("测试未启动：串口未连接")
            return
        if not self.handshaken:
            self.status_var.set("请等待握手完成")
            self.log("测试未启动：尚未与单片机握手")
            return
        self.page_enable_vars[page].set(True)
        values = self._values()
        baseline = {
            "j4310": (values[0],),
            "m3508": (values[4], values[5]),
            "conveyor": (values[6],),
            "gripper": (values[7],),
        }[target]
        self.test_runner.start(page, target, target_name, baseline)

    def set_test_button_state(self, running: bool) -> None:
        for page, button in self.page_test_buttons.items():
            if self.test_runner.phase is not None and page == self.test_runner.phase.target and running:
                button.configure(text="停止测试")
            else:
                target_names = {
                    "j4310": "J4310",
                    "m3508": "M3508",
                    "conveyor": "M2006",
                    "gripper": "M2006",
                }
                button.configure(text=f"测试 {target_names[page]}")

    def set_test_target(self, phase: TestPhase) -> None:
        values = list(self._values())
        if phase.direction == 0:
            targets = phase.baseline
        else:
            targets = tuple(base + phase.direction * TEST_ANGLE_RAD for base in phase.baseline)
        if phase.target == "j4310":
            values[0] = targets[0]
        elif phase.target == "m3508":
            values[4], values[5] = targets
        elif phase.target == "conveyor":
            values[6] = targets[0]
        else:
            values[7] = targets[0]
        self._send_position_values(phase.page, tuple(values), True)

    def _accept_handshake_ack(self, frame: Frame) -> None:
        if self.handshaken or self.handshake_sequence is None:
            return
        if frame.msg_type != MSG_ACK:
            return
        if frame.sequence != self.handshake_sequence or frame.payload != HANDSHAKE_MAGIC:
            return
        self.handshaken = True
        self.last_heartbeat = 0.0
        self.status_var.set("已握手")
        self.connect_button.configure(text="已握手", state="disabled")
        self.disconnect_button.configure(state="normal")
        self.log("已收到单片机握手确认，控制链路就绪")

    def _periodic_send(self) -> None:
        if self.transport.connected:
            now = time.monotonic()
            if not self.handshaken:
                if now - self.handshake_started_at >= HANDSHAKE_TIMEOUT_MS / 1000.0:
                    self.log("握手超时，未收到单片机确认")
                    self.disconnect()
                elif now - self.last_handshake >= HANDSHAKE_PERIOD_MS / 1000.0:
                    self._send_handshake()
            else:
                if now - self.last_heartbeat >= HEARTBEAT_PERIOD_MS / 1000.0:
                    self._send(MSG_HEARTBEAT)
                    self.last_heartbeat = now
                if self.test_runner.phase is None:
                    variable = self.page_enable_vars.get(self.active_page)
                    if variable is not None and variable.get():
                        self._send_position_values(self.active_page, self._values(), True)
        self.root.after(SEND_PERIOD_MS, self._periodic_send)

    def _poll_io(self) -> None:
        while True:
            try:
                data = self.transport.rx_queue.get_nowait()
            except queue.Empty:
                break
            self.rx_count += len(data)
            self.rx_var.set(f"RX {self.rx_count} B")
            for frame in self.parser.feed(data):
                if frame.msg_type == MSG_ACK:
                    self._accept_handshake_ack(frame)
                elif frame.msg_type == MSG_ROBOT_STATE:
                    try:
                        state = decode_robot_state(frame.payload)
                    except ValueError:
                        continue
                    self.board_state_var.set(f"板端状态: {STATE_NAMES.get(state['state'], str(state['state']))}")
                    self.remote_var.set(f"远程链路: {'活动' if state['remote_active'] else '空闲'} · RX序号 {state['last_rx_sequence']}")
        while True:
            try:
                message = self.transport.error_queue.get_nowait()
            except queue.Empty:
                break
            self.status_var.set(message)
            self.log(message)
        self.root.after(20, self._poll_io)

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{timestamp}  {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    UpperConsole(root)
    root.mainloop()


if __name__ == "__main__":
    main()
