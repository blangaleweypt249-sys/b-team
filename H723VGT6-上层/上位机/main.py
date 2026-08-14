# -*- coding: utf-8 -*-
"""H723VGT6 upper-control console."""

from __future__ import annotations

import math
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import ttk

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
    MSG_ESTOP,
    MSG_FAULT,
    MSG_HEARTBEAT,
    MSG_MOTOR_ACTION,
    MSG_MOTOR_ACTION_RESULT,
    MSG_DJI_TELEMETRY,
    MSG_ROBOT_STATE,
    MSG_UPPER_POSITION_CMD,
    Frame,
    FrameParser,
    build_motor_action_payload,
    build_handshake_frame,
    build_extended_position_payload,
    decode_motor_event,
    decode_motor_action_result,
    decode_dji_telemetry,
    decode_robot_state,
    encode_frame,
)


BAUD_RATES = (115200, 230400, 460800, 921600)
SEND_PERIOD_MS = 50
HEARTBEAT_PERIOD_MS = 100
HANDSHAKE_PERIOD_MS = 150
HANDSHAKE_TIMEOUT_MS = 3000
HANDSHAKE_OPEN_DELAY_MS = 300
BOARD_RESPONSE_TIMEOUT_MS = 1000
HANDSHAKE_SEQUENCE = 0x1234
J4310_POSITION_LIMIT = 12.5
J4310_VELOCITY_LIMIT = 30.0
J4310_KP_LIMIT = 500.0
J4310_KD_LIMIT = 5.0
J4310_TORQUE_LIMIT = 10.0
J4310_RAW_MIN_DEG = -math.degrees(J4310_POSITION_LIMIT)
J4310_RAW_MAX_DEG = math.degrees(J4310_POSITION_LIMIT)
POSITION_MIN_DEG = -90.0
POSITION_MAX_DEG = 90.0
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
        ("速度环", "80", "40", "0", "57.2958", "2458"),
        ("位置环", "60", "0", "0", "0", "150 rpm"),
    ),
    "M2006 / C610": (
        ("速度环", "30", "10", "0", "95.493", "2000"),
        ("位置环", "40", "0", "0", "0", "100 rpm"),
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
                    "开关门 M2006", "--", "等待主控反馈"
                )
            ),
            (2, 3, 2): tk.StringVar(
                value=format_dji_feedback(
                    "夹爪 M2006", "--", "等待主控反馈"
                )
            ),
        }
        self.j_position = tk.DoubleVar(value=0.0)
        self.j_velocity = tk.DoubleVar(value=2.0)
        self.j_kp = tk.DoubleVar(value=20.0)
        self.j_kd = tk.DoubleVar(value=0.5)
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
        self.m2006_speed_pid_vars = self._pid_vars(
            REFERENCE_PID_VALUES["M2006 / C610"][0]
        )
        self.m2006_position_pid_vars = self._pid_vars(
            REFERENCE_PID_VALUES["M2006 / C610"][1]
        )
        self.arm_enable = tk.BooleanVar(value=False)
        self.m3508_enable = tk.BooleanVar(value=False)
        self.conveyor_enable = tk.BooleanVar(value=False)
        self.gripper_enable = tk.BooleanVar(value=False)
        self.j4310_feedback_status = "未收到反馈"
        self.j4310_auto_return_status_received = False
        self.j4310_auto_return_enabled = False
        self.j4310_auto_return_storage_ready = False
        self.j4310_auto_return_active = False
        self.j4310_auto_return_stage = 0
        self.j4310_auto_return_pending: bool | None = None
        self.position_slider_controls: dict[str, tuple[tk.Widget, ...]] = {}
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
        style.configure("TCheckbutton", background="#dff3fa", foreground="#1f3442", font=("Microsoft YaHei UI", 10))
        style.map("TCheckbutton", background=[("active", "#dff3fa")])
        self._output_check_off = self._checkbox_indicator_image(selected=False)
        self._output_check_on = self._checkbox_indicator_image(selected=True)
        style.element_create(
            "OutputCheckbutton.indicator",
            "image",
            self._output_check_off,
            ("selected", self._output_check_on),
        )
        style.layout(
            "Output.TCheckbutton",
            [
                (
                    "Checkbutton.padding",
                    {
                        "sticky": "nswe",
                        "children": [
                            ("OutputCheckbutton.indicator", {"side": "left", "sticky": ""}),
                            (
                                "Checkbutton.focus",
                                {
                                    "side": "left",
                                    "sticky": "w",
                                    "children": [("Checkbutton.label", {"sticky": "nswe"})],
                                },
                            ),
                        ],
                    },
                )
            ],
        )
        style.configure("Output.TCheckbutton", background="#dff3fa", foreground="#1f3442", font=("Microsoft YaHei UI", 10))
        style.map("Output.TCheckbutton", background=[("active", "#dff3fa")])
        style.configure("TEntry", fieldbackground="#f7fdff", foreground="#1f3442", insertcolor="#143747")
        style.configure("TCombobox", fieldbackground="#f7fdff", foreground="#1f3442")

    def _checkbox_indicator_image(self, selected: bool) -> tk.PhotoImage:
        image = tk.PhotoImage(width=14, height=14)
        image.put("#000000" if selected else "#8b9aa2", to=(0, 0, 14, 14))
        if not selected:
            image.put("#f7fdff", to=(1, 1, 13, 13))
        return image

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
        ttk.Label(status, textvariable=self.board_state_var, style="CardTitle.TLabel").pack(side="left")
        ttk.Label(status, textvariable=self.remote_var, style="CardMuted.TLabel").pack(side="left", padx=20)
        ttk.Label(status, text="断开或 200 ms 无有效帧后停止输出", style="CardMuted.TLabel").pack(side="right")

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

        log_frame = ttk.Frame(root, style="Card.TFrame", padding=10, height=150)
        log_frame.pack_propagate(False)
        log_frame.pack(side="bottom", fill="x", pady=(14, 0))
        ttk.Label(log_frame, text="运行日志", style="CardTitle.TLabel").pack(anchor="w")
        self.log_text = tk.Text(log_frame, height=5, bg="#f4fcff", fg="#1e2a33", insertbackground="#143747", relief="flat", state="disabled", font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True, pady=(6, 0))
        self.page_container.pack(fill="both", expand=True)

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
            POSITION_MIN_DEG,
            POSITION_MAX_DEG,
            "j4310",
        )
        self.position_slider_controls["j4310"] = self.j_position_controls
        self._field_at(left, 3, 3, "目标速度 v_des", self.j_velocity, "rad/s", -J4310_VELOCITY_LIMIT, J4310_VELOCITY_LIMIT, 0.1)
        self.j4310_enable_check = ttk.Checkbutton(
            left,
            text="使能 J4310 输出",
            variable=self.arm_enable,
            style="Output.TCheckbutton",
            command=self._on_j4310_enable_changed,
        )
        self.j4310_enable_check.grid(
            row=4, column=0, columnspan=3, sticky="w", pady=(6, 3)
        )
        self.j4310_bus_label = ttk.Label(
            left,
            text="反馈状态: 未收到反馈\nFlash 是否就绪: --",
            style="CardMuted.TLabel",
            justify="left",
        )
        self.j4310_bus_label.grid(
            row=4,
            column=3,
            columnspan=3,
            sticky="w",
            padx=(18, 0),
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
            left, 7, 0, "M3508 #1", self.m3508_position_1
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
            self.m3508_sync_slider,
        )
        self.position_slider_controls["m3508"] = self.m3508_position_controls
        ttk.Label(left, text="FDCAN2 · ID 1 / ID 2", style="CardMuted.TLabel").grid(row=7, column=3, columnspan=3, rowspan=2, sticky="w", padx=(18, 0))
        self.m3508_enable_check = ttk.Checkbutton(
            left,
            text="允许 M3508 输出",
            variable=self.m3508_enable,
            style="Output.TCheckbutton",
        )
        self.m3508_enable_check.grid(
            row=10, column=0, columnspan=6, sticky="w", pady=(7, 3)
        )
        m3508_actions = ttk.Frame(left, style="Card.TFrame")
        m3508_actions.grid(
            row=11, column=0, columnspan=6, sticky="w", pady=(3, 0)
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
        self.m3508_stop_button.pack(side="left")
        m3508_feedback = ttk.Frame(left, style="Card.TFrame")
        m3508_feedback.grid(
            row=12, column=0, columnspan=6, sticky="ew", pady=(8, 0)
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

    def _build_single_motor_page(self, title: str, _mask: int, position_var: tk.DoubleVar, enable_var: tk.BooleanVar, key: str) -> None:
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
            POSITION_MIN_DEG,
            POSITION_MAX_DEG,
            key,
        )
        ttk.Label(left, text=f"FDCAN3 · ID {node_id} · {title}", style="CardMuted.TLabel").grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 12))
        ttk.Checkbutton(left, text="允许 M2006 输出", variable=enable_var, style="Output.TCheckbutton").grid(row=3, column=0, columnspan=3, sticky="w", pady=(4, 4))
        buttons = ttk.Frame(left, style="Card.TFrame")
        buttons.grid(row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))
        self._add_page_buttons(buttons, title, key, "M2006")
        diagnostic_key = (2, 3, node_id)
        ttk.Label(
            left,
            textvariable=self.dji_diagnostic_vars[diagnostic_key],
            style="CardMuted.TLabel",
            wraplength=440,
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(10, 0))
        self._build_pid_editor(
            right,
            "M2006 / C610 · PID 参数",
            self.m2006_speed_pid_vars,
            self.m2006_position_pid_vars,
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
        spin = ttk.Spinbox(
            controls,
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
    ) -> ttk.Spinbox:
        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row, column=column, sticky="w", pady=4
        )
        spinbox = ttk.Spinbox(
            parent,
            textvariable=variable,
            from_=POSITION_MIN_DEG,
            to=POSITION_MAX_DEG,
            increment=1.0,
            width=7,
        )
        spinbox.grid(row=row, column=column + 1, sticky="w", padx=8, pady=4)
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
            from_=POSITION_MIN_DEG,
            to=POSITION_MAX_DEG,
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
        self.m3508_position_1.set(-position if position else 0.0)
        self.m3508_position_2.set(position)

    def _field_at(self, parent: ttk.Frame, row: int, column: int, label: str, variable: tk.DoubleVar, unit: str, lower: float, upper: float, increment: float) -> None:
        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row,
            column=column,
            sticky="w",
            padx=(18 if column > 0 else 0, 0),
            pady=4,
        )
        spin = ttk.Spinbox(
            parent,
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

    @staticmethod
    def _pid_row(
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
            ("kp", 0.0, 10000.0, 1.0),
            ("ki", 0.0, 10000.0, 1.0),
            ("kd", 0.0, 1000.0, 0.1),
            ("integral_limit", 0.0, 100000.0, 0.01),
            ("output_limit", 0.1, 1000.0 if position else 32767.0,
             0.1 if position else 1.0),
        )
        for column, (name, lower, upper, increment) in enumerate(
            specs, start=1
        ):
            spin = ttk.Spinbox(
                parent,
                textvariable=variables[name],
                from_=lower,
                to=upper,
                increment=increment,
                width=9,
            )
            spin.grid(row=row, column=column, sticky="w", padx=(0, 8), pady=4)
    def _add_action_buttons(self, parent: ttk.Frame, page: str) -> None:
        ttk.Button(parent, text="发送目标", command=lambda: self.send_page_now(page)).pack(side="left", padx=(0, 8))
        ttk.Button(parent, text="停止发送", command=lambda: self.stop_page(page)).pack(side="left", padx=(0, 8))

    def _add_page_buttons(self, parent: ttk.Frame, page: str, target: str, target_name: str) -> None:
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
            (2, 3, 1): "开关门 M2006",
            (2, 3, 2): "夹爪 M2006",
        }
        for key, variable in self.dji_diagnostic_vars.items():
            variable.set(format_dji_feedback(names[key], "--", status))

    def _reset_connection_state(self) -> None:
        self._active_arm_mask = 0
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
        self._disable_all_outputs()
        self._send_estop_frame()
        self.status_var.set("已发送急停")
        self.log("急停：板端进入错误状态，需重新上电或按固件流程清错")

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
        if page == "机械臂":
            mask = ENABLE_J4310_ONLY if self.arm_enable.get() else 0
            if self.m3508_enable.get():
                mask |= ENABLE_M3508_ONLY
            return mask
        if page == "开关门":
            return ENABLE_CONVEYOR if self.conveyor_enable.get() else 0
        if page == "夹爪":
            return ENABLE_GRIPPER if self.gripper_enable.get() else 0
        return 0

    def _update_j4310_position_range(self) -> None:
        lower = POSITION_MIN_DEG
        upper = POSITION_MAX_DEG
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
        elif not self.j4310_auto_return_storage_ready:
            text = "重启归零：Flash未就绪"
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
        if not self.j4310_auto_return_storage_ready:
            self.status_var.set("J4310 重启归零不可用：Flash 未就绪")
            self.log("J4310 重启归零设置未发送：H723 Flash 尚未就绪")
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
            f"{'开启' if requested else '关闭'}，等待板端持久化回执"
        )

    def _apply_j4310_auto_return_state(self, status: object) -> None:
        if not isinstance(status, dict):
            self.j4310_auto_return_status_received = False
            self.j4310_auto_return_storage_ready = False
            self.j4310_auto_return_active = False
            self.j4310_auto_return_stage = 0
            self._refresh_j4310_status()
            self._update_j4310_auto_return_button()
            return
        self.j4310_auto_return_status_received = True
        self.j4310_auto_return_storage_ready = bool(status["storage_ready"])
        self.j4310_auto_return_active = bool(status["active"])
        self.j4310_auto_return_stage = int(status["stage"])
        if self.j4310_auto_return_pending is None:
            self.j4310_auto_return_enabled = bool(status["enabled"])
        self._refresh_j4310_status()
        self._update_j4310_auto_return_button()

    def _pid_values(self) -> tuple[float, ...]:
        groups = (
            self.m3508_speed_pid_vars,
            self.m3508_position_pid_vars,
            self.m2006_speed_pid_vars,
            self.m2006_position_pid_vars,
        )
        return tuple(
            float(group[name].get())
            for group in groups
            for name in ("kp", "ki", "kd", "integral_limit", "output_limit")
        )

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
    ) -> bytes:
        mask = (
            self._mask_for_page(page) if enabled else 0
        ) if enable_mask is None else enable_mask
        if j4310_stop:
            mask |= COMMAND_J4310_STOP
        lower = J4310_RAW_MIN_DEG
        upper = J4310_RAW_MAX_DEG
        j_position_deg = max(lower, min(upper, values[0]))
        payload = build_extended_position_payload(
            mask,
            math.radians(j_position_deg),
            values[1],
            values[2],
            values[3],
            float(self.j_tau.get()) if j_tau is None else j_tau,
            float(self.j_torque_limit.get()) if j_torque_limit is None else j_torque_limit,
            math.radians(values[4]),
            math.radians(values[5]),
            math.radians(values[6]),
            math.radians(values[7]),
            *(self._pid_values() if pid_values is None else pid_values),
        )
        return encode_frame(MSG_UPPER_POSITION_CMD, self._next_sequence(), payload)

    def send_page_now(self, page: str) -> None:
        if not self._page_output_enabled(page):
            self.status_var.set("请先勾选本页需要控制的电机")
            self.log(f"{page} 目标未发送：未允许任何电机输出")
            return
        if page == "开关门":
            self.gripper_enable.set(False)
        elif page == "夹爪":
            self.conveyor_enable.set(False)
        values = self._values()
        if self._send_position_values(page, values, True) and page == "机械臂":
            self._active_arm_mask = self._mask_for_page(page)
            self._sent_arm_values = values
            self._sent_j_tau = float(self.j_tau.get())
            self._sent_j_torque_limit = float(self.j_torque_limit.get())
            self._sent_pid_values = self._pid_values()

    def _start_slider_drag(self, target: str) -> None:
        if self._drag_target is not None and self._drag_target != target:
            self._stop_slider_target(self._drag_target)
        enable_var = self._slider_target_enable_var(target)
        enabled_by_slider = not enable_var.get()
        if enabled_by_slider:
            enable_var.set(True)
        if self._send_slider_target(target):
            self._drag_target = target
            self.status_var.set(f"{self._slider_target_name(target)} 滑块控制中")
            if enabled_by_slider:
                self.log(
                    f"{self._slider_target_name(target)} 滑块已自动允许输出"
                )
        elif enabled_by_slider:
            enable_var.set(False)

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

    def _refresh_j4310_status(self) -> None:
        flash_status = "--"
        if self.j4310_auto_return_status_received:
            flash_status = (
                "是" if self.j4310_auto_return_storage_ready else "否"
            )
        self.j4310_bus_label.configure(
            text=(
                f"反馈状态: {self.j4310_feedback_status}\n"
                f"Flash 是否就绪: {flash_status}"
            )
        )

    def _finish_slider_drag(self, target: str) -> None:
        if self._drag_target != target:
            return
        # Keep the final target active so a smooth position trajectory can finish.
        self._send_slider_target(target)
        self._drag_target = None
        self.status_var.set(f"{self._slider_target_name(target)} 保持当前目标")

    @staticmethod
    def _slider_target_name(target: str) -> str:
        return {
            "j4310": "J4310",
            "m3508": "M3508",
            "conveyor": "开关门 M2006",
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
                j_tau = float(self.j_tau.get())
                j_torque_limit = float(self.j_torque_limit.get())
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

        if target == "conveyor":
            self.gripper_enable.set(False)
        else:
            self.conveyor_enable.set(False)
        page, _enable_var = {
            "conveyor": ("开关门", self.conveyor_enable),
            "gripper": ("夹爪", self.gripper_enable),
        }[target]
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
            "conveyor": ("开关门", self.conveyor_enable),
            "gripper": ("夹爪", self.gripper_enable),
        }[target]
        enable_var.set(False)
        self._send_position_values(page, self._values(), False)

    def _arm_group_controls(
        self, group: str
    ) -> tuple[int, tk.BooleanVar, tuple[int, ...], str]:
        if group == "j4310":
            return ENABLE_J4310_ONLY, self.arm_enable, (0, 1, 2, 3), "J4310"
        if group == "m3508":
            return ENABLE_M3508_ONLY, self.m3508_enable, (4, 5), "M3508"
        raise ValueError(f"unknown arm group: {group}")

    def _on_j4310_enable_changed(self) -> None:
        if self.arm_enable.get():
            if not self.send_arm_group_now("j4310"):
                self.arm_enable.set(False)
        else:
            self.stop_arm_group("j4310")

    def send_arm_group_now(self, group: str) -> bool:
        bit, enable_var, value_indices, display_name = self._arm_group_controls(group)
        if not enable_var.get():
            self.status_var.set(f"请先勾选允许 {display_name} 输出")
            self.log(f"{display_name} 目标未发送：未允许输出")
            return False

        current_values = self._values()
        sent_values = list(self._sent_arm_values)
        for index in value_indices:
            sent_values[index] = current_values[index]
        j_tau = self._sent_j_tau
        j_torque_limit = self._sent_j_torque_limit
        pid_values = list(self._sent_pid_values)
        if group == "j4310":
            j_tau = float(self.j_tau.get())
            j_torque_limit = float(self.j_torque_limit.get())
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
        elif page == "开关门":
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
        self.log(f"{page} 已下发停止帧并取消允许输出")

    def _page_output_enabled(self, page: str) -> bool:
        if page == "机械臂":
            return bool(self.arm_enable.get() or self.m3508_enable.get())
        if page == "开关门":
            return bool(self.conveyor_enable.get())
        if page == "夹爪":
            return bool(self.gripper_enable.get())
        return False

    def _send_position_values(
        self,
        page: str,
        values: tuple[float, ...],
        enabled: bool,
        **frame_options,
    ) -> bool:
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
            (2, 3, 1): "开关门 M2006",
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
            if (
                self.handshaken
                and self.last_board_response > 0.0
                and now - self.last_board_response >=
                BOARD_RESPONSE_TIMEOUT_MS / 1000.0
            ):
                self._begin_handshake(now, reset_feedback=False)
                self.status_var.set("主控无响应，重新握手中...")
                self.connect_button.configure(text="重连中...", state="disabled")
                self.log("主控状态回包超时，保留输出授权并自动重新握手")
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
                    if state["j4310_position_valid"]:
                        output_deg = math.degrees(state["j4310_position_rad"])
                        self.j4310_output_position_var.set(
                            f"当前输出轴角度: {output_deg:.2f} deg"
                        )
                    else:
                        self.j4310_output_position_var.set("当前输出轴角度: -- deg")
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
                self.j4310_auto_return_storage_ready = True
                self.j4310_auto_return_enabled = requested
                state_name = "开启" if requested else "关闭"
                self.status_var.set(f"J4310 重启归零已{state_name}")
                self.log(
                    f"J4310 重启归零已{state_name}并保存到 H723 Flash"
                )
            elif result["status"] != 0:
                self.status_var.set("J4310 重启归零设置失败")
                self.log(
                    "J4310 重启归零设置失败：请检查 H723 W25Q Flash"
                )
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
