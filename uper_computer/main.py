from __future__ import annotations

import math
import struct
import sys
from dataclasses import dataclass

from PyQt6.QtCore import (
    QEvent,
    QObject,
    QThread,
    QTimer,
    Qt,
    pyqtSignal,
    pyqtSlot,
)
from PyQt6.QtGui import QCloseEvent, QFont
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QStyle,
    QVBoxLayout,
    QWidget,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUDRATE = 115200
SEND_PERIOD_MS = 100
STOP_REPEAT_COUNT = 3
FRAME_HEADER = bytes((0xA5, 0x5A))
MIN_LINEAR_SPEED_M_S = 0.05
MAX_LINEAR_SPEED_M_S = 5.0
DEFAULT_LINEAR_SPEED_M_S = 0.05
LINEAR_SPEED_STEP_M_S = 0.05
MIN_ANGULAR_SPEED_RAD_S = 0.01
MAX_ANGULAR_SPEED_RAD_S = 10.0
DEFAULT_ANGULAR_SPEED_RAD_S = 0.01
ANGULAR_SPEED_STEP_RAD_S = 0.01


@dataclass(frozen=True)
class VelocityCommand:
    vx_mm_s: int = 0
    vy_mm_s: int = 0
    wz_mrad_s: int = 0


@dataclass(frozen=True)
class MotionState:
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    counterclockwise: bool = False
    clockwise: bool = False


def calculate_checksum(payload: bytes) -> int:
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum


def build_velocity_frame(command: VelocityCommand) -> bytes:
    payload = struct.pack(
        "<hhh",
        command.vx_mm_s,
        command.vy_mm_s,
        command.wz_mrad_s,
    )
    return FRAME_HEADER + payload + bytes((calculate_checksum(payload),))


def calculate_velocity(
    state: MotionState,
    linear_speed_m_s: float,
    angular_speed_rad_s: float,
) -> VelocityCommand:
    vx_axis = int(state.forward) - int(state.backward)
    vy_axis = int(state.left) - int(state.right)
    wz_axis = int(state.counterclockwise) - int(state.clockwise)
    linear_speed_mm_s = linear_speed_m_s * 1000.0

    if vx_axis != 0 and vy_axis != 0:
        linear_speed_mm_s /= math.sqrt(2.0)

    return VelocityCommand(
        vx_mm_s=round(vx_axis * linear_speed_mm_s),
        vy_mm_s=round(vy_axis * linear_speed_mm_s),
        wz_mrad_s=round(wz_axis * angular_speed_rad_s * 1000.0),
    )


ZERO_FRAME = build_velocity_frame(VelocityCommand())


class SerialWorker(QObject):
    status_changed = pyqtSignal(bool, str)
    error_occurred = pyqtSignal(str)
    rx_bytes = pyqtSignal(bytes)
    tx_bytes = pyqtSignal(bytes)

    def __init__(self) -> None:
        super().__init__()
        self._serial = None
        self._poll_timer: QTimer | None = None

    @pyqtSlot()
    def start(self) -> None:
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(10)
        self._poll_timer.timeout.connect(self._poll_rx)
        self._poll_timer.start()

    @pyqtSlot(str, int)
    def open_port(self, port: str, baudrate: int) -> None:
        self._close_port()

        if serial is None:
            self.error_occurred.emit("缺少 pyserial，请先安装项目依赖。")
            self.status_changed.emit(False, "未连接")
            return

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0.2,
            )
        except Exception as exc:
            self._serial = None
            self.error_occurred.emit(f"打开串口失败：{exc}")
            self.status_changed.emit(False, "未连接")
            return

        self.status_changed.emit(True, f"{port} @ {baudrate}")

    @pyqtSlot(bytes)
    def send_bytes(self, payload: bytes) -> None:
        if not payload or self._serial is None or not self._serial.is_open:
            return

        try:
            self._serial.write(payload)
            self.tx_bytes.emit(payload)
        except Exception as exc:
            self.error_occurred.emit(f"串口发送失败：{exc}")
            self._close_port()
            self.status_changed.emit(False, "连接已断开")

    @pyqtSlot(bytes)
    def disconnect_port(self, stop_frame: bytes) -> None:
        self._send_stop_frames(stop_frame)
        self._close_port()
        self.status_changed.emit(False, "未连接")

    @pyqtSlot(bytes)
    def shutdown(self, stop_frame: bytes) -> None:
        if self._poll_timer is not None:
            self._poll_timer.stop()
        self._send_stop_frames(stop_frame)
        self._close_port()

    def _send_stop_frames(self, stop_frame: bytes) -> None:
        if not stop_frame or self._serial is None or not self._serial.is_open:
            return

        try:
            for _ in range(STOP_REPEAT_COUNT):
                self._serial.write(stop_frame)
            self._serial.flush()
        except Exception:
            pass

    def _close_port(self) -> None:
        if self._serial is None:
            return

        try:
            if self._serial.is_open:
                self._serial.close()
        finally:
            self._serial = None

    def _poll_rx(self) -> None:
        if self._serial is None or not self._serial.is_open:
            return

        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                payload = self._serial.read(waiting)
                if payload:
                    self.rx_bytes.emit(payload)
        except Exception as exc:
            self.error_occurred.emit(f"串口接收失败：{exc}")
            self._close_port()
            self.status_changed.emit(False, "连接已断开")


class KeyTile(QFrame):
    def __init__(self, key_text: str, action_text: str) -> None:
        super().__init__()
        self.setObjectName("KeyTile")
        self.setProperty("active", False)
        self.setFixedSize(112, 72)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 7, 8, 7)
        layout.setSpacing(2)

        key_label = QLabel(key_text)
        key_label.setObjectName("KeyName")
        key_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        action_label = QLabel(action_text)
        action_label.setObjectName("KeyAction")
        action_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(key_label)
        layout.addWidget(action_label)

    def set_active(self, active: bool) -> None:
        if self.property("active") == active:
            return
        self.setProperty("active", active)
        self.style().unpolish(self)
        self.style().polish(self)


class MainWindow(QMainWindow):
    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal(bytes)
    send_requested = pyqtSignal(bytes)
    shutdown_requested = pyqtSignal(bytes)

    KEY_FORWARD = Qt.Key.Key_W.value
    KEY_BACKWARD = Qt.Key.Key_S.value
    KEY_LEFT = Qt.Key.Key_A.value
    KEY_RIGHT = Qt.Key.Key_D.value
    KEY_CCW = Qt.Key.Key_Q.value
    KEY_CW = Qt.Key.Key_E.value
    MOTION_KEYS = {
        KEY_FORWARD,
        KEY_BACKWARD,
        KEY_LEFT,
        KEY_RIGHT,
        KEY_CCW,
        KEY_CW,
    }

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("机器人运动控制")
        self.setMinimumSize(860, 610)
        self.resize(940, 650)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setFont(QFont("Microsoft YaHei UI", 10))

        self._connected = False
        self._pressed_keys: set[int] = set()
        self._command = VelocityCommand()
        self._rx_count = 0
        self._tx_count = 0

        self._serial_thread = QThread(self)
        self._serial_worker = SerialWorker()
        self._serial_worker.moveToThread(self._serial_thread)
        self._serial_thread.started.connect(self._serial_worker.start)
        self.connect_requested.connect(self._serial_worker.open_port)
        self.disconnect_requested.connect(self._serial_worker.disconnect_port)
        self.send_requested.connect(self._serial_worker.send_bytes)
        self.shutdown_requested.connect(
            self._serial_worker.shutdown,
            type=Qt.ConnectionType.BlockingQueuedConnection,
        )
        self._serial_worker.status_changed.connect(self._on_serial_status)
        self._serial_worker.error_occurred.connect(self._on_serial_error)
        self._serial_worker.rx_bytes.connect(self._on_rx_bytes)
        self._serial_worker.tx_bytes.connect(self._on_tx_bytes)
        self._serial_thread.start()

        self._build_ui()
        self._apply_style()

        self._port_timer = QTimer(self)
        self._port_timer.setInterval(1500)
        self._port_timer.timeout.connect(self.scan_ports)
        self._port_timer.start()

        self._send_timer = QTimer(self)
        self._send_timer.setInterval(SEND_PERIOD_MS)
        self._send_timer.timeout.connect(self._send_active_command)
        self._send_timer.start()

        QApplication.instance().installEventFilter(self)
        self.scan_ports()
        self._update_command(send_now=False)

    def _build_ui(self) -> None:
        central = QWidget(self)
        central.setObjectName("Central")
        root = QVBoxLayout(central)
        root.setContentsMargins(20, 18, 20, 18)
        root.setSpacing(14)
        self.setCentralWidget(central)

        header = QHBoxLayout()
        header.setSpacing(12)
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("机器人运动控制")
        title.setObjectName("Title")
        serial_spec = QLabel("UART4   115200 / 8N1   10 Hz")
        serial_spec.setObjectName("Subtle")
        title_box.addWidget(title)
        title_box.addWidget(serial_spec)
        header.addLayout(title_box)
        header.addStretch(1)

        self.status_dot = QFrame()
        self.status_dot.setObjectName("StatusDot")
        self.status_dot.setProperty("connected", False)
        self.status_dot.setFixedSize(10, 10)
        self.status_label = QLabel("未连接")
        self.status_label.setObjectName("StatusText")
        header.addWidget(self.status_dot)
        header.addWidget(self.status_label)
        root.addLayout(header)

        connection = QFrame()
        connection.setObjectName("Section")
        connection_layout = QHBoxLayout(connection)
        connection_layout.setContentsMargins(16, 14, 16, 14)
        connection_layout.setSpacing(10)

        connection_layout.addWidget(QLabel("串口"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(300)
        self.port_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        connection_layout.addWidget(self.port_combo, 1)

        self.refresh_button = QPushButton("刷新")
        self.refresh_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_BrowserReload)
        )
        self.refresh_button.clicked.connect(self.scan_ports)
        connection_layout.addWidget(self.refresh_button)

        self.connect_button = QPushButton("连接")
        self.connect_button.setObjectName("PrimaryButton")
        self.connect_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_DialogApplyButton)
        )
        self.connect_button.clicked.connect(self._toggle_connection)
        connection_layout.addWidget(self.connect_button)
        root.addWidget(connection)

        body = QHBoxLayout()
        body.setSpacing(14)
        root.addLayout(body, 1)

        settings_group = QGroupBox("控制参数")
        settings_group.setMinimumWidth(285)
        settings_layout = QGridLayout(settings_group)
        settings_layout.setContentsMargins(16, 18, 16, 16)
        settings_layout.setHorizontalSpacing(10)
        settings_layout.setVerticalSpacing(14)

        self.linear_speed_spin = QDoubleSpinBox()
        self.linear_speed_spin.setRange(
            MIN_LINEAR_SPEED_M_S,
            MAX_LINEAR_SPEED_M_S,
        )
        self.linear_speed_spin.setDecimals(3)
        self.linear_speed_spin.setSingleStep(LINEAR_SPEED_STEP_M_S)
        self.linear_speed_spin.setValue(DEFAULT_LINEAR_SPEED_M_S)
        self.linear_speed_spin.setSuffix(" m/s")
        self.linear_speed_spin.valueChanged.connect(self._speed_changed)
        self.linear_speed_spin.editingFinished.connect(self.setFocus)

        self.angular_speed_spin = QDoubleSpinBox()
        self.angular_speed_spin.setRange(
            MIN_ANGULAR_SPEED_RAD_S,
            MAX_ANGULAR_SPEED_RAD_S,
        )
        self.angular_speed_spin.setDecimals(3)
        self.angular_speed_spin.setSingleStep(ANGULAR_SPEED_STEP_RAD_S)
        self.angular_speed_spin.setValue(DEFAULT_ANGULAR_SPEED_RAD_S)
        self.angular_speed_spin.setSuffix(" rad/s")
        self.angular_speed_spin.valueChanged.connect(self._speed_changed)
        self.angular_speed_spin.editingFinished.connect(self.setFocus)

        self.keyboard_check = QCheckBox("键盘控制")
        self.keyboard_check.setChecked(True)
        self.keyboard_check.toggled.connect(self._keyboard_toggled)

        settings_layout.addWidget(QLabel("平移速度"), 0, 0)
        settings_layout.addWidget(self.linear_speed_spin, 0, 1)
        settings_layout.addWidget(QLabel("旋转速度"), 1, 0)
        settings_layout.addWidget(self.angular_speed_spin, 1, 1)
        settings_layout.addWidget(self.keyboard_check, 2, 0, 1, 2)

        settings_layout.addWidget(self._divider(), 3, 0, 1, 2)
        self.tx_count_label = QLabel("TX 0 帧")
        self.rx_count_label = QLabel("RX 0 字节")
        self.tx_count_label.setObjectName("Metric")
        self.rx_count_label.setObjectName("Metric")
        settings_layout.addWidget(self.tx_count_label, 4, 0, 1, 2)
        settings_layout.addWidget(self.rx_count_label, 5, 0, 1, 2)
        settings_layout.setRowStretch(6, 1)

        self.stop_button = QPushButton("停止")
        self.stop_button.setObjectName("StopButton")
        self.stop_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_MediaStop)
        )
        self.stop_button.setMinimumHeight(42)
        self.stop_button.clicked.connect(self._stop_motion)
        settings_layout.addWidget(self.stop_button, 7, 0, 1, 2)
        body.addWidget(settings_group)

        keys_group = QGroupBox("运动控制")
        keys_layout = QGridLayout(keys_group)
        keys_layout.setContentsMargins(20, 22, 20, 20)
        keys_layout.setHorizontalSpacing(10)
        keys_layout.setVerticalSpacing(10)

        key_defs = {
            self.KEY_FORWARD: ("W", "前进"),
            self.KEY_BACKWARD: ("S", "后退"),
            self.KEY_LEFT: ("A", "左移"),
            self.KEY_RIGHT: ("D", "右移"),
            self.KEY_CCW: ("Q", "逆时针"),
            self.KEY_CW: ("E", "顺时针"),
        }
        self.key_tiles = {
            key: KeyTile(key_text, action_text)
            for key, (key_text, action_text) in key_defs.items()
        }

        keys_layout.addWidget(self.key_tiles[self.KEY_FORWARD], 0, 1)
        keys_layout.addWidget(self.key_tiles[self.KEY_LEFT], 1, 0)
        keys_layout.addWidget(self.key_tiles[self.KEY_BACKWARD], 1, 1)
        keys_layout.addWidget(self.key_tiles[self.KEY_RIGHT], 1, 2)
        keys_layout.addWidget(self.key_tiles[self.KEY_CCW], 2, 0)
        keys_layout.addWidget(self.key_tiles[self.KEY_CW], 2, 2)
        keys_layout.setColumnStretch(0, 1)
        keys_layout.setColumnStretch(1, 1)
        keys_layout.setColumnStretch(2, 1)
        keys_layout.setRowStretch(3, 1)
        body.addWidget(keys_group, 1)

        command_bar = QFrame()
        command_bar.setObjectName("CommandBar")
        command_layout = QHBoxLayout(command_bar)
        command_layout.setContentsMargins(16, 13, 16, 13)
        command_layout.setSpacing(24)
        command_layout.addWidget(QLabel("当前指令"))
        self.vx_label = QLabel()
        self.vy_label = QLabel()
        self.wz_label = QLabel()
        for label in (self.vx_label, self.vy_label, self.wz_label):
            label.setObjectName("CommandValue")
            label.setMinimumWidth(175)
            command_layout.addWidget(label)
        command_layout.addStretch(1)
        root.addWidget(command_bar)

    @staticmethod
    def _divider() -> QFrame:
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setObjectName("Divider")
        return divider

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                color: #18212b;
                font-family: "Microsoft YaHei UI";
                font-size: 14px;
                letter-spacing: 0px;
            }
            QWidget#Central {
                background: #f3f5f7;
            }
            QLabel#Title {
                font-size: 24px;
                font-weight: 700;
            }
            QLabel#Subtle, QLabel#Metric {
                color: #68737f;
            }
            QLabel#StatusText {
                font-weight: 600;
            }
            QFrame#StatusDot {
                background: #9aa3ad;
                border-radius: 5px;
            }
            QFrame#StatusDot[connected="true"] {
                background: #16844f;
            }
            QFrame#Section, QFrame#CommandBar, QGroupBox {
                background: #ffffff;
                border: 1px solid #cfd6dd;
                border-radius: 6px;
            }
            QGroupBox {
                font-weight: 700;
                margin-top: 10px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
                background: #f3f5f7;
            }
            QComboBox, QDoubleSpinBox {
                min-height: 34px;
                padding: 0 9px;
                background: #ffffff;
                border: 1px solid #b9c2cb;
                border-radius: 4px;
            }
            QComboBox:focus, QDoubleSpinBox:focus {
                border: 2px solid #2563ad;
            }
            QPushButton {
                min-height: 34px;
                padding: 0 13px;
                background: #ffffff;
                border: 1px solid #b9c2cb;
                border-radius: 4px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #eef2f5;
            }
            QPushButton:disabled {
                color: #8b949e;
                background: #e9edf0;
            }
            QPushButton#PrimaryButton {
                color: #ffffff;
                background: #2563ad;
                border-color: #2563ad;
            }
            QPushButton#PrimaryButton:hover {
                background: #1f568f;
            }
            QPushButton#StopButton {
                color: #ffffff;
                background: #bf3434;
                border-color: #bf3434;
            }
            QPushButton#StopButton:hover {
                background: #a92d2d;
            }
            QFrame#KeyTile {
                background: #f8fafb;
                border: 1px solid #b9c2cb;
                border-radius: 6px;
            }
            QFrame#KeyTile[active="true"] {
                background: #dbeafe;
                border: 2px solid #2563ad;
            }
            QLabel#KeyName {
                font-size: 21px;
                font-weight: 800;
            }
            QLabel#KeyAction {
                color: #5e6975;
                font-size: 12px;
            }
            QLabel#CommandValue {
                color: #173b64;
                font-family: Consolas, monospace;
                font-size: 16px;
                font-weight: 700;
            }
            QFrame#Divider {
                color: #d9dfe5;
            }
            """
        )

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        del watched

        if event.type() == QEvent.Type.ApplicationDeactivate:
            self._stop_motion()
            return False
        if event.type() not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return False

        key = event.key()
        if key not in self.MOTION_KEYS:
            return False
        if event.isAutoRepeat():
            return True
        if not self.keyboard_check.isChecked():
            return False

        if event.type() == QEvent.Type.KeyPress:
            self._pressed_keys.add(key)
        else:
            self._pressed_keys.discard(key)

        self._update_command(send_now=True)
        return True

    def scan_ports(self) -> None:
        if self._connected:
            return

        previous = self.port_combo.currentData()
        if list_ports is None:
            ports = []
        else:
            ports = sorted(list_ports.comports(), key=lambda item: item.device)
        self.port_combo.clear()

        if not ports:
            self.port_combo.addItem("未发现串口", None)
            self.connect_button.setEnabled(False)
            return

        for port in ports:
            description = port.description or "Serial Port"
            self.port_combo.addItem(
                f"{port.device}  ·  {description}",
                port.device,
            )

        if previous is not None:
            index = self.port_combo.findData(previous)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
        self.connect_button.setEnabled(True)

    def _toggle_connection(self) -> None:
        if self._connected:
            self._clear_motion(send_now=False)
            self.disconnect_requested.emit(ZERO_FRAME)
            return

        port = self.port_combo.currentData()
        if port is None:
            self._set_status_message("没有可连接的串口", error=True)
            return

        self.connect_button.setEnabled(False)
        self._set_status_message("正在连接…")
        self.connect_requested.emit(port, BAUDRATE)

    def _on_serial_status(self, connected: bool, detail: str) -> None:
        self._connected = connected
        self.status_label.setStyleSheet("")
        self.status_dot.setProperty("connected", connected)
        self.status_dot.style().unpolish(self.status_dot)
        self.status_dot.style().polish(self.status_dot)
        self.status_label.setText(detail)
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)

        self.connect_button.setEnabled(connected or self.port_combo.currentData() is not None)
        self.connect_button.setText("断开" if connected else "连接")
        if connected:
            icon = QStyle.StandardPixmap.SP_DialogCloseButton
        else:
            icon = QStyle.StandardPixmap.SP_DialogApplyButton
        self.connect_button.setIcon(self.style().standardIcon(icon))

        self._clear_motion(send_now=connected)

    def _on_serial_error(self, message: str) -> None:
        self._set_status_message(message, error=True)
        if not self._connected:
            self.connect_button.setEnabled(self.port_combo.currentData() is not None)

    def _set_status_message(self, message: str, error: bool = False) -> None:
        self.status_label.setText(message)
        self.status_label.setStyleSheet("color: #b42318;" if error else "")
        if not error:
            self.status_label.setStyleSheet("")

    def _speed_changed(self) -> None:
        self._update_command(send_now=True)

    def _keyboard_toggled(self, enabled: bool) -> None:
        if not enabled:
            self._clear_motion(send_now=True)
        self.setFocus()

    def _update_command(self, send_now: bool) -> None:
        state = MotionState(
            forward=self.KEY_FORWARD in self._pressed_keys,
            backward=self.KEY_BACKWARD in self._pressed_keys,
            left=self.KEY_LEFT in self._pressed_keys,
            right=self.KEY_RIGHT in self._pressed_keys,
            counterclockwise=self.KEY_CCW in self._pressed_keys,
            clockwise=self.KEY_CW in self._pressed_keys,
        )
        self._command = calculate_velocity(
            state,
            self.linear_speed_spin.value(),
            self.angular_speed_spin.value(),
        )

        for key, tile in self.key_tiles.items():
            tile.set_active(key in self._pressed_keys)

        self.vx_label.setText(f"Vx {self._command.vx_mm_s / 1000.0:+.3f} m/s")
        self.vy_label.setText(f"Vy {self._command.vy_mm_s / 1000.0:+.3f} m/s")
        self.wz_label.setText(
            f"Wz {self._command.wz_mrad_s / 1000.0:+.3f} rad/s"
        )

        if send_now:
            self._send_current_command()

    def _send_current_command(self) -> None:
        if not self._connected:
            return
        self.send_requested.emit(build_velocity_frame(self._command))

    def _send_active_command(self) -> None:
        if self._command == VelocityCommand():
            return
        self._send_current_command()

    def _clear_motion(self, send_now: bool) -> None:
        self._pressed_keys.clear()
        self._update_command(send_now=send_now)

    def _stop_motion(self) -> None:
        self._clear_motion(send_now=True)
        self.setFocus()

    def _on_tx_bytes(self, payload: bytes) -> None:
        del payload
        self._tx_count += 1
        self.tx_count_label.setText(f"TX {self._tx_count} 帧")

    def _on_rx_bytes(self, payload: bytes) -> None:
        self._rx_count += len(payload)
        self.rx_count_label.setText(f"RX {self._rx_count} 字节")

    def closeEvent(self, event: QCloseEvent) -> None:
        self._send_timer.stop()
        self._port_timer.stop()
        self._clear_motion(send_now=False)
        QApplication.instance().removeEventFilter(self)

        if self._serial_thread.isRunning():
            self.shutdown_requested.emit(ZERO_FRAME)
            self._serial_thread.quit()
            self._serial_thread.wait(1500)

        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Motion Console")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
