from __future__ import annotations

import json
import math
import os
import struct
import sys
from dataclasses import dataclass
from datetime import datetime

from PyQt6.QtCore import (
    QEvent,
    QObject,
    QPoint,
    QPointF,
    Qt,
    QThread,
    QTimer,
    pyqtSignal,
    pyqtSlot,
)
from PyQt6.QtGui import (
    QCloseEvent,
    QFont,
    QImage,
    QPainter,
    QPen,
    QPixmap,
    QTextCursor,
)
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
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


COMMON_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
DEFAULT_BAUDRATE = 115200
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

MAX_LOG_LINES = 500

# 配置文件路径（和程序同目录）
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")


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
    vx_axis = int(state.right) - int(state.left)
    vy_axis = int(state.forward) - int(state.backward)
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


class TabButton(QFrame):
    """苹果风格标签按钮 - 带小圆点指示器"""

    clicked = pyqtSignal()

    def __init__(self, text: str, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("TabButton")
        self.setProperty("active", False)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedHeight(36)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 0, 18, 0)
        layout.setSpacing(8)

        self._dot = QFrame()
        self._dot.setObjectName("TabDot")
        self._dot.setFixedSize(6, 6)
        layout.addWidget(self._dot)

        self._label = QLabel(text)
        self._label.setObjectName("TabButtonText")
        layout.addWidget(self._label)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)

    def set_active(self, active: bool) -> None:
        if self.property("active") == active:
            return
        self.setProperty("active", active)
        self.style().unpolish(self)
        self.style().polish(self)
        self._dot.style().unpolish(self._dot)
        self._dot.style().polish(self._dot)
        self._label.style().unpolish(self._label)
        self._label.style().polish(self._label)

    def text(self) -> str:
        return self._label.text()


class FieldMapView(QGraphicsView):
    """场地地图视图 - 支持鼠标交互和坐标显示"""

    coordinate_changed = pyqtSignal(float, float)  # x_m, y_m（实际坐标，米）
    origin_set = pyqtSignal(float, float)  # 原点像素坐标

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("FieldMapView")
        self.setRenderHints(
            QPainter.RenderHint.Antialiasing
            | QPainter.RenderHint.SmoothPixmapTransform
        )
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMouseTracking(True)
        self.setFrameShape(QFrame.Shape.NoFrame)

        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        self._pixmap_item: QGraphicsPixmapItem | None = None
        self._original_pixmap: QPixmap | None = None

        # 场地参数
        self._field_width_m = 3.0  # 场地宽（米）
        self._field_height_m = 2.0  # 场地高（米）
        self._origin_px = QPointF(0, 0)  # 原点在照片上的像素坐标
        self._setting_origin = False  # 是否正在设置原点模式

    def set_field_image(self, image_path: str) -> bool:
        """加载场地照片"""
        pixmap = QPixmap(image_path)
        if pixmap.isNull():
            return False

        self._original_pixmap = pixmap
        if self._pixmap_item is None:
            self._pixmap_item = self._scene.addPixmap(pixmap)
        else:
            self._pixmap_item.setPixmap(pixmap)

        self._scene.setSceneRect(pixmap.rect().toRectF())
        # 默认原点在左下角
        self._origin_px = QPointF(0, pixmap.height())
        self._fit_image()
        self.viewport().update()
        return True

    def set_field_size(self, width_m: float, height_m: float) -> None:
        """设置场地实际尺寸（米）"""
        self._field_width_m = max(width_m, 0.1)
        self._field_height_m = max(height_m, 0.1)

    def start_set_origin(self) -> None:
        """进入设置原点模式"""
        self._setting_origin = True
        self.setCursor(Qt.CursorShape.CrossCursor)

    def cancel_set_origin(self) -> None:
        """取消设置原点"""
        self._setting_origin = False
        self.setCursor(Qt.CursorShape.ArrowCursor)

    def is_setting_origin(self) -> bool:
        return self._setting_origin

    def get_origin_m(self) -> tuple[float, float]:
        """获取原点在场地坐标系中的位置（米）"""
        return self._pixel_to_meter(self._origin_px)

    def set_origin_px(self, pos: QPointF) -> None:
        """直接设置原点像素位置"""
        self._origin_px = pos
        self.viewport().update()

    def _pixel_to_meter(self, px_pos: QPointF) -> tuple[float, float]:
        """像素坐标转实际坐标（米），相对于原点"""
        if self._original_pixmap is None:
            return (0.0, 0.0)

        img_w = self._original_pixmap.width()
        img_h = self._original_pixmap.height()
        if img_w == 0 or img_h == 0:
            return (0.0, 0.0)

        # 计算像素到米的比例（取宽高比例的较小值，保持照片比例）
        scale_x = self._field_width_m / img_w
        scale_y = self._field_height_m / img_h
        scale = min(scale_x, scale_y)

        # 计算偏移（居中对齐）
        offset_x = (img_w - self._field_width_m / scale) / 2 if scale > 0 else 0
        offset_y = (img_h - self._field_height_m / scale) / 2 if scale > 0 else 0

        # 相对于原点的坐标（x向右为正，y向上为正）
        dx = (px_pos.x() - self._origin_px.x()) * scale
        dy = (self._origin_px.y() - px_pos.y()) * scale  # y轴翻转

        return (dx, dy)

    def _fit_image(self) -> None:
        """自适应缩放图片以填满视图"""
        if self._original_pixmap is None:
            return

        self.fitInView(
            self._scene.sceneRect(),
            Qt.AspectRatioMode.KeepAspectRatio,
        )

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._fit_image()

    def mouseMoveEvent(self, event) -> None:
        super().mouseMoveEvent(event)
        if self._original_pixmap is None:
            return

        scene_pos = self.mapToScene(event.pos())
        x_m, y_m = self._pixel_to_meter(scene_pos)
        self.coordinate_changed.emit(x_m, y_m)

    def mousePressEvent(self, event) -> None:
        if self._setting_origin and event.button() == Qt.MouseButton.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            # 限制在图片范围内
            if self._original_pixmap is not None:
                rect = self._original_pixmap.rect().toRectF()
                if rect.contains(scene_pos):
                    self._origin_px = scene_pos
                    self._setting_origin = False
                    self.setCursor(Qt.CursorShape.ArrowCursor)
                    x_m, y_m = self._pixel_to_meter(scene_pos)
                    self.origin_set.emit(x_m, y_m)
                    self.viewport().update()
            return

        super().mousePressEvent(event)

    def drawForeground(self, painter: QPainter, rect) -> None:
        """在前景绘制原点标记和坐标网格"""
        super().drawForeground(painter, rect)

        if self._original_pixmap is None:
            return

        # 绘制原点标记
        painter.save()
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 原点十字
        pen = QPen(QColor("#ff3b30"))
        pen.setWidth(2)
        painter.setPen(pen)

        ox = self._origin_px.x()
        oy = self._origin_px.y()
        size = 12

        painter.drawLine(int(ox - size), int(oy), int(ox + size), int(oy))
        painter.drawLine(int(ox), int(oy - size), int(ox), int(oy + size))

        # 原点圆圈
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(ox, oy), 6, 6)

        # 原点标签
        painter.setPen(QColor("#ff3b30"))
        font = painter.font()
        font.setBold(True)
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(int(ox + 10), int(oy - 8), "O (0,0)")

        painter.restore()


# 需要导入 QColor
from PyQt6.QtGui import QColor  # noqa: E402


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
        self.setMinimumSize(900, 640)
        self.resize(1000, 680)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.setFont(QFont(".AppleSystemUIFont", 13))

        self._connected = False
        self._current_baudrate = DEFAULT_BAUDRATE
        self._pressed_keys: set[int] = set()
        self._command = VelocityCommand()
        self._rx_count = 0
        self._tx_count = 0
        self._auto_scroll = True
        self._show_hex = True

        # 日志过滤
        self._filter_tx = True
        self._filter_rx = True
        self._log_buffer: list[tuple[str, str, bytes, str]] = []

        # 定位页配置
        self._field_image_path = ""
        self._field_width_m = 3.0
        self._field_height_m = 2.0
        self._origin_px_x = 0.0
        self._origin_px_y = 0.0

        # 加载配置
        self._load_config()

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

        # 加载保存的场地照片
        if self._field_image_path and os.path.exists(self._field_image_path):
            self.field_map.set_field_image(self._field_image_path)
            self.field_map.set_field_size(self._field_width_m, self._field_height_m)
            self.field_map.set_origin_px(QPointF(self._origin_px_x, self._origin_px_y))

    def _build_ui(self) -> None:
        central = QWidget(self)
        central.setObjectName("Central")
        root = QVBoxLayout(central)
        root.setContentsMargins(28, 20, 28, 24)
        root.setSpacing(16)
        self.setCentralWidget(central)

        # ── Header ──
        header = QHBoxLayout()
        header.setSpacing(14)
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("机器人运动控制")
        title.setObjectName("Title")
        serial_spec = QLabel("UART4   ·   8N1   ·   10 Hz")
        serial_spec.setObjectName("Subtle")
        title_box.addWidget(title)
        title_box.addWidget(serial_spec)
        header.addLayout(title_box)
        header.addStretch(1)

        # 状态胶囊
        status_pill = QFrame()
        status_pill.setObjectName("StatusPill")
        status_pill.setProperty("connected", False)
        pill_layout = QHBoxLayout(status_pill)
        pill_layout.setContentsMargins(12, 6, 14, 6)
        pill_layout.setSpacing(8)

        self.status_dot = QFrame()
        self.status_dot.setObjectName("StatusDot")
        self.status_dot.setProperty("connected", False)
        self.status_dot.setFixedSize(8, 8)
        self.status_label = QLabel("未连接")
        self.status_label.setObjectName("StatusText")
        pill_layout.addWidget(self.status_dot)
        pill_layout.addWidget(self.status_label)

        header.addWidget(status_pill)
        root.addLayout(header)

        # ── 顶部标签栏 ──
        tab_bar = QFrame()
        tab_bar.setObjectName("TabBar")
        tab_layout = QHBoxLayout(tab_bar)
        tab_layout.setContentsMargins(3, 3, 3, 3)
        tab_layout.setSpacing(0)

        self._tab_buttons = []
        tab_names = ["调试", "串口", "定位"]
        for i, name in enumerate(tab_names):
            btn = TabButton(name)
            btn.clicked.connect(lambda idx=i: self._switch_tab(idx))
            tab_layout.addWidget(btn)
            self._tab_buttons.append(btn)

        root.addWidget(tab_bar)

        # ── 页面堆栈 ──
        self._stack = QStackedWidget()
        root.addWidget(self._stack, 1)

        # 第0页：调试页（控制参数 + 连接 + 指令）
        self._build_debug_page()
        # 第1页：串口页（串口配置 + 日志）
        self._build_serial_page()
        # 第2页：定位页（场地地图 + 坐标交互）
        self._build_location_page()

        self._switch_tab(0)

    def _build_debug_page(self) -> None:
        """调试页：控制参数 + 连接控制 + 指令栏"""
        page = QWidget()
        page_layout = QHBoxLayout(page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.setSpacing(18)

        # 左侧：控制参数卡片
        settings_group = QFrame()
        settings_group.setObjectName("CardPanel")
        settings_group.setMinimumWidth(280)
        settings_group.setMaximumWidth(340)
        settings_layout = QVBoxLayout(settings_group)
        settings_layout.setContentsMargins(20, 18, 20, 20)
        settings_layout.setSpacing(16)

        card_title = QLabel("控制参数")
        card_title.setObjectName("CardTitle")
        settings_layout.addWidget(card_title)

        # 平移速度
        linear_row = QVBoxLayout()
        linear_row.setSpacing(6)
        linear_label = QLabel("平移速度")
        linear_label.setObjectName("FieldLabel")
        self.linear_speed_spin = QDoubleSpinBox()
        self.linear_speed_spin.setRange(MIN_LINEAR_SPEED_M_S, MAX_LINEAR_SPEED_M_S)
        self.linear_speed_spin.setDecimals(3)
        self.linear_speed_spin.setSingleStep(LINEAR_SPEED_STEP_M_S)
        self.linear_speed_spin.setValue(DEFAULT_LINEAR_SPEED_M_S)
        self.linear_speed_spin.setSuffix(" m/s")
        self.linear_speed_spin.valueChanged.connect(self._speed_changed)
        self.linear_speed_spin.editingFinished.connect(self.setFocus)
        linear_row.addWidget(linear_label)
        linear_row.addWidget(self.linear_speed_spin)
        settings_layout.addLayout(linear_row)

        # 旋转速度
        angular_row = QVBoxLayout()
        angular_row.setSpacing(6)
        angular_label = QLabel("旋转速度")
        angular_label.setObjectName("FieldLabel")
        self.angular_speed_spin = QDoubleSpinBox()
        self.angular_speed_spin.setRange(MIN_ANGULAR_SPEED_RAD_S, MAX_ANGULAR_SPEED_RAD_S)
        self.angular_speed_spin.setDecimals(3)
        self.angular_speed_spin.setSingleStep(ANGULAR_SPEED_STEP_RAD_S)
        self.angular_speed_spin.setValue(DEFAULT_ANGULAR_SPEED_RAD_S)
        self.angular_speed_spin.setSuffix(" rad/s")
        self.angular_speed_spin.valueChanged.connect(self._speed_changed)
        self.angular_speed_spin.editingFinished.connect(self.setFocus)
        angular_row.addWidget(angular_label)
        angular_row.addWidget(self.angular_speed_spin)
        settings_layout.addLayout(angular_row)

        # 键盘开关
        self.keyboard_check = QCheckBox("键盘控制")
        self.keyboard_check.setObjectName("SwitchCheck")
        self.keyboard_check.setChecked(True)
        self.keyboard_check.toggled.connect(self._keyboard_toggled)
        settings_layout.addWidget(self.keyboard_check)

        # 分隔线
        settings_layout.addWidget(self._divider())

        # 连接状态
        conn_section = QVBoxLayout()
        conn_section.setSpacing(8)
        conn_label = QLabel("连接状态")
        conn_label.setObjectName("FieldLabel")
        conn_section.addWidget(conn_label)

        # 状态显示
        conn_status_row = QHBoxLayout()
        conn_status_row.setSpacing(10)
        self.debug_status_dot = QFrame()
        self.debug_status_dot.setObjectName("DebugStatusDot")
        self.debug_status_dot.setProperty("connected", False)
        self.debug_status_dot.setFixedSize(10, 10)
        self.debug_status_text = QLabel("未连接")
        self.debug_status_text.setObjectName("DebugStatusText")
        conn_status_row.addWidget(self.debug_status_dot)
        conn_status_row.addWidget(self.debug_status_text)
        conn_status_row.addStretch(1)
        conn_section.addLayout(conn_status_row)

        # 连接/断开按钮
        self.debug_connect_btn = QPushButton("连接")
        self.debug_connect_btn.setObjectName("PrimaryButton")
        self.debug_connect_btn.setMinimumHeight(42)
        self.debug_connect_btn.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_DialogApplyButton)
        )
        self.debug_connect_btn.clicked.connect(self._toggle_connection)
        conn_section.addWidget(self.debug_connect_btn)

        settings_layout.addLayout(conn_section)

        settings_layout.addStretch(1)

        # 停止按钮
        self.stop_button = QPushButton("停止")
        self.stop_button.setObjectName("StopButton")
        self.stop_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_MediaStop)
        )
        self.stop_button.setMinimumHeight(46)
        self.stop_button.clicked.connect(self._stop_motion)
        settings_layout.addWidget(self.stop_button)

        page_layout.addWidget(settings_group)

        # 右侧：当前指令 + 按键说明
        right_panel = QVBoxLayout()
        right_panel.setSpacing(16)

        # 当前指令卡片
        cmd_card = QFrame()
        cmd_card.setObjectName("CardPanel")
        cmd_layout = QVBoxLayout(cmd_card)
        cmd_layout.setContentsMargins(20, 18, 20, 20)
        cmd_layout.setSpacing(14)

        cmd_title = QLabel("当前指令")
        cmd_title.setObjectName("CardTitle")
        cmd_layout.addWidget(cmd_title)

        # 三个数值
        self.vx_label = QLabel()
        self.vy_label = QLabel()
        self.wz_label = QLabel()
        for label in (self.vx_label, self.vy_label, self.wz_label):
            label.setObjectName("CommandValueLarge")
            label.setMinimumHeight(36)
            cmd_layout.addWidget(label)

        right_panel.addWidget(cmd_card)

        # 按键说明卡片
        key_card = QFrame()
        key_card.setObjectName("CardPanel")
        key_layout = QVBoxLayout(key_card)
        key_layout.setContentsMargins(20, 18, 20, 20)
        key_layout.setSpacing(12)

        key_title = QLabel("键盘控制")
        key_title.setObjectName("CardTitle")
        key_layout.addWidget(key_title)

        # 按键网格
        key_grid = QVBoxLayout()
        key_grid.setSpacing(8)

        key_rows = [
            ("W / S", "前进 / 后退"),
            ("A / D", "左移 / 右移"),
            ("Q / E", "逆时针 / 顺时针"),
        ]
        for key, desc in key_rows:
            row = QHBoxLayout()
            row.setSpacing(12)
            key_label = QLabel(key)
            key_label.setObjectName("KeyBadge")
            key_label.setMinimumWidth(70)
            key_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            desc_label = QLabel(desc)
            desc_label.setObjectName("Subtle")
            row.addWidget(key_label)
            row.addWidget(desc_label)
            row.addStretch(1)
            key_grid.addLayout(row)

        key_layout.addLayout(key_grid)
        right_panel.addWidget(key_card)
        right_panel.addStretch(1)

        page_layout.addLayout(right_panel, 1)

        self._stack.addWidget(page)

    def _build_serial_page(self) -> None:
        """串口页：串口配置 + 波特率 + 日志"""
        page = QWidget()
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.setSpacing(16)

        # 顶部：串口配置行
        config_row = QHBoxLayout()
        config_row.setSpacing(12)

        # 串口
        port_label = QLabel("串口")
        port_label.setObjectName("SectionLabel")
        port_label.setMinimumWidth(30)
        config_row.addWidget(port_label)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        self.port_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        config_row.addWidget(self.port_combo, 2)

        # 波特率
        baud_label = QLabel("波特率")
        baud_label.setObjectName("SectionLabel")
        baud_label.setMinimumWidth(40)
        config_row.addWidget(baud_label)
        self.baud_combo = QComboBox()
        self.baud_combo.setObjectName("BaudCombo")
        self.baud_combo.setMinimumWidth(120)
        self.baud_combo.setMaximumWidth(160)
        for baud in COMMON_BAUDRATES:
            self.baud_combo.addItem(f"{baud:,}", baud)
        default_idx = self.baud_combo.findData(DEFAULT_BAUDRATE)
        if default_idx >= 0:
            self.baud_combo.setCurrentIndex(default_idx)
        self.baud_combo.currentIndexChanged.connect(self._on_baudrate_changed)
        config_row.addWidget(self.baud_combo)

        config_row.addStretch(1)

        self.refresh_button = QPushButton("刷新")
        self.refresh_button.setObjectName("GhostButton")
        self.refresh_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_BrowserReload)
        )
        self.refresh_button.clicked.connect(self.scan_ports)
        config_row.addWidget(self.refresh_button)

        self.connect_button = QPushButton("连接")
        self.connect_button.setObjectName("PrimaryButton")
        self.connect_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_DialogApplyButton)
        )
        self.connect_button.clicked.connect(self._toggle_connection)
        config_row.addWidget(self.connect_button)

        page_layout.addLayout(config_row)

        # 日志面板
        log_group = QFrame()
        log_group.setObjectName("CardPanel")
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(20, 18, 20, 20)
        log_layout.setSpacing(12)

        # 日志标题栏
        log_header = QHBoxLayout()
        log_header.setSpacing(8)

        log_title = QLabel("串口日志")
        log_title.setObjectName("CardTitle")
        log_header.addWidget(log_title)
        log_header.addStretch(1)

        # 统计
        self.tx_count_label = QLabel("TX 0")
        self.rx_count_label = QLabel("RX 0")
        self.tx_count_label.setObjectName("MetricBadge")
        self.rx_count_label.setObjectName("MetricBadge")
        log_header.addWidget(self.tx_count_label)
        log_header.addWidget(self.rx_count_label)

        # HEX/ASCII 切换
        self.format_btn = QPushButton("HEX")
        self.format_btn.setObjectName("FormatChip")
        self.format_btn.setCheckable(True)
        self.format_btn.setChecked(True)
        self.format_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.format_btn.clicked.connect(self._on_format_toggled)
        log_header.addWidget(self.format_btn)

        # TX/RX 过滤
        self.filter_tx_btn = QPushButton("TX")
        self.filter_tx_btn.setObjectName("FilterChipTX")
        self.filter_tx_btn.setCheckable(True)
        self.filter_tx_btn.setChecked(True)
        self.filter_tx_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.filter_tx_btn.clicked.connect(self._on_filter_tx_toggled)
        log_header.addWidget(self.filter_tx_btn)

        self.filter_rx_btn = QPushButton("RX")
        self.filter_rx_btn.setObjectName("FilterChipRX")
        self.filter_rx_btn.setCheckable(True)
        self.filter_rx_btn.setChecked(True)
        self.filter_rx_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.filter_rx_btn.clicked.connect(self._on_filter_rx_toggled)
        log_header.addWidget(self.filter_rx_btn)

        # 自动滚动
        self.auto_scroll_check = QCheckBox("自动滚动")
        self.auto_scroll_check.setObjectName("SwitchCheck")
        self.auto_scroll_check.setChecked(True)
        self.auto_scroll_check.toggled.connect(self._on_auto_scroll_toggled)
        log_header.addWidget(self.auto_scroll_check)

        # 清空
        self.clear_log_button = QPushButton("清空")
        self.clear_log_button.setObjectName("GhostButton")
        self.clear_log_button.clicked.connect(self._clear_log)
        log_header.addWidget(self.clear_log_button)

        log_layout.addLayout(log_header)

        # 日志文本区
        self.log_text = QPlainTextEdit()
        self.log_text.setObjectName("LogConsole")
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(MAX_LOG_LINES)
        self.log_text.setFont(QFont("Menlo", 11))
        self.log_text.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        log_layout.addWidget(self.log_text, 1)

        page_layout.addWidget(log_group, 1)

        self._stack.addWidget(page)

    def _build_location_page(self) -> None:
        """定位页：场地照片 + 坐标交互"""
        page = QWidget()
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.setSpacing(16)

        # 顶部工具栏
        toolbar = QFrame()
        toolbar.setObjectName("CardPanel")
        tool_layout = QHBoxLayout(toolbar)
        tool_layout.setContentsMargins(16, 12, 16, 12)
        tool_layout.setSpacing(12)

        # 场地尺寸
        size_label = QLabel("场地尺寸")
        size_label.setObjectName("FieldLabel")
        tool_layout.addWidget(size_label)

        self.field_width_spin = QDoubleSpinBox()
        self.field_width_spin.setRange(0.1, 100.0)
        self.field_width_spin.setDecimals(2)
        self.field_width_spin.setSingleStep(0.1)
        self.field_width_spin.setValue(self._field_width_m)
        self.field_width_spin.setSuffix(" m 宽")
        self.field_width_spin.valueChanged.connect(self._on_field_size_changed)
        tool_layout.addWidget(self.field_width_spin)

        self.field_height_spin = QDoubleSpinBox()
        self.field_height_spin.setRange(0.1, 100.0)
        self.field_height_spin.setDecimals(2)
        self.field_height_spin.setSingleStep(0.1)
        self.field_height_spin.setValue(self._field_height_m)
        self.field_height_spin.setSuffix(" m 高")
        self.field_height_spin.valueChanged.connect(self._on_field_size_changed)
        tool_layout.addWidget(self.field_height_spin)

        tool_layout.addStretch(1)

        # 上传照片
        self.upload_btn = QPushButton("上传场地照片")
        self.upload_btn.setObjectName("GhostButton")
        self.upload_btn.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_DirOpenIcon)
        )
        self.upload_btn.clicked.connect(self._upload_field_image)
        tool_layout.addWidget(self.upload_btn)

        # 设置原点
        self.origin_btn = QPushButton("标记原点")
        self.origin_btn.setObjectName("PrimaryButton")
        self.origin_btn.clicked.connect(self._toggle_set_origin)
        tool_layout.addWidget(self.origin_btn)

        page_layout.addWidget(toolbar)

        # 场地地图
        map_container = QFrame()
        map_container.setObjectName("CardPanel")
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(12, 12, 12, 12)
        map_layout.setSpacing(0)

        self.field_map = FieldMapView()
        self.field_map.setMinimumHeight(400)
        self.field_map.coordinate_changed.connect(self._on_mouse_coordinate)
        self.field_map.origin_set.connect(self._on_origin_set)
        map_layout.addWidget(self.field_map, 1)

        page_layout.addWidget(map_container, 1)

        # 底部坐标栏
        coord_bar = QFrame()
        coord_bar.setObjectName("CommandBar")
        coord_layout = QHBoxLayout(coord_bar)
        coord_layout.setContentsMargins(20, 12, 20, 12)
        coord_layout.setSpacing(24)

        coord_title = QLabel("鼠标坐标")
        coord_title.setObjectName("SectionLabel")
        coord_layout.addWidget(coord_title)

        self.mouse_x_label = QLabel("X: 0.000 m")
        self.mouse_y_label = QLabel("Y: 0.000 m")
        for label in (self.mouse_x_label, self.mouse_y_label):
            label.setObjectName("CoordValue")
            label.setMinimumWidth(130)
            coord_layout.addWidget(label)

        coord_layout.addStretch(1)

        origin_title = QLabel("原点")
        origin_title.setObjectName("SectionLabel")
        coord_layout.addWidget(origin_title)

        self.origin_x_label = QLabel("X: 0.000 m")
        self.origin_y_label = QLabel("Y: 0.000 m")
        for label in (self.origin_x_label, self.origin_y_label):
            label.setObjectName("OriginValue")
            label.setMinimumWidth(130)
            coord_layout.addWidget(label)

        page_layout.addWidget(coord_bar)

        self._stack.addWidget(page)

    @staticmethod
    def _divider() -> QFrame:
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setObjectName("Divider")
        return divider

    def _switch_tab(self, index: int) -> None:
        for i, btn in enumerate(self._tab_buttons):
            btn.set_active(i == index)
        self._stack.setCurrentIndex(index)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            /* ========== 基础重置 ========== */
            QWidget {
                color: #1d1d1f;
                font-family: -apple-system, BlinkMacSystemFont, "SF Pro Text",
                             "Helvetica Neue", "PingFang SC", "Microsoft YaHei UI",
                             sans-serif;
                font-size: 13px;
                selection-background-color: #007aff;
                selection-color: #ffffff;
            }

            /* ========== 主背景 ========== */
            QWidget#Central {
                background: #f5f5f7;
            }

            /* ========== 标题 ========== */
            QLabel#Title {
                font-size: 26px;
                font-weight: 700;
                color: #1d1d1f;
                letter-spacing: -0.5px;
            }
            QLabel#Subtle {
                color: #86868b;
                font-size: 12px;
            }
            QLabel#CardTitle {
                font-size: 15px;
                font-weight: 600;
                color: #1d1d1f;
                letter-spacing: -0.2px;
            }
            QLabel#FieldLabel {
                font-size: 12px;
                font-weight: 500;
                color: #86868b;
                padding-left: 2px;
            }
            QLabel#SectionLabel {
                font-size: 12px;
                font-weight: 600;
                color: #86868b;
                text-transform: uppercase;
                letter-spacing: 0.5px;
            }
            QLabel#DebugStatusText {
                font-size: 14px;
                font-weight: 600;
                color: #86868b;
            }
            QLabel#KeyBadge {
                background: #f2f2f7;
                color: #1d1d1f;
                border-radius: 6px;
                padding: 6px 10px;
                font-size: 12px;
                font-weight: 600;
                font-family: "SF Mono", Menlo, Consolas, monospace;
            }

            /* ========== 顶部标签栏 ========== */
            QFrame#TabBar {
                background: #e8e8ed;
                border-radius: 9px;
                padding: 0;
            }
            QFrame#TabButton {
                background: transparent;
                border-radius: 6px;
                margin: 0;
            }
            QFrame#TabButton:hover {
                background: rgba(0, 0, 0, 0.05);
            }
            QFrame#TabButton[active="true"] {
                background: #ffffff;
            }
            QFrame#TabDot {
                background: #aeaeb2;
                border-radius: 3px;
            }
            QFrame#TabButton[active="true"] QFrame#TabDot {
                background: #007aff;
            }
            QFrame#TabButton:hover QFrame#TabDot {
                background: #86868b;
            }
            QLabel#TabButtonText {
                color: #86868b;
                font-size: 13px;
                font-weight: 500;
            }
            QFrame#TabButton[active="true"] QLabel#TabButtonText {
                color: #1d1d1f;
                font-weight: 600;
            }
            QFrame#TabButton:hover QLabel#TabButtonText {
                color: #1d1d1f;
            }

            /* ========== 状态胶囊 ========== */
            QFrame#StatusPill {
                background: #ffffff;
                border: 1px solid #e5e5ea;
                border-radius: 16px;
                padding: 0px;
            }
            QFrame#StatusPill[connected="true"] {
                background: #e8f8ed;
                border: 1px solid #b8e6c5;
            }
            QLabel#StatusText {
                font-weight: 600;
                font-size: 12px;
                color: #86868b;
            }
            QFrame#StatusDot {
                background: #aeaeb2;
                border-radius: 4px;
            }
            QFrame#StatusDot[connected="true"] {
                background: #34c759;
            }
            QFrame#DebugStatusDot {
                background: #aeaeb2;
                border-radius: 5px;
            }
            QFrame#DebugStatusDot[connected="true"] {
                background: #34c759;
            }

            /* ========== 卡片面板 ========== */
            QFrame#CardPanel, QFrame#CommandBar {
                background: #ffffff;
                border: 1px solid #e5e5ea;
                border-radius: 12px;
            }

            /* ========== 下拉框 & 数字输入 ========== */
            QComboBox, QDoubleSpinBox {
                min-height: 36px;
                padding: 0 12px;
                background: #f2f2f7;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                color: #1d1d1f;
                font-size: 13px;
            }
            QComboBox:hover, QDoubleSpinBox:hover {
                background: #e8e8ed;
            }
            QComboBox:focus, QDoubleSpinBox:focus {
                border: 2px solid #007aff;
                background: #ffffff;
                padding: 0 11px;
            }
            QComboBox::drop-down {
                border: none;
                width: 24px;
            }
            QComboBox::down-arrow {
                image: none;
                border: none;
                width: 0;
                height: 0;
            }
            QComboBox QAbstractItemView {
                background: #ffffff;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                padding: 4px;
                selection-background-color: #f2f2f7;
                selection-color: #1d1d1f;
                outline: none;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 20px;
                background: transparent;
                border: none;
            }
            QDoubleSpinBox::up-arrow, QDoubleSpinBox::down-arrow {
                width: 0;
                height: 0;
            }

            QComboBox#BaudCombo {
                font-weight: 600;
            }

            /* ========== 按钮 ========== */
            QPushButton {
                min-height: 36px;
                padding: 0 16px;
                background: #f2f2f7;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                font-weight: 500;
                font-size: 13px;
                color: #1d1d1f;
            }
            QPushButton:hover {
                background: #e8e8ed;
            }
            QPushButton:pressed {
                background: #dddde3;
            }
            QPushButton:disabled {
                color: #aeaeb2;
                background: #f2f2f7;
                border-color: #e5e5ea;
            }

            /* 幽灵按钮 */
            QPushButton#GhostButton {
                background: transparent;
                border: 1px solid #d1d1d6;
                color: #007aff;
                min-height: 30px;
                padding: 0 12px;
                font-size: 12px;
            }
            QPushButton#GhostButton:hover {
                background: #f0f7ff;
                border-color: #007aff;
            }
            QPushButton#GhostButton:pressed {
                background: #e0efff;
            }

            /* 主按钮 */
            QPushButton#PrimaryButton {
                color: #ffffff;
                background: #007aff;
                border: 1px solid #007aff;
                font-weight: 600;
                font-size: 14px;
            }
            QPushButton#PrimaryButton:hover {
                background: #0a84ff;
                border-color: #0a84ff;
            }
            QPushButton#PrimaryButton:pressed {
                background: #0066cc;
                border-color: #0066cc;
            }
            QPushButton#PrimaryButton:disabled {
                background: #a8c8f0;
                border-color: #a8c8f0;
                color: #ffffff;
            }

            /* 停止按钮 */
            QPushButton#StopButton {
                color: #ffffff;
                background: #ff3b30;
                border: 1px solid #ff3b30;
                font-weight: 600;
                font-size: 14px;
                border-radius: 10px;
            }
            QPushButton#StopButton:hover {
                background: #ff453a;
                border-color: #ff453a;
            }
            QPushButton#StopButton:pressed {
                background: #d70015;
                border-color: #d70015;
            }

            /* ========== 格式切换芯片按钮 ========== */
            QPushButton#FormatChip {
                min-height: 30px;
                min-width: 56px;
                padding: 0 14px;
                background: #e5e5ea;
                border: 1px solid #d1d1d6;
                border-radius: 15px;
                font-size: 12px;
                font-weight: 700;
                color: #86868b;
            }
            QPushButton#FormatChip:checked {
                background: #1d1d1f;
                border: 1px solid #1d1d1f;
                color: #ffffff;
            }
            QPushButton#FormatChip:checked:hover {
                background: #2c2c2e;
                border-color: #2c2c2e;
            }
            QPushButton#FormatChip:hover:!checked {
                background: #d1d1d6;
                color: #1d1d1f;
            }

            /* ========== TX/RX 过滤芯片按钮 ========== */
            QPushButton#FilterChipTX {
                min-height: 30px;
                min-width: 52px;
                padding: 0 16px;
                background: #e5e5ea;
                border: 1px solid #d1d1d6;
                border-radius: 15px;
                font-size: 12px;
                font-weight: 700;
                color: #86868b;
            }
            QPushButton#FilterChipTX:checked {
                background: #007aff;
                border: 1px solid #007aff;
                color: #ffffff;
            }
            QPushButton#FilterChipTX:checked:hover {
                background: #0a84ff;
                border-color: #0a84ff;
            }
            QPushButton#FilterChipTX:hover:!checked {
                background: #d1d1d6;
                color: #1d1d1f;
            }

            QPushButton#FilterChipRX {
                min-height: 30px;
                min-width: 52px;
                padding: 0 16px;
                background: #e5e5ea;
                border: 1px solid #d1d1d6;
                border-radius: 15px;
                font-size: 12px;
                font-weight: 700;
                color: #86868b;
            }
            QPushButton#FilterChipRX:checked {
                background: #34c759;
                border: 1px solid #34c759;
                color: #ffffff;
            }
            QPushButton#FilterChipRX:checked:hover {
                background: #30d158;
                border-color: #30d158;
            }
            QPushButton#FilterChipRX:hover:!checked {
                background: #d1d1d6;
                color: #1d1d1f;
            }

            /* ========== 日志控制台 ========== */
            QPlainTextEdit#LogConsole {
                background: #1d1d1f;
                color: #f5f5f7;
                border: 1px solid #e5e5ea;
                border-radius: 8px;
                padding: 12px;
                font-family: "SF Mono", Menlo, Consolas, monospace;
                font-size: 12px;
                selection-background-color: #007aff;
            }

            /* ========== 指令数值 ========== */
            QLabel#CommandValueLarge {
                color: #007aff;
                font-family: "SF Mono", "Menlo", "Consolas", monospace;
                font-size: 18px;
                font-weight: 700;
                letter-spacing: 0.3px;
                padding: 8px 0;
            }

            /* ========== 坐标数值 ========== */
            QLabel#CoordValue {
                color: #007aff;
                font-family: "SF Mono", "Menlo", "Consolas", monospace;
                font-size: 14px;
                font-weight: 600;
            }
            QLabel#OriginValue {
                color: #ff3b30;
                font-family: "SF Mono", "Menlo", "Consolas", monospace;
                font-size: 14px;
                font-weight: 600;
            }

            /* ========== 指标徽章 ========== */
            QLabel#MetricBadge {
                background: #f2f2f7;
                color: #86868b;
                border-radius: 6px;
                padding: 5px 10px;
                font-size: 12px;
                font-weight: 500;
            }

            /* ========== 开关风格 CheckBox ========== */
            QCheckBox#SwitchCheck {
                spacing: 10px;
                font-size: 12px;
                font-weight: 500;
                color: #86868b;
                padding: 4px 0;
            }
            QCheckBox#SwitchCheck::indicator {
                width: 38px;
                height: 22px;
                border-radius: 11px;
                background: #e5e5ea;
                border: none;
            }
            QCheckBox#SwitchCheck::indicator:checked {
                background: #34c759;
            }
            QCheckBox#SwitchCheck::indicator:unchecked:hover {
                background: #d1d1d6;
            }

            /* ========== 场地地图 ========== */
            QFrame#FieldMapView {
                background: #1d1d1f;
                border-radius: 8px;
            }

            /* ========== 分隔线 ========== */
            QFrame#Divider {
                background: #e5e5ea;
                max-height: 1px;
                border: none;
            }

            /* ========== 滚动条 ========== */
            QScrollBar:vertical {
                background: transparent;
                width: 8px;
                margin: 0;
            }
            QScrollBar::handle:vertical {
                background: #c7c7cc;
                border-radius: 4px;
                min-height: 30px;
            }
            QScrollBar::handle:vertical:hover {
                background: #aeaeb2;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0;
            }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
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
            self.debug_connect_btn.setEnabled(False)
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
        self.debug_connect_btn.setEnabled(True)

    # ── 波特率 ──

    def _on_baudrate_changed(self, index: int) -> None:
        baudrate = self.baud_combo.itemData(index)
        if baudrate is None:
            return
        self._current_baudrate = baudrate
        if not self._connected:
            self._append_log("SYS", f"波特率已设置为 {baudrate:,} bps", "#86868b")

    def _toggle_connection(self) -> None:
        if self._connected:
            self._clear_motion(send_now=False)
            self.disconnect_requested.emit(ZERO_FRAME)
            self._append_log("SYS", "串口已断开", "#ff9500")
            return

        port = self.port_combo.currentData()
        if port is None:
            self._set_status_message("没有可连接的串口", error=True)
            return

        self.connect_button.setEnabled(False)
        self.debug_connect_btn.setEnabled(False)
        self._set_status_message("正在连接…")
        self._append_log("SYS", f"正在连接 {port} @ {self._current_baudrate:,}…", "#ff9500")
        self.connect_requested.emit(port, self._current_baudrate)

    def _on_serial_status(self, connected: bool, detail: str) -> None:
        self._connected = connected
        self.status_label.setStyleSheet("")
        self.status_dot.setProperty("connected", connected)
        self.status_dot.style().unpolish(self.status_dot)
        self.status_dot.style().polish(self.status_dot)

        # 更新状态胶囊
        status_pill = self.status_dot.parent()
        if status_pill:
            status_pill.setProperty("connected", connected)
            status_pill.style().unpolish(status_pill)
            status_pill.style().polish(status_pill)

        self.status_label.setText(detail)
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)

        self.connect_button.setEnabled(connected or self.port_combo.currentData() is not None)
        self.connect_button.setText("断开" if connected else "连接")
        self.debug_connect_btn.setEnabled(connected or self.port_combo.currentData() is not None)
        self.debug_connect_btn.setText("断开" if connected else "连接")

        if connected:
            icon = QStyle.StandardPixmap.SP_DialogCloseButton
            self._append_log("SYS", f"串口已连接：{detail}", "#34c759")
            self.debug_status_text.setText("已连接")
            self.debug_status_text.setStyleSheet("color: #34c759;")
        else:
            icon = QStyle.StandardPixmap.SP_DialogApplyButton
            self.debug_status_text.setText("未连接")
            self.debug_status_text.setStyleSheet("color: #86868b;")

        self.connect_button.setIcon(self.style().standardIcon(icon))
        self.debug_connect_btn.setIcon(self.style().standardIcon(icon))

        # 调试页状态点
        self.debug_status_dot.setProperty("connected", connected)
        self.debug_status_dot.style().unpolish(self.debug_status_dot)
        self.debug_status_dot.style().polish(self.debug_status_dot)

        self._clear_motion(send_now=connected)

    def _on_serial_error(self, message: str) -> None:
        self._set_status_message(message, error=True)
        self._append_log("ERR", message, "#ff3b30")
        if not self._connected:
            self.connect_button.setEnabled(self.port_combo.currentData() is not None)
            self.debug_connect_btn.setEnabled(self.port_combo.currentData() is not None)

    def _set_status_message(self, message: str, error: bool = False) -> None:
        self.status_label.setText(message)
        if error:
            self.status_label.setStyleSheet("color: #ff3b30;")
        else:
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

        self.vx_label.setText(f"Vx  {self._command.vx_mm_s / 1000.0:+.3f} m/s")
        self.vy_label.setText(f"Vy  {self._command.vy_mm_s / 1000.0:+.3f} m/s")
        self.wz_label.setText(f"Wz  {self._command.wz_mrad_s / 1000.0:+.3f} rad/s")

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

    # ── 日志格式切换 ──

    def _on_format_toggled(self, checked: bool) -> None:
        self._show_hex = checked
        self.format_btn.setText("HEX" if checked else "ASCII")
        self._refresh_log_view()

    # ── 日志过滤 ──

    def _on_filter_tx_toggled(self, checked: bool) -> None:
        self._filter_tx = checked
        self._refresh_log_view()

    def _on_filter_rx_toggled(self, checked: bool) -> None:
        self._filter_rx = checked
        self._refresh_log_view()

    def _should_show_tag(self, tag: str) -> bool:
        if tag == "TX":
            return self._filter_tx
        if tag == "RX":
            return self._filter_rx
        return True

    def _refresh_log_view(self) -> None:
        self.log_text.clear()
        for timestamp, tag, data, color in self._log_buffer:
            if self._should_show_tag(tag):
                self._render_log_line(timestamp, tag, data, color)

        if self._auto_scroll:
            cursor = self.log_text.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self.log_text.setTextCursor(cursor)

    # ── 日志相关 ──

    def _format_data(self, data: bytes) -> str:
        if self._show_hex:
            return " ".join(f"{b:02X}" for b in data)
        else:
            result = []
            for b in data:
                if 32 <= b < 127:
                    result.append(chr(b))
                else:
                    result.append(".")
            return "".join(result)

    def _render_log_line(self, timestamp: str, tag: str, data: bytes, color: str) -> None:
        content = self._format_data(data)
        size = len(data)
        html = (
            f'<span style="color:#6b7280;">{timestamp}</span> '
            f'<span style="color:{color};font-weight:600;">[{tag}]</span> '
            f'<span style="color:#9aa0a6;">{size}B</span> '
            f'<span style="color:#f5f5f7;">{content}</span>'
        )
        self.log_text.appendHtml(html)

    def _append_log(self, tag: str, message: str, color: str = "#f5f5f7") -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        data = message.encode("utf-8", errors="replace")

        self._log_buffer.append((timestamp, tag, data, color))
        if len(self._log_buffer) > MAX_LOG_LINES:
            self._log_buffer = self._log_buffer[-MAX_LOG_LINES:]

        if self._should_show_tag(tag):
            html = (
                f'<span style="color:#6b7280;">{timestamp}</span> '
                f'<span style="color:{color};font-weight:600;">[{tag}]</span> '
                f'<span style="color:#f5f5f7;">{message}</span>'
            )
            self.log_text.appendHtml(html)
            if self._auto_scroll:
                cursor = self.log_text.textCursor()
                cursor.movePosition(QTextCursor.MoveOperation.End)
                self.log_text.setTextCursor(cursor)

    def _on_tx_bytes(self, payload: bytes) -> None:
        self._tx_count += 1
        self.tx_count_label.setText(f"TX {self._tx_count}")
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = "#007aff"

        self._log_buffer.append((timestamp, "TX", payload, color))
        if len(self._log_buffer) > MAX_LOG_LINES:
            self._log_buffer = self._log_buffer[-MAX_LOG_LINES:]

        if self._filter_tx:
            self._render_log_line(timestamp, "TX", payload, color)
            if self._auto_scroll:
                cursor = self.log_text.textCursor()
                cursor.movePosition(QTextCursor.MoveOperation.End)
                self.log_text.setTextCursor(cursor)

    def _on_rx_bytes(self, payload: bytes) -> None:
        self._rx_count += len(payload)
        self.rx_count_label.setText(f"RX {self._rx_count}")
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = "#34c759"

        self._log_buffer.append((timestamp, "RX", payload, color))
        if len(self._log_buffer) > MAX_LOG_LINES:
            self._log_buffer = self._log_buffer[-MAX_LOG_LINES:]

        if self._filter_rx:
            self._render_log_line(timestamp, "RX", payload, color)
            if self._auto_scroll:
                cursor = self.log_text.textCursor()
                cursor.movePosition(QTextCursor.MoveOperation.End)
                self.log_text.setTextCursor(cursor)

    def _clear_log(self) -> None:
        self.log_text.clear()
        self._log_buffer.clear()
        self._tx_count = 0
        self._rx_count = 0
        self.tx_count_label.setText("TX 0")
        self.rx_count_label.setText("RX 0")

    def _on_auto_scroll_toggled(self, checked: bool) -> None:
        self._auto_scroll = checked
        if checked:
            cursor = self.log_text.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self.log_text.setTextCursor(cursor)

    # ── 定位页功能 ──

    def _upload_field_image(self) -> None:
        """上传场地照片"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择场地照片",
            "",
            "图片文件 (*.png *.jpg *.jpeg *.bmp *.gif *.webp)",
        )
        if not file_path:
            return

        if self.field_map.set_field_image(file_path):
            self._field_image_path = file_path
            self._save_config()
            self._append_log("SYS", f"场地照片已加载：{os.path.basename(file_path)}", "#86868b")

    def _on_field_size_changed(self) -> None:
        """场地尺寸改变"""
        w = self.field_width_spin.value()
        h = self.field_height_spin.value()
        self._field_width_m = w
        self._field_height_m = h
        self.field_map.set_field_size(w, h)
        self._save_config()

    def _toggle_set_origin(self) -> None:
        """切换设置原点模式"""
        if self.field_map.is_setting_origin():
            self.field_map.cancel_set_origin()
            self.origin_btn.setText("标记原点")
            self.origin_btn.setStyleSheet("")
        else:
            self.field_map.start_set_origin()
            self.origin_btn.setText("取消标记")
            self.origin_btn.setStyleSheet(
                "background: #ff9500; border-color: #ff9500; color: white;"
            )

    def _on_origin_set(self, x_m: float, y_m: float) -> None:
        """原点设置完成"""
        self.origin_btn.setText("标记原点")
        self.origin_btn.setStyleSheet("")
        # 原点坐标应该是(0,0)，这里显示原点在图片上的位置对应的实际坐标
        self.origin_x_label.setText(f"X: {x_m:+.3f} m")
        self.origin_y_label.setText(f"Y: {y_m:+.3f} m")

        # 保存原点像素位置
        origin_px = self.field_map._origin_px
        self._origin_px_x = origin_px.x()
        self._origin_px_y = origin_px.y()
        self._save_config()

    def _on_mouse_coordinate(self, x_m: float, y_m: float) -> None:
        """鼠标坐标更新"""
        self.mouse_x_label.setText(f"X: {x_m:+.3f} m")
        self.mouse_y_label.setText(f"Y: {y_m:+.3f} m")

    # ── 配置持久化 ──

    def _load_config(self) -> None:
        """从文件加载配置"""
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                    config = json.load(f)
                self._field_image_path = config.get("field_image", "")
                self._field_width_m = config.get("field_width_m", 3.0)
                self._field_height_m = config.get("field_height_m", 2.0)
                self._origin_px_x = config.get("origin_px_x", 0.0)
                self._origin_px_y = config.get("origin_px_y", 0.0)
        except Exception:
            pass

    def _save_config(self) -> None:
        """保存配置到文件"""
        try:
            config = {
                "field_image": self._field_image_path,
                "field_width_m": self._field_width_m,
                "field_height_m": self._field_height_m,
                "origin_px_x": self._origin_px_x,
                "origin_px_y": self._origin_px_y,
            }
            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def closeEvent(self, event: QCloseEvent) -> None:
        self._send_timer.stop()
        self._port_timer.stop()
        self._clear_motion(send_now=False)
        QApplication.instance().removeEventFilter(self)

        # 保存配置
        self._save_config()

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