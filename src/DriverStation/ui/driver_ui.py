import os
import math
import logging
import sys
import urllib.request
import json
import time
from dataclasses import dataclass
from pathlib import Path
import numpy as np

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QTabWidget, QLabel, QGridLayout, QGroupBox,
    QPlainTextEdit, QPushButton, QHBoxLayout, QSizePolicy, QSlider,
    QCheckBox, QComboBox, QSpinBox, QDoubleSpinBox, QScrollArea, QFrame,
    QTableWidget, QTableWidgetItem, QHeaderView, QLineEdit, QSplitter,
    QAbstractItemView,
)
from PyQt6.QtCore import Qt, QPointF, QRectF, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF, QImage, QPixmap

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from match_history import MatchHistory, MatchResult, calculate_score
except ImportError:
    try:
        import sys as _sys
        _sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from match_history import MatchHistory, MatchResult, calculate_score
    except ImportError:
        MatchHistory = None
        MatchResult = None
        calculate_score = None

logger = logging.getLogger(__name__)

CAMERA_RECONNECT_MS = 1500
DRIVERSTATION_DIR = Path(__file__).resolve().parents[1]
PROJECT_ROOT = DRIVERSTATION_DIR.parents[1]
FIELD_IMAGE_PATH = DRIVERSTATION_DIR / "field.png"
ROBOT_SIZE_M = 18.0 * 0.0254
DEFAULT_ROBOT_ADDRESSES = [
    "10.10.89.3",
    "10.42.0.85",
    "10.42.0.3",
    "10.42.0.2",
    "127.0.0.1",
    "10.91.75.23",
    "10.222.255.253",
]


@dataclass(frozen=True)
class CameraFeedConfig:
    name: str
    label: str
    stream_url: str


def build_camera_feed_configs():
    base_url = os.environ.get("KSU_CAMERA_BASE_URL", "http://10.10.89.3:8080").rstrip("/")
    return [
        CameraFeedConfig(
            name="front_left",
            label=os.environ.get("KSU_CAMERA_FRONT_LEFT_LABEL", "Front Left"),
            stream_url=os.environ.get(
                "KSU_CAMERA_FRONT_LEFT_STREAM_URL",
                f"{base_url}/front_left/stream.mjpg",
            ),
        ),
        CameraFeedConfig(
            name="front_right",
            label=os.environ.get("KSU_CAMERA_FRONT_RIGHT_LABEL", "Front Right"),
            stream_url=os.environ.get(
                "KSU_CAMERA_FRONT_RIGHT_STREAM_URL",
                f"{base_url}/front_right/stream.mjpg",
            ),
        ),
        CameraFeedConfig(
            name="driver",
            label=os.environ.get("KSU_CAMERA_DRIVER_LABEL", "Driver"),
            stream_url=os.environ.get(
                "KSU_CAMERA_DRIVER_STREAM_URL",
                f"{base_url}/driver/stream.mjpg",
            ),
        ),
    ]


CAMERA_FEEDS = build_camera_feed_configs()


def missing_opencv_message() -> str:
    active_python = Path(sys.executable)
    message = f"Camera unavailable: OpenCV not installed in {active_python}"

    repo_venv_python = PROJECT_ROOT / "venv" / "bin" / "python"
    if repo_venv_python.exists() and repo_venv_python != active_python:
        return (
            f"{message}\n"
            f"Launch the driver station with {repo_venv_python}"
        )

    return message


class FieldWidget(QWidget):
    """Simple 2D field map showing robot position and heading."""
    # Power Flash field: 16ft x 8ft (§2, §8.1)
    FIELD_WIDTH_M = 16 * 0.3048    # 4.877 m
    FIELD_HEIGHT_M = 8 * 0.3048    # 2.438 m

    def __init__(self, parent=None):
        super().__init__(parent)
        self.field_width_m = self.FIELD_WIDTH_M
        self.field_height_m = self.FIELD_HEIGHT_M
        self.robot_x_m = self.field_width_m / 2.0
        self.robot_y_m = self.field_height_m / 2.0
        self.robot_theta_deg = 0.0
        self.expected_x_m = self.robot_x_m
        self.expected_y_m = self.robot_y_m
        self.expected_theta_deg = self.robot_theta_deg
        self.field_background = QPixmap(str(FIELD_IMAGE_PATH))
        self.setMinimumHeight(180)
        self.setStyleSheet("background-color: rgb(15, 20, 25); border: 1px solid rgb(55, 100, 102);")

    def set_field_size(self, width_m, height_m):
        self.field_width_m = max(0.1, float(width_m))
        self.field_height_m = max(0.1, float(height_m))
        self.update()

    def set_pose(self, x_m, y_m, theta_deg):
        self.robot_x_m = max(0.0, min(self.field_width_m, float(x_m)))
        self.robot_y_m = max(0.0, min(self.field_height_m, float(y_m)))
        self.robot_theta_deg = float(theta_deg) % 360.0
        self.update()

    def set_expected_pose(self, x_m, y_m, theta_deg):
        self.expected_x_m = max(0.0, min(self.field_width_m, float(x_m)))
        self.expected_y_m = max(0.0, min(self.field_height_m, float(y_m)))
        self.expected_theta_deg = float(theta_deg) % 360.0
        self.update()

    def _field_to_screen(self, x_m, y_m, draw_rect):
        sx = draw_rect.left() + (x_m / self.field_width_m) * draw_rect.width()
        sy = draw_rect.bottom() - (y_m / self.field_height_m) * draw_rect.height()
        return QPointF(sx, sy)

    def _meters_to_pixels(self, draw_rect):
        pixels_per_meter_x = draw_rect.width() / self.field_width_m
        pixels_per_meter_y = draw_rect.height() / self.field_height_m
        return min(pixels_per_meter_x, pixels_per_meter_y)

    def _robot_polygon(self, x_m, y_m, theta_deg, draw_rect, scale=1.0):
        center = self._field_to_screen(x_m, y_m, draw_rect)
        pixels_per_meter = self._meters_to_pixels(draw_rect)
        half_size_px = max(4.0, (ROBOT_SIZE_M * pixels_per_meter * scale) / 2.0)

        local_corners = [
            (-half_size_px, -half_size_px),
            (half_size_px, -half_size_px),
            (half_size_px, half_size_px),
            (-half_size_px, half_size_px),
        ]

        heading_rad = math.radians(theta_deg)
        cos_theta = math.cos(heading_rad)
        sin_theta = math.sin(heading_rad)
        corners = []
        for local_x, local_y in local_corners:
            rotated_x = (local_x * cos_theta) - (local_y * sin_theta)
            rotated_y = (local_x * sin_theta) + (local_y * cos_theta)
            corners.append(
                QPointF(
                    center.x() + rotated_x,
                    center.y() - rotated_y,
                )
            )

        return center, half_size_px, QPolygonF(corners)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        margin = 12
        draw_rect = QRectF(
            margin,
            margin,
            max(10, self.width() - 2 * margin),
            max(10, self.height() - 2 * margin),
        )

        if not self.field_background.isNull():
            scaled_background = self.field_background.scaled(
                int(draw_rect.width()),
                int(draw_rect.height()),
                Qt.AspectRatioMode.IgnoreAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
            painter.drawPixmap(int(draw_rect.left()), int(draw_rect.top()), scaled_background)
            painter.setPen(QPen(QColor(235, 235, 235, 180), 2))
            painter.drawRect(draw_rect)
        else:
            painter.fillRect(draw_rect, QColor(30, 45, 55))
            painter.setPen(QPen(QColor(95, 140, 150), 2))
            painter.drawRect(draw_rect)

            painter.setPen(QPen(QColor(70, 95, 110), 1, Qt.PenStyle.DashLine))
            for i in range(1, 6):
                x = draw_rect.left() + (draw_rect.width() * i / 6.0)
                y = draw_rect.top() + (draw_rect.height() * i / 6.0)
                painter.drawLine(QPointF(x, draw_rect.top()), QPointF(x, draw_rect.bottom()))
                painter.drawLine(QPointF(draw_rect.left(), y), QPointF(draw_rect.right(), y))

        center, half_size_px, robot_polygon = self._robot_polygon(
            self.robot_x_m,
            self.robot_y_m,
            self.robot_theta_deg,
            draw_rect,
        )
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(230, 120, 40)))
        painter.drawPolygon(robot_polygon)

        heading_rad = math.radians(self.robot_theta_deg)
        heading_length = half_size_px * 1.5
        heading_tip = QPointF(
            center.x() + heading_length * math.cos(heading_rad),
            center.y() - heading_length * math.sin(heading_rad),
        )
        painter.setPen(QPen(QColor(255, 220, 120), 3))
        painter.drawLine(center, heading_tip)

        expected_center = self._field_to_screen(self.expected_x_m, self.expected_y_m, draw_rect)
        painter.setPen(QPen(QColor(125, 235, 240), 2, Qt.PenStyle.DashLine))
        painter.drawLine(center, expected_center)
        expected_center, expected_half_size_px, expected_polygon = self._robot_polygon(
            self.expected_x_m,
            self.expected_y_m,
            self.expected_theta_deg,
            draw_rect,
            scale=0.9,
        )
        painter.setPen(QPen(QColor(125, 235, 240), 2, Qt.PenStyle.DashLine))
        painter.setBrush(QBrush(QColor(60, 180, 200, 80)))
        painter.drawPolygon(expected_polygon)

        expected_heading_rad = math.radians(self.expected_theta_deg)
        expected_tip = QPointF(
            expected_center.x() + expected_half_size_px * 1.3 * math.cos(expected_heading_rad),
            expected_center.y() - expected_half_size_px * 1.3 * math.sin(expected_heading_rad),
        )
        painter.drawLine(expected_center, expected_tip)

        painter.setPen(QPen(QColor(235, 235, 235), 1))
        background_label = "custom image" if not self.field_background.isNull() else "grid"
        painter.drawText(8, 16, f"Field View ({background_label}; orange=current, cyan=expected)")


class CameraStreamThread(QThread):
    frame_ready = pyqtSignal(str, QImage)
    status_changed = pyqtSignal(str, str)

    def __init__(self, camera_name, stream_url, parent=None):
        super().__init__(parent)
        self.camera_name = camera_name
        self.stream_url = stream_url
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        if cv2 is None:
            self.status_changed.emit(self.camera_name, missing_opencv_message())
            return

        while self._running:
            self.status_changed.emit(self.camera_name, f"Connecting: {self.stream_url}")

            try:
                stream = urllib.request.urlopen(self.stream_url, timeout=5)
                self.status_changed.emit(self.camera_name, "Camera connected")
                buffer = b""

                while self._running:
                    chunk = stream.read(4096)
                    if not chunk:
                        self.status_changed.emit(self.camera_name, "Camera stream dropped (reconnecting...)")
                        break

                    buffer += chunk

                    start = buffer.find(b"\xff\xd8")
                    end = buffer.find(b"\xff\xd9")

                    if start != -1 and end != -1 and end > start:
                        jpg = buffer[start:end + 2]
                        buffer = buffer[end + 2:]

                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if frame_bgr is None:
                            continue

                        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                        h, w, c = frame_rgb.shape
                        image = QImage(
                            frame_rgb.data,
                            w,
                            h,
                            c * w,
                            QImage.Format.Format_RGB888
                        ).copy()
                        self.frame_ready.emit(self.camera_name, image)

            except Exception:
                self.status_changed.emit(self.camera_name, "Camera disconnected (retrying...)")

            if self._running:
                self.msleep(CAMERA_RECONNECT_MS)


class CameraView(QLabel):
    def __init__(self, placeholder_text="Camera feed unavailable", parent=None):
        super().__init__(parent)
        self._last_image = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(160, 120)
        self.setText(placeholder_text)
        self.setStyleSheet("background-color: rgb(12, 12, 12); border: 1px solid rgb(55, 100, 102);")

    def set_frame(self, image):
        self._last_image = image
        self._render_latest()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_latest()

    def _render_latest(self):
        if self._last_image is None:
            return
        pixmap = QPixmap.fromImage(self._last_image).scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.setPixmap(pixmap)

    def has_frame(self):
        return self._last_image is not None


class DriverUIHelpers:
    """UI-only helpers that keep window layout and visual widgets out of driver logic."""

    def setup_tabs(self):
        if not hasattr(self, "gridLayout") or not hasattr(self, "frame"):
            return

        self.main_tabs = QTabWidget(self.centralwidget)
        self.main_tabs.setObjectName("main_tabs")
        self._camera_latest_frames = {}
        self._camera_status_cache = {}
        self._camera_render_timer = QTimer(self.main_tabs)
        self._camera_render_timer.setInterval(100)
        self._camera_render_timer.timeout.connect(self._flush_camera_frames)

        self.gridLayout.removeWidget(self.frame)
        self.gridLayout.addWidget(self.main_tabs, 0, 0)
        self.main_tabs.setStyleSheet(self._panel_style())
        self.main_tabs.addTab(self.frame, "Main")
        self.setup_main_page_layout()

        self.camera_tab = QWidget()
        camera_layout = QGridLayout(self.camera_tab)
        camera_layout.setContentsMargins(10, 10, 10, 10)
        camera_layout.setHorizontalSpacing(10)
        camera_layout.setVerticalSpacing(10)
        self.camera_tab_status_labels = {}
        self.camera_tab_views = {}

        for index, feed in enumerate(CAMERA_FEEDS):
            status_label = QLabel(f"{feed.label}: waiting for stream...")
            status_label.setStyleSheet("color: rgb(200, 210, 215);")
            view = CameraView(f"{feed.label}\nWaiting for stream...")
            self.camera_tab_status_labels[feed.name] = status_label
            self.camera_tab_views[feed.name] = view

            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.addWidget(status_label)
            cell_layout.addWidget(view, 1)

            camera_layout.addWidget(cell, index // 2, index % 2)

        self.main_tabs.addTab(self.camera_tab, "Camera")

        self.setup_settings_tab()
        self.setup_network_tab()
        self.setup_odometry_tab()
        self.setup_mechanism_tab()
        self.setup_diagnostics_tab()
        self.setup_score_tab()
        self.setup_match_history_tab()
        self.main_tabs.currentChanged.connect(self._handle_tab_changed)
        self._camera_render_timer.start()

    def _panel_style(self):
        return (
            "QGroupBox {"
            "background-color: rgb(18, 24, 30);"
            "border: 1px solid rgb(55, 100, 102);"
            "border-radius: 8px;"
            "margin-top: 18px;"
            "padding-top: 4px;"
            "}"
            "QGroupBox::title {"
            "subcontrol-origin: margin;"
            "subcontrol-position: top left;"
            "padding: 2px 8px;"
            "color: rgb(130, 190, 200);"
            "font-weight: bold;"
            "font-size: 12px;"
            "}"
            "QWidget { background-color: transparent; color: rgb(220, 228, 234); }"
            "QLabel { color: rgb(220, 228, 234); background: transparent; border: none; }"
            "QCheckBox { color: rgb(220, 228, 234); background: transparent; border: none; }"
            "QPushButton {"
            "background-color: rgb(37, 54, 64);"
            "color: rgb(240, 245, 247);"
            "border: 1px solid rgb(77, 122, 126);"
            "border-radius: 6px;"
            "padding: 5px 10px;"
            "}"
            "QPushButton:hover { background-color: rgb(49, 70, 82); }"
            "QPushButton:pressed { background-color: rgb(28, 42, 52); }"
            "QComboBox {"
            "background-color: rgb(37, 54, 64);"
            "color: rgb(240, 245, 247);"
            "border: 1px solid rgb(77, 122, 126);"
            "border-radius: 6px;"
            "padding: 4px 8px;"
            "}"
            "QComboBox::drop-down { border: none; }"
            "QComboBox QAbstractItemView {"
            "background-color: rgb(30, 42, 52);"
            "color: rgb(220, 228, 234);"
            "selection-background-color: rgb(55, 85, 100);"
            "}"
            "QPlainTextEdit {"
            "background-color: rgb(13, 18, 23);"
            "color: rgb(210, 220, 225);"
            "border: 1px solid rgb(45, 80, 85);"
            "border-radius: 4px;"
            "}"
            "QSlider::groove:horizontal {"
            "background: rgb(40, 55, 65);"
            "height: 6px; border-radius: 3px;"
            "}"
            "QSlider::handle:horizontal {"
            "background: rgb(77, 140, 150);"
            "width: 14px; height: 14px;"
            "border-radius: 7px; margin: -4px 0;"
            "}"
            "QTabWidget { background: transparent; border: none; }"
            "QTabWidget::pane {"
            "border: 1px solid rgb(55, 100, 102);"
            "background-color: rgb(13, 18, 23);"
            "border-radius: 6px;"
            "top: -1px;"
            "}"
            "QTabBar::tab {"
            "background-color: rgb(22, 30, 38);"
            "color: rgb(180, 195, 205);"
            "border: 1px solid rgb(45, 80, 85);"
            "padding: 5px 14px;"
            "margin-right: 3px;"
            "border-top-left-radius: 6px;"
            "border-top-right-radius: 6px;"
            "}"
            "QTabBar::tab:selected {"
            "background-color: rgb(37, 54, 64);"
            "color: rgb(240, 245, 247);"
            "border-bottom-color: rgb(13, 18, 23);"
            "}"
            "QTabBar::tab:hover { background-color: rgb(32, 46, 58); }"
        )

    def setup_main_page_layout(self):
        self.frame.setStyleSheet("background-color: rgb(11, 16, 20);")
        existing_layout = self.frame.layout()
        if existing_layout is None:
            root_layout = QHBoxLayout(self.frame)
        else:
            root_layout = existing_layout
            while root_layout.count():
                item = root_layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.setParent(None)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(10)

        # Hide the original geometry-based containers so they do not sit on top of
        # the rebuilt dashboard and steal mouse events.
        for name in ("frame_connection", "frame_robot_control", "frame_connection_2",
                      "frame_connection_3", "frame_keyboard"):
            panel = getattr(self, name, None)
            if panel is not None:
                panel.hide()

        # ── LEFT SIDEBAR ──────────────────────────────────────────────────────
        left_sidebar = QWidget(self.frame)
        left_sidebar.setFixedWidth(260)
        left_sidebar.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        left_layout = QVBoxLayout(left_sidebar)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)

        # Connection info
        conn_group = QGroupBox("Connection")
        conn_layout = QVBoxLayout(conn_group)
        conn_layout.setSpacing(4)
        for widget in (self.status_label, self.address_label, self.ping_label,
                       self.control_mode_label):
            widget.setStyleSheet("font-size: 12px;")
            conn_layout.addWidget(widget)
        left_layout.addWidget(conn_group)

        # Robot control buttons
        ctrl_group = QGroupBox("Robot Control")
        ctrl_layout = QVBoxLayout(ctrl_group)
        ctrl_layout.setSpacing(6)

        # Status + timer row
        status_row = QHBoxLayout()
        self.robot_status.setStyleSheet("font-size: 13px; font-weight: bold; color: rgb(240, 200, 80);")
        self.timer.setStyleSheet("font-size: 13px; font-weight: bold; color: rgb(140, 210, 240);")
        status_row.addWidget(self.robot_status, 1)
        status_row.addWidget(self.timer)
        ctrl_layout.addLayout(status_row)

        # Mode buttons row
        mode_row = QHBoxLayout()
        mode_row.setSpacing(6)
        self.btn_auto.setMinimumHeight(36)
        self.btn_auto.setStyleSheet(
            "QPushButton { background-color: rgb(40, 80, 40); color: white; border: 1px solid rgb(60, 120, 60);"
            "border-radius: 6px; font-weight: bold; font-size: 13px; }"
            "QPushButton:hover { background-color: rgb(55, 110, 55); }"
            "QPushButton:pressed { background-color: rgb(30, 60, 30); }")
        self.btn_teleop.setMinimumHeight(36)
        self.btn_teleop.setStyleSheet(
            "QPushButton { background-color: rgb(30, 60, 110); color: white; border: 1px solid rgb(50, 90, 160);"
            "border-radius: 6px; font-weight: bold; font-size: 13px; }"
            "QPushButton:hover { background-color: rgb(40, 80, 150); }"
            "QPushButton:pressed { background-color: rgb(20, 45, 85); }")
        self.btn_rst.setMinimumHeight(36)
        self.btn_rst.setStyleSheet(
            "QPushButton { background-color: rgb(110, 30, 30); color: white; border: 1px solid rgb(160, 50, 50);"
            "border-radius: 6px; font-weight: bold; font-size: 13px; }"
            "QPushButton:hover { background-color: rgb(150, 40, 40); }"
            "QPushButton:pressed { background-color: rgb(80, 20, 20); }")
        mode_row.addWidget(self.btn_auto)
        mode_row.addWidget(self.btn_teleop)
        mode_row.addWidget(self.btn_rst)
        ctrl_layout.addLayout(mode_row)

        # Alliance row (keep existing button)
        alliance_row = QHBoxLayout()
        alliance_row.setSpacing(6)
        self.btn_auto_3.setMinimumHeight(28)
        alliance_row.addWidget(self.btn_auto_3)
        ctrl_layout.addLayout(alliance_row)
        left_layout.addWidget(ctrl_group)

        # FMS status panel
        fms_group = QGroupBox("Field Management System")
        fms_layout = QVBoxLayout(fms_group)
        fms_layout.setSpacing(4)

        self.fms_status_label = QLabel("FMS: Not connected")
        self.fms_status_label.setStyleSheet("font-size: 12px;")
        fms_layout.addWidget(self.fms_status_label)

        self.fms_match_label = QLabel("Match: No active match")
        self.fms_match_label.setStyleSheet("font-size: 12px;")
        fms_layout.addWidget(self.fms_match_label)

        self.fms_time_label = QLabel("FMS Time: --")
        self.fms_time_label.setStyleSheet("font-size: 12px;")
        fms_layout.addWidget(self.fms_time_label)

        # Voltage display — prominent, drives the circuit (§3.3.5 Jumpstart the Grid)
        voltage_row = QHBoxLayout()
        voltage_row.setSpacing(6)
        voltage_lbl = QLabel("Grid Voltage:")
        voltage_lbl.setStyleSheet("font-size: 12px;")

        # QStackedWidget: page 0 = normal voltage, page 1 = 30s cooldown overlay
        from PyQt6.QtWidgets import QStackedWidget
        self.voltage_stack = QStackedWidget()
        self.voltage_stack.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        self.fms_voltage_label = QLabel("--  V")
        self.fms_voltage_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: rgb(140, 210, 240);"
        )
        self.fms_voltage_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.voltage_stack.addWidget(self.fms_voltage_label)   # index 0

        self.jumpstart_cooldown_label = QLabel("COOLDOWN  30 s")
        self.jumpstart_cooldown_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: rgb(255, 80, 60);"
            "background: rgba(120, 30, 20, 180); border-radius: 4px; padding: 1px 6px;"
        )
        self.jumpstart_cooldown_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.voltage_stack.addWidget(self.jumpstart_cooldown_label)   # index 1

        voltage_row.addWidget(voltage_lbl)
        voltage_row.addWidget(self.voltage_stack, 1)
        fms_layout.addLayout(voltage_row)

        # Jumpstart button — triggers the 30 s cooldown overlay (§3.3.5)
        self.jumpstart_btn = QPushButton("Jumpstart Grid")
        self.jumpstart_btn.setObjectName("jumpstart_btn")
        self.jumpstart_btn.setMinimumHeight(30)
        self.jumpstart_btn.setStyleSheet(
            "font-weight: bold; font-size: 13px;"
            "background: rgb(60, 130, 60); color: white; border-radius: 4px;"
        )
        fms_layout.addWidget(self.jumpstart_btn)

        # RPM display — for spinning the Charging Wheel (§3.3.4 Generate Electricity)
        rpm_row = QHBoxLayout()
        rpm_row.setSpacing(6)
        rpm_lbl = QLabel("Grid Frequency:")
        rpm_lbl.setStyleSheet("font-size: 12px;")
        self.fms_rpm_label = QLabel("--  RPM")
        self.fms_rpm_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: rgb(255, 200, 100);"
        )
        self.fms_rpm_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        rpm_row.addWidget(rpm_lbl)
        rpm_row.addWidget(self.fms_rpm_label, 1)
        fms_layout.addLayout(rpm_row)

        left_layout.addWidget(fms_group)

        # Auto routine selector
        auto_group = QGroupBox("Autonomous")
        auto_layout = QVBoxLayout(auto_group)
        auto_layout.setSpacing(5)
        self.auto_routine_selector = QComboBox()
        self.auto_routine_selector.setObjectName("auto_routine_selector")
        self.auto_routine_selector.setMinimumHeight(28)
        auto_layout.addWidget(self.auto_routine_selector)

        auto_btn_row = QHBoxLayout()
        auto_btn_row.setSpacing(6)
        self.auto_run_selected_button = QPushButton("Run Auto")
        self.auto_run_selected_button.setObjectName("auto_run_selected_button")
        self.auto_run_selected_button.setMinimumHeight(28)
        self.auto_cancel_button = QPushButton("Cancel")
        self.auto_cancel_button.setObjectName("auto_cancel_button")
        self.auto_cancel_button.setMinimumHeight(28)
        auto_btn_row.addWidget(self.auto_run_selected_button)
        auto_btn_row.addWidget(self.auto_cancel_button)
        auto_layout.addLayout(auto_btn_row)

        self.auto_routine_description_label = QLabel("Selected: None")
        self.auto_routine_description_label.setWordWrap(True)
        self.auto_routine_description_label.setStyleSheet("font-size: 11px; color: rgb(180, 190, 200);")
        self.auto_status_label = QLabel("Status: Idle")
        self.auto_status_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.auto_status_detail_label = QLabel("No routine running")
        self.auto_status_detail_label.setWordWrap(True)
        self.auto_status_detail_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.auto_status_detail_label.setStyleSheet("font-size: 11px; color: rgb(180, 190, 200);")
        auto_layout.addWidget(self.auto_routine_description_label)
        auto_layout.addWidget(self.auto_status_label)
        auto_layout.addWidget(self.auto_status_detail_label)
        left_layout.addWidget(auto_group)

        # Gamepad state
        pad_group = QGroupBox("Gamepad")
        pad_layout = QVBoxLayout(pad_group)
        pad_layout.setSpacing(4)
        self.gamepad_label.setStyleSheet("font-size: 12px;")
        pad_layout.addWidget(self.gamepad_label)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)
        for lbl in (self.button_b_label, self.button_y_label,
                    self.button_a_label, self.button_x_label):
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setMinimumWidth(28)
            lbl.setStyleSheet("border: 1px solid rgb(60, 70, 80); border-radius: 4px;"
                              "padding: 2px; font-size: 12px; font-weight: bold;")
            btn_row.addWidget(lbl)
        pad_layout.addLayout(btn_row)

        axes_row = QHBoxLayout()
        axes_row.setSpacing(4)
        for lbl in (self.lx_label, self.ly_label, self.rx_label, self.ry_label):
            lbl.setStyleSheet("font-size: 11px; color: rgb(180, 190, 200);")
            axes_row.addWidget(lbl)
        pad_layout.addLayout(axes_row)
        left_layout.addWidget(pad_group)

        left_layout.addStretch(1)

        # ── CENTER: FIELD MAP / DRIVER CAMERA TABS ───────────────────────────
        if not hasattr(self, "main_center_tabs"):
            self.main_center_tabs = QTabWidget()
            self.main_center_tabs.setObjectName("main_center_tabs")
        self.main_center_tabs.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        if not hasattr(self, "main_field_tab"):
            self.main_field_tab = QWidget()
        if self.main_field_tab.layout() is None:
            self.main_field_tab_layout = QVBoxLayout(self.main_field_tab)
            self.main_field_tab_layout.setContentsMargins(0, 0, 0, 0)
        else:
            self.main_field_tab_layout = self.main_field_tab.layout()
        if self.main_field_tab_layout.indexOf(self.field_view_placeholder) == -1:
            self.main_field_tab_layout.addWidget(self.field_view_placeholder, 1)

        if not hasattr(self, "main_driver_camera_tab"):
            self.main_driver_camera_tab = QWidget()
        if self.main_driver_camera_tab.layout() is None:
            self.main_driver_camera_tab_layout = QVBoxLayout(self.main_driver_camera_tab)
            self.main_driver_camera_tab_layout.setContentsMargins(0, 0, 0, 0)
        else:
            self.main_driver_camera_tab_layout = self.main_driver_camera_tab.layout()
        if not hasattr(self, "main_driver_camera_placeholder"):
            self.main_driver_camera_placeholder = CameraView("Driver Camera\nWaiting for stream...")
            self.main_driver_camera_placeholder.setMinimumSize(320, 240)
            self.main_driver_camera_tab_layout.addWidget(self.main_driver_camera_placeholder, 1)

        if self.main_center_tabs.indexOf(self.main_field_tab) == -1:
            self.main_center_tabs.addTab(self.main_field_tab, "Field Map")
        else:
            self.main_center_tabs.setTabText(self.main_center_tabs.indexOf(self.main_field_tab), "Field Map")
        if self.main_center_tabs.indexOf(self.main_driver_camera_tab) == -1:
            self.main_center_tabs.addTab(self.main_driver_camera_tab, "Driver Camera")
        else:
            self.main_center_tabs.setTabText(
                self.main_center_tabs.indexOf(self.main_driver_camera_tab), "Driver Camera")

        # ── RIGHT SIDEBAR: camera stack ───────────────────────────────────────
        right_sidebar = None
        if hasattr(self, "main_camera_stack_container"):
            right_sidebar = QWidget(self.frame)
            right_sidebar.setFixedWidth(220)
            right_sidebar.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
            right_layout = QVBoxLayout(right_sidebar)
            right_layout.setContentsMargins(0, 0, 0, 0)
            right_layout.setSpacing(0)
            cam_group = QGroupBox("Cameras")
            cam_layout = QVBoxLayout(cam_group)
            cam_layout.setContentsMargins(6, 6, 6, 6)
            self.main_camera_stack_container.setMinimumHeight(0)
            self.main_camera_stack_container.setSizePolicy(
                QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
            cam_layout.addWidget(self.main_camera_stack_container, 1)
            right_layout.addWidget(cam_group, 1)

        root_layout.addWidget(left_sidebar)
        root_layout.addWidget(self.main_center_tabs, 1)
        if right_sidebar is not None:
            root_layout.addWidget(right_sidebar)

        if hasattr(self, "main_camera_placeholder"):
            self.main_camera_placeholder.hide()

    def setup_odometry_tab(self):
        self.odometry_tab = QWidget()
        layout = QGridLayout(self.odometry_tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        self.odometry_field_widget = FieldWidget(self)
        self.odometry_field_widget.setMinimumHeight(420)
        layout.addWidget(self.odometry_field_widget, 0, 0, 3, 2)

        summary_group = QGroupBox("Pose Summary")
        summary_layout = QVBoxLayout(summary_group)
        self.odo_tab_x_label = QLabel("X: 0.00 m")
        self.odo_tab_y_label = QLabel("Y: 0.00 m")
        self.odo_tab_theta_label = QLabel("Theta: 0.0 deg")
        self.odo_tab_expected_label = QLabel("Expected: X 0.00 m | Y 0.00 m | Theta 0.0 deg")
        for label in (
            self.odo_tab_x_label,
            self.odo_tab_y_label,
            self.odo_tab_theta_label,
            self.odo_tab_expected_label,
        ):
            summary_layout.addWidget(label)
        layout.addWidget(summary_group, 0, 2)

        mechanism_group = QGroupBox("Mechanism Encoders")
        mechanism_layout = QVBoxLayout(mechanism_group)
        self.odo_tab_elevator_left_encoder_label = QLabel("Elevator Left: 0")
        self.odo_tab_elevator_right_encoder_label = QLabel("Elevator Right: 0")
        self.odo_tab_arm_motor_encoder_label = QLabel("Arm Motor: 0")
        for label in (
            self.odo_tab_elevator_left_encoder_label,
            self.odo_tab_elevator_right_encoder_label,
            self.odo_tab_arm_motor_encoder_label,
        ):
            mechanism_layout.addWidget(label)
        mechanism_layout.addStretch(1)
        layout.addWidget(mechanism_group, 1, 2)

        context_group = QGroupBox("Field Context")
        context_layout = QVBoxLayout(context_group)
        self.odo_tab_alliance_label = QLabel("Alliance: Red")
        self.odo_tab_mode_label = QLabel("Mode: Stopped")
        self.odo_tab_odo_mode_label = QLabel("Odometry Mode: Pre_Start")
        self.odo_tab_field_size_label = QLabel("Field: 0.00 m x 0.00 m")
        for label in (
            self.odo_tab_alliance_label,
            self.odo_tab_mode_label,
            self.odo_tab_odo_mode_label,
            self.odo_tab_field_size_label,
        ):
            context_layout.addWidget(label)
        layout.addWidget(context_group, 2, 2)

        actions_group = QGroupBox("Actions")
        actions_layout = QVBoxLayout(actions_group)
        self.odo_tab_reset_button = QPushButton("Reset Odometry")
        actions_layout.addWidget(self.odo_tab_reset_button)

        mode_row = QHBoxLayout()
        self.odo_tab_optical_button = QPushButton("Optical")
        self.odo_tab_motor_button = QPushButton("Motor")
        self.odo_tab_hybrid_button = QPushButton("Hybrid")
        mode_row.addWidget(self.odo_tab_optical_button)
        mode_row.addWidget(self.odo_tab_motor_button)
        mode_row.addWidget(self.odo_tab_hybrid_button)
        actions_layout.addLayout(mode_row)
        actions_layout.addStretch(1)
        layout.addWidget(actions_group, 3, 2)

        rotation_group = QGroupBox("Rotation Rate (OTOS)")
        rotation_layout = QVBoxLayout(rotation_group)

        self.odo_tab_rotation_raw_label = QLabel("Raw:       +0.000 °/s")
        self.odo_tab_rotation_corrected_label = QLabel("Corrected: +0.000 °/s")
        rotation_layout.addWidget(self.odo_tab_rotation_raw_label)
        rotation_layout.addWidget(self.odo_tab_rotation_corrected_label)

        offset_row = QHBoxLayout()
        offset_row.addWidget(QLabel("Offset:"))
        self.odo_tab_gyro_offset_spin = QDoubleSpinBox()
        self.odo_tab_gyro_offset_spin.setRange(-50.0, 50.0)
        self.odo_tab_gyro_offset_spin.setDecimals(3)
        self.odo_tab_gyro_offset_spin.setSingleStep(0.01)
        self.odo_tab_gyro_offset_spin.setSuffix("  °/s")
        self.odo_tab_gyro_offset_spin.setValue(0.0)
        offset_row.addWidget(self.odo_tab_gyro_offset_spin)
        self.odo_tab_gyro_offset_apply_btn = QPushButton("Apply")
        offset_row.addWidget(self.odo_tab_gyro_offset_apply_btn)
        rotation_layout.addLayout(offset_row)

        layout.addWidget(rotation_group, 4, 2)

        self.main_tabs.addTab(self.odometry_tab, "Odometry")

    def setup_mechanism_tab(self):
        self.mechanism_tab = QWidget()

        # Wrap everything in a scroll area so it fits any window size
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        content = QWidget()
        vlayout = QVBoxLayout(content)
        vlayout.setContentsMargins(12, 12, 12, 12)
        vlayout.setSpacing(12)

        # ── Elevator PID ─────────────────────────────────────────────────────
        elev_group = QGroupBox("Elevator PID")
        elev_outer = QHBoxLayout(elev_group)

        gains_layout = QGridLayout()
        gains_layout.addWidget(QLabel("kP:"), 0, 0)
        self.elev_kp_spin = QDoubleSpinBox()
        self.elev_kp_spin.setDecimals(4)
        self.elev_kp_spin.setRange(0.0, 10.0)
        self.elev_kp_spin.setSingleStep(0.0001)
        self.elev_kp_spin.setValue(0.002)
        gains_layout.addWidget(self.elev_kp_spin, 0, 1)

        gains_layout.addWidget(QLabel("kI:"), 1, 0)
        self.elev_ki_spin = QDoubleSpinBox()
        self.elev_ki_spin.setDecimals(6)
        self.elev_ki_spin.setRange(0.0, 10.0)
        self.elev_ki_spin.setSingleStep(0.000001)
        self.elev_ki_spin.setValue(0.0)
        gains_layout.addWidget(self.elev_ki_spin, 1, 1)

        gains_layout.addWidget(QLabel("kD:"), 2, 0)
        self.elev_kd_spin = QDoubleSpinBox()
        self.elev_kd_spin.setDecimals(6)
        self.elev_kd_spin.setRange(0.0, 10.0)
        self.elev_kd_spin.setSingleStep(0.000001)
        self.elev_kd_spin.setValue(0.0)
        gains_layout.addWidget(self.elev_kd_spin, 2, 1)

        gains_layout.addWidget(QLabel("Max Out:"), 3, 0)
        self.elev_max_out_spin = QDoubleSpinBox()
        self.elev_max_out_spin.setDecimals(2)
        self.elev_max_out_spin.setRange(0.0, 1.0)
        self.elev_max_out_spin.setSingleStep(0.05)
        self.elev_max_out_spin.setValue(0.6)
        gains_layout.addWidget(self.elev_max_out_spin, 3, 1)

        gains_layout.addWidget(QLabel("Decel Zone (ticks):"), 4, 0)
        self.elev_decel_zone_spin = QSpinBox()
        self.elev_decel_zone_spin.setRange(0, 10000)
        self.elev_decel_zone_spin.setSingleStep(50)
        self.elev_decel_zone_spin.setValue(800)
        gains_layout.addWidget(self.elev_decel_zone_spin, 4, 1)

        self.elev_set_gains_btn = QPushButton("Set Gains")
        gains_layout.addWidget(self.elev_set_gains_btn, 5, 0, 1, 2)
        elev_outer.addLayout(gains_layout)

        setpoint_layout = QVBoxLayout()
        sp_row = QHBoxLayout()
        sp_row.addWidget(QLabel("Setpoint (ticks):"))
        self.elev_setpoint_spin = QSpinBox()
        self.elev_setpoint_spin.setRange(-100000, 100000)
        self.elev_setpoint_spin.setValue(0)
        sp_row.addWidget(self.elev_setpoint_spin)
        self.elev_go_btn = QPushButton("Go")
        sp_row.addWidget(self.elev_go_btn)
        setpoint_layout.addLayout(sp_row)

        presets_row = QHBoxLayout()
        self.elev_preset_buttons = {}
        for label, ticks in [("Ground", 0), ("Low", 500), ("Mid", 1500), ("High", 3000)]:
            btn = QPushButton(label)
            btn.setProperty("elev_ticks", ticks)
            presets_row.addWidget(btn)
            self.elev_preset_buttons[label] = btn
        setpoint_layout.addLayout(presets_row)

        self.elev_disable_btn = QPushButton("Disable PID")
        self.elev_disable_btn.setStyleSheet("color: rgb(230, 80, 80);")
        setpoint_layout.addWidget(self.elev_disable_btn)
        elev_outer.addLayout(setpoint_layout)

        status_layout = QVBoxLayout()
        self.elev_status_label = QLabel("PID: Inactive")
        self.elev_position_label = QLabel("Position: 0")
        self.elev_setpoint_label = QLabel("Setpoint: 0")
        self.elev_output_label = QLabel("Output: 0.00")
        for lbl in (self.elev_status_label, self.elev_position_label,
                    self.elev_setpoint_label, self.elev_output_label):
            status_layout.addWidget(lbl)
        status_layout.addStretch(1)
        elev_outer.addLayout(status_layout)

        vlayout.addWidget(elev_group)

        # ── Elevator Manual Drive ─────────────────────────────────────────────
        manual_group = QGroupBox("Elevator Manual Drive")
        manual_outer = QHBoxLayout(manual_group)

        def _motor_column(label_text):
            col = QVBoxLayout()
            col.addWidget(QLabel(label_text))
            slider = QSlider(Qt.Orientation.Vertical)
            slider.setRange(-100, 100)
            slider.setValue(0)
            slider.setTickPosition(QSlider.TickPosition.TicksBothSides)
            slider.setTickInterval(25)
            slider.setMinimumHeight(120)
            pct_label = QLabel("0%")
            pct_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
            col.addWidget(slider, 1, Qt.AlignmentFlag.AlignHCenter)
            col.addWidget(pct_label)
            return col, slider, pct_label

        left_col, self.elev_manual_left_slider, self.elev_manual_left_label = _motor_column("Left")
        right_col, self.elev_manual_right_slider, self.elev_manual_right_label = _motor_column("Right")

        self.elev_manual_left_slider.valueChanged.connect(
            lambda v: self.elev_manual_left_label.setText(f"{v}%"))
        self.elev_manual_right_slider.valueChanged.connect(
            lambda v: self.elev_manual_right_label.setText(f"{v}%"))

        manual_outer.addLayout(left_col)
        manual_outer.addLayout(right_col)

        btn_col = QVBoxLayout()
        self.elev_manual_send_btn = QPushButton("Send")
        self.elev_manual_stop_btn = QPushButton("Stop")
        self.elev_manual_stop_btn.setStyleSheet("color: rgb(230, 80, 80);")
        btn_col.addWidget(self.elev_manual_send_btn)
        btn_col.addWidget(self.elev_manual_stop_btn)
        btn_col.addStretch(1)
        manual_outer.addLayout(btn_col)

        vlayout.addWidget(manual_group)

        # ── Arm PID ───────────────────────────────────────────────────────────
        arm_group = QGroupBox("Arm PID  (degrees)")
        arm_outer = QHBoxLayout(arm_group)

        arm_gains_layout = QGridLayout()
        arm_gains_layout.addWidget(QLabel("kP:"), 0, 0)
        self.arm_kp_spin = QDoubleSpinBox()
        self.arm_kp_spin.setDecimals(4)
        self.arm_kp_spin.setRange(0.0, 10.0)
        self.arm_kp_spin.setSingleStep(0.0001)
        self.arm_kp_spin.setValue(0.01)
        arm_gains_layout.addWidget(self.arm_kp_spin, 0, 1)

        arm_gains_layout.addWidget(QLabel("kI:"), 1, 0)
        self.arm_ki_spin = QDoubleSpinBox()
        self.arm_ki_spin.setDecimals(6)
        self.arm_ki_spin.setRange(0.0, 10.0)
        self.arm_ki_spin.setSingleStep(0.000001)
        self.arm_ki_spin.setValue(0.0)
        arm_gains_layout.addWidget(self.arm_ki_spin, 1, 1)

        arm_gains_layout.addWidget(QLabel("kD:"), 2, 0)
        self.arm_kd_spin = QDoubleSpinBox()
        self.arm_kd_spin.setDecimals(6)
        self.arm_kd_spin.setRange(0.0, 10.0)
        self.arm_kd_spin.setSingleStep(0.000001)
        self.arm_kd_spin.setValue(0.0)
        arm_gains_layout.addWidget(self.arm_kd_spin, 2, 1)

        arm_gains_layout.addWidget(QLabel("Max Out:"), 3, 0)
        self.arm_max_out_spin = QDoubleSpinBox()
        self.arm_max_out_spin.setDecimals(2)
        self.arm_max_out_spin.setRange(0.0, 1.0)
        self.arm_max_out_spin.setSingleStep(0.05)
        self.arm_max_out_spin.setValue(0.5)
        arm_gains_layout.addWidget(self.arm_max_out_spin, 3, 1)

        arm_gains_layout.addWidget(QLabel("Decel Zone (°):"), 4, 0)
        self.arm_decel_zone_spin = QDoubleSpinBox()
        self.arm_decel_zone_spin.setDecimals(1)
        self.arm_decel_zone_spin.setRange(0.0, 360.0)
        self.arm_decel_zone_spin.setSingleStep(5.0)
        self.arm_decel_zone_spin.setValue(15.0)
        arm_gains_layout.addWidget(self.arm_decel_zone_spin, 4, 1)

        self.arm_set_gains_btn = QPushButton("Set Gains")
        arm_gains_layout.addWidget(self.arm_set_gains_btn, 5, 0, 1, 2)
        arm_outer.addLayout(arm_gains_layout)

        arm_sp_layout = QVBoxLayout()
        arm_sp_row = QHBoxLayout()
        arm_sp_row.addWidget(QLabel("Setpoint (°):"))
        self.arm_setpoint_spin = QDoubleSpinBox()
        self.arm_setpoint_spin.setDecimals(1)
        self.arm_setpoint_spin.setRange(-720.0, 720.0)
        self.arm_setpoint_spin.setSingleStep(5.0)
        self.arm_setpoint_spin.setValue(0.0)
        arm_sp_row.addWidget(self.arm_setpoint_spin)
        self.arm_go_btn = QPushButton("Go")
        arm_sp_row.addWidget(self.arm_go_btn)
        arm_sp_layout.addLayout(arm_sp_row)

        arm_presets_row = QHBoxLayout()
        self.arm_preset_buttons = {}
        for label, deg in [("0°", 0.0), ("45°", 45.0), ("90°", 90.0), ("180°", 180.0)]:
            btn = QPushButton(label)
            btn.setProperty("arm_degrees", deg)
            arm_presets_row.addWidget(btn)
            self.arm_preset_buttons[label] = btn
        arm_sp_layout.addLayout(arm_presets_row)

        self.arm_disable_btn = QPushButton("Disable PID")
        self.arm_disable_btn.setStyleSheet("color: rgb(230, 80, 80);")
        arm_sp_layout.addWidget(self.arm_disable_btn)
        arm_outer.addLayout(arm_sp_layout)

        arm_status_layout = QVBoxLayout()
        self.arm_status_label = QLabel("PID: Inactive")
        self.arm_position_label = QLabel("Position: 0.0°")
        self.arm_setpoint_label = QLabel("Setpoint: 0.0°")
        self.arm_output_label = QLabel("Output: 0.000")
        for lbl in (self.arm_status_label, self.arm_position_label,
                    self.arm_setpoint_label, self.arm_output_label):
            arm_status_layout.addWidget(lbl)
        arm_status_layout.addStretch(1)
        arm_outer.addLayout(arm_status_layout)

        vlayout.addWidget(arm_group)

        # ── Arm Manual Drive ──────────────────────────────────────────────────
        arm_manual_group = QGroupBox("Arm Manual Drive")
        arm_manual_outer = QHBoxLayout(arm_manual_group)

        arm_slider_col = QVBoxLayout()
        arm_slider_col.addWidget(QLabel("Arm Speed"))
        self.arm_manual_slider = QSlider(Qt.Orientation.Vertical)
        self.arm_manual_slider.setRange(-100, 100)
        self.arm_manual_slider.setValue(0)
        self.arm_manual_slider.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.arm_manual_slider.setTickInterval(25)
        self.arm_manual_slider.setMinimumHeight(120)
        self.arm_manual_label = QLabel("0%")
        self.arm_manual_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.arm_manual_slider.valueChanged.connect(
            lambda v: self.arm_manual_label.setText(f"{v}%"))
        arm_slider_col.addWidget(self.arm_manual_slider, 1, Qt.AlignmentFlag.AlignHCenter)
        arm_slider_col.addWidget(self.arm_manual_label)
        arm_manual_outer.addLayout(arm_slider_col)

        arm_btn_col = QVBoxLayout()
        self.arm_manual_send_btn = QPushButton("Send")
        self.arm_manual_stop_btn = QPushButton("Stop")
        self.arm_manual_stop_btn.setStyleSheet("color: rgb(230, 80, 80);")
        arm_btn_col.addWidget(self.arm_manual_send_btn)
        arm_btn_col.addWidget(self.arm_manual_stop_btn)
        arm_btn_col.addStretch(1)
        arm_manual_outer.addLayout(arm_btn_col)

        vlayout.addWidget(arm_manual_group)

        # ── Claw Servo ───────────────────────────────────────────────────────
        claw_group = QGroupBox("Claw Servo")
        claw_outer = QHBoxLayout(claw_group)

        claw_setpoints_layout = QGridLayout()
        claw_setpoints_layout.addWidget(QLabel("Open Setpoint:"), 0, 0)
        self.claw_open_spin = QDoubleSpinBox()
        self.claw_open_spin.setDecimals(3)
        self.claw_open_spin.setRange(-1.0, 1.0)
        self.claw_open_spin.setSingleStep(0.05)
        self.claw_open_spin.setValue(-1.0)
        claw_setpoints_layout.addWidget(self.claw_open_spin, 0, 1)
        self.claw_open_btn = QPushButton("Open")
        claw_setpoints_layout.addWidget(self.claw_open_btn, 0, 2)

        claw_setpoints_layout.addWidget(QLabel("Closed Setpoint:"), 1, 0)
        self.claw_closed_spin = QDoubleSpinBox()
        self.claw_closed_spin.setDecimals(3)
        self.claw_closed_spin.setRange(-1.0, 1.0)
        self.claw_closed_spin.setSingleStep(0.05)
        self.claw_closed_spin.setValue(1.0)
        claw_setpoints_layout.addWidget(self.claw_closed_spin, 1, 1)
        self.claw_closed_btn = QPushButton("Close")
        claw_setpoints_layout.addWidget(self.claw_closed_btn, 1, 2)

        claw_outer.addLayout(claw_setpoints_layout)

        claw_status_layout = QVBoxLayout()
        self.claw_target_label = QLabel("Target: Open")
        self.claw_setpoint_label = QLabel("Setpoint: -1.000")
        self.claw_hint_label = QLabel("Gamepad: Square toggles open/closed")
        for lbl in (self.claw_target_label, self.claw_setpoint_label, self.claw_hint_label):
            claw_status_layout.addWidget(lbl)
        claw_status_layout.addStretch(1)
        claw_outer.addLayout(claw_status_layout)

        vlayout.addWidget(claw_group)

        # ── Position Presets (trigger / bumper) ───────────────────────────────
        presets_group = QGroupBox("Position Presets (Gamepad)")
        presets_grid = QGridLayout(presets_group)
        presets_grid.addWidget(QLabel("<b>Button</b>"),           0, 0)
        presets_grid.addWidget(QLabel("<b>Name</b>"),             0, 1)
        presets_grid.addWidget(QLabel("<b>Elevator (ticks)</b>"), 0, 2)
        presets_grid.addWidget(QLabel("<b>Arm (°)</b>"),          0, 3)

        self.position_preset_elev_spins = {}
        self.position_preset_arm_spins = {}
        self.position_preset_name_edits = {}
        self.position_preset_go_btns = {}

        for row, key in enumerate(["LB", "LT", "RT", "RB"], start=1):
            presets_grid.addWidget(QLabel(f"<b>{key}</b>"), row, 0)

            name_edit = QLineEdit()
            name_edit.setPlaceholderText("Position name...")
            presets_grid.addWidget(name_edit, row, 1)
            self.position_preset_name_edits[key] = name_edit

            elev_spin = QSpinBox()
            elev_spin.setRange(-100000, 100000)
            elev_spin.setValue(0)
            presets_grid.addWidget(elev_spin, row, 2)
            self.position_preset_elev_spins[key] = elev_spin

            arm_spin = QDoubleSpinBox()
            arm_spin.setDecimals(1)
            arm_spin.setRange(-720.0, 720.0)
            arm_spin.setSingleStep(5.0)
            arm_spin.setValue(0.0)
            presets_grid.addWidget(arm_spin, row, 3)
            self.position_preset_arm_spins[key] = arm_spin

            go_btn = QPushButton("Go")
            presets_grid.addWidget(go_btn, row, 4)
            self.position_preset_go_btns[key] = go_btn

        vlayout.addWidget(presets_group)
        vlayout.addStretch(1)

        scroll.setWidget(content)
        tab_layout = QVBoxLayout(self.mechanism_tab)
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.addWidget(scroll)

        self.main_tabs.addTab(self.mechanism_tab, "Mechanism")

    def setup_settings_tab(self):
        self.settings_tab = QWidget()
        layout = QGridLayout(self.settings_tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        drive_group = QGroupBox("Drive Settings")
        drive_layout = QVBoxLayout(drive_group)
        self.settings_keyboard_speed_label = QLabel("Keyboard Speed: 70%")
        self.settings_keyboard_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.settings_keyboard_speed_slider.setMinimum(0)
        self.settings_keyboard_speed_slider.setMaximum(100)
        self.settings_keyboard_speed_slider.setValue(70)
        self.settings_slow_drive_checkbox = QCheckBox("Enable Slow Drive")
        self.settings_disable_drive_checkbox = QCheckBox("Disable Drive")
        self.settings_only_drive_checkbox = QCheckBox("Only Drive")
        for widget in (
            self.settings_keyboard_speed_label,
            self.settings_keyboard_speed_slider,
            self.settings_slow_drive_checkbox,
            self.settings_disable_drive_checkbox,
            self.settings_only_drive_checkbox,
        ):
            drive_layout.addWidget(widget)
        drive_layout.addStretch(1)
        layout.addWidget(drive_group, 0, 0)

        vision_group = QGroupBox("Vision and Sensors")
        vision_layout = QVBoxLayout(vision_group)
        self.settings_disable_vision_checkbox = QCheckBox("Disable Vision")
        self.settings_check_odo_checkbox = QCheckBox("Check Odometry")
        self.settings_camera_base_label = QLabel(f"Camera Base URL: {os.environ.get('KSU_CAMERA_BASE_URL', 'http://10.42.0.3:8080')}")
        self.settings_camera_streams = QPlainTextEdit()
        self.settings_camera_streams.setReadOnly(True)
        self.settings_camera_streams.setPlainText(
            "\n".join(f"{feed.label}: {feed.stream_url}" for feed in CAMERA_FEEDS)
        )
        vision_layout.addWidget(self.settings_disable_vision_checkbox)
        vision_layout.addWidget(self.settings_check_odo_checkbox)
        vision_layout.addWidget(self.settings_camera_base_label)
        vision_layout.addWidget(self.settings_camera_streams, 1)
        layout.addWidget(vision_group, 0, 1)

        match_group = QGroupBox("Match Preferences")
        match_layout = QVBoxLayout(match_group)
        self.settings_end_after_teleop_checkbox = QCheckBox("End After Teleop")
        self.settings_alliance_summary = QLabel("Alliance: Red")
        self.settings_mode_summary = QLabel("Mode: Stopped")
        self.settings_odometry_summary = QLabel("Odometry Mode: Pre_Start")
        self.settings_auto_summary = QLabel("Selected Auto: None")
        self.settings_auto_status_summary = QLabel("Auto Status: Idle")
        for widget in (
            self.settings_end_after_teleop_checkbox,
            self.settings_alliance_summary,
            self.settings_mode_summary,
            self.settings_odometry_summary,
            self.settings_auto_summary,
            self.settings_auto_status_summary,
        ):
            match_layout.addWidget(widget)
        match_layout.addStretch(1)
        layout.addWidget(match_group, 1, 0, 1, 2)

        self.main_tabs.addTab(self.settings_tab, "Settings")

    def setup_network_tab(self):
        self.network_tab = QWidget()
        layout = QGridLayout(self.network_tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        connection_group = QGroupBox("Connection Status")
        connection_layout = QVBoxLayout(connection_group)
        self.network_status_label = QLabel("Status: Disconnected")
        self.network_active_address_label = QLabel("Active Address: N/A")
        self.network_ping_label = QLabel("Last Ping: -- ms")
        self.network_robot_mode_label = QLabel("Robot Mode: Stopped")
        self.network_last_telemetry_label = QLabel("Last Telemetry: Never")
        for widget in (
            self.network_status_label,
            self.network_active_address_label,
            self.network_ping_label,
            self.network_robot_mode_label,
            self.network_last_telemetry_label,
        ):
            connection_layout.addWidget(widget)
        connection_layout.addStretch(1)
        layout.addWidget(connection_group, 0, 0)

        addresses_group = QGroupBox("Known Robot Addresses")
        addresses_layout = QVBoxLayout(addresses_group)
        self.network_addresses_view = QPlainTextEdit()
        self.network_addresses_view.setReadOnly(True)
        self.network_addresses_view.setPlainText("\n".join(DEFAULT_ROBOT_ADDRESSES))
        addresses_layout.addWidget(self.network_addresses_view)
        layout.addWidget(addresses_group, 0, 1)

        camera_group = QGroupBox("Camera Endpoints")
        camera_layout = QVBoxLayout(camera_group)
        self.network_camera_endpoints = QPlainTextEdit()
        self.network_camera_endpoints.setReadOnly(True)
        self.network_camera_endpoints.setPlainText(
            "\n".join(f"{feed.label}: {feed.stream_url}" for feed in CAMERA_FEEDS)
        )
        camera_layout.addWidget(self.network_camera_endpoints)
        layout.addWidget(camera_group, 1, 0, 1, 2)

        self.main_tabs.addTab(self.network_tab, "Network")

    def setup_diagnostics_tab(self):
        self.diagnostics_tab = QWidget()
        layout = QGridLayout(self.diagnostics_tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        self.diagnostic_panels = {}

        panels = [
            ("connection", "Connection"),
            ("telemetry", "Robot Telemetry"),
            ("camera", "Camera Streams"),
            ("controls", "Operator Input"),
        ]

        for index, (key, title) in enumerate(panels):
            group = QGroupBox(title)
            group_layout = QVBoxLayout(group)
            viewer = QPlainTextEdit()
            viewer.setReadOnly(True)
            viewer.setMaximumBlockCount(250)
            viewer.setStyleSheet(
                "background-color: rgb(15, 20, 25);"
                "color: rgb(220, 230, 235);"
                "border: 1px solid rgb(55, 100, 102);"
            )
            group_layout.addWidget(viewer)
            layout.addWidget(group, index // 2, index % 2)
            self.diagnostic_panels[key] = viewer

        self.main_tabs.addTab(self.diagnostics_tab, "Diagnostics")

        self.append_diagnostic("connection", "Driver station diagnostics initialized")
        self.append_diagnostic("telemetry", "Waiting for robot telemetry...")
        self.append_diagnostic("camera", "Waiting for camera streams...")
        self.append_diagnostic("controls", "Waiting for operator input...")

    def append_diagnostic(self, panel_name, message):
        viewer = getattr(self, "diagnostic_panels", {}).get(panel_name)
        if viewer is None:
            return
        timestamp = time.strftime("%H:%M:%S")
        viewer.appendPlainText(f"[{timestamp}] {message}")

    def append_diagnostic_json(self, panel_name, label, payload):
        try:
            pretty_payload = json.dumps(payload, indent=2, sort_keys=True)
        except TypeError:
            pretty_payload = str(payload)
        self.append_diagnostic(panel_name, f"{label}\n{pretty_payload}")

    def setup_score_tab(self):
        self.score_tab = QWidget()
        layout = QGridLayout(self.score_tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        # ── Input group ──────────────────────────────────────────────────────
        input_group = QGroupBox("Scoring Inputs")
        input_layout = QGridLayout(input_group)

        def _spin(min_val=0, max_val=99, default=0):
            s = QSpinBox()
            s.setMinimum(min_val)
            s.setMaximum(max_val)
            s.setValue(default)
            return s

        row = 0
        input_layout.addWidget(QLabel("Batteries (Auto):"), row, 0)
        self.score_batteries_auto = _spin()
        input_layout.addWidget(self.score_batteries_auto, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Batteries (Teleop):"), row, 0)
        self.score_batteries_tele = _spin()
        input_layout.addWidget(self.score_batteries_tele, row, 1)

        row += 1
        self.score_auto_zone_exit = QCheckBox("Auto Zone Exit (+3)")
        input_layout.addWidget(self.score_auto_zone_exit, row, 0, 1, 2)

        row += 1
        input_layout.addWidget(QLabel("Jumpstarts (Auto):"), row, 0)
        self.score_jumpstarts_auto = _spin()
        input_layout.addWidget(self.score_jumpstarts_auto, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Jumpstarts (Teleop):"), row, 0)
        self.score_jumpstarts_tele = _spin()
        input_layout.addWidget(self.score_jumpstarts_tele, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Wheel Time Auto (s):"), row, 0)
        self.score_wheel_auto = _spin(max_val=9999)
        input_layout.addWidget(self.score_wheel_auto, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Wheel Time Teleop (s):"), row, 0)
        self.score_wheel_tele = _spin(max_val=9999)
        input_layout.addWidget(self.score_wheel_tele, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Climb:"), row, 0)
        self.score_climb_combo = QComboBox()
        self.score_climb_combo.addItems(["None", "Line", "High"])
        input_layout.addWidget(self.score_climb_combo, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Minor Penalties:"), row, 0)
        self.score_minor_penalties = _spin()
        input_layout.addWidget(self.score_minor_penalties, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Major Penalties:"), row, 0)
        self.score_major_penalties = _spin()
        input_layout.addWidget(self.score_major_penalties, row, 1)

        calc_btn = QPushButton("Calculate Score")
        calc_btn.clicked.connect(self._recalculate_score)
        input_layout.addWidget(calc_btn, row + 1, 0, 1, 2)

        layout.addWidget(input_group, 0, 0)

        # ── Breakdown group ──────────────────────────────────────────────────
        breakdown_group = QGroupBox("Score Breakdown")
        breakdown_layout = QVBoxLayout(breakdown_group)
        self.score_breakdown_table = QTableWidget(0, 2)
        self.score_breakdown_table.setHorizontalHeaderLabels(["Component", "Points"])
        self.score_breakdown_table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.Stretch
        )
        self.score_breakdown_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.score_breakdown_table.setStyleSheet(
            "background-color: rgb(15, 20, 25); color: rgb(220, 230, 235);"
        )
        breakdown_layout.addWidget(self.score_breakdown_table)
        self.score_total_label = QLabel("Total: 0")
        self.score_total_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        breakdown_layout.addWidget(self.score_total_label)

        layout.addWidget(breakdown_group, 0, 1)

        self.main_tabs.addTab(self.score_tab, "Score")

    def _recalculate_score(self):
        if calculate_score is None:
            return
        breakdown = calculate_score(
            batteries_auto=self.score_batteries_auto.value(),
            batteries_teleop=self.score_batteries_tele.value(),
            auto_zone_exit=self.score_auto_zone_exit.isChecked(),
            jumpstarts_auto=self.score_jumpstarts_auto.value(),
            jumpstarts_teleop=self.score_jumpstarts_tele.value(),
            wheel_time_auto_s=self.score_wheel_auto.value(),
            wheel_time_teleop_s=self.score_wheel_tele.value(),
            climb=self.score_climb_combo.currentText(),
            minor_penalties=self.score_minor_penalties.value(),
            major_penalties=self.score_major_penalties.value(),
        )
        display_rows = [
            ("Battery pts (auto)", breakdown["battery_pts_auto"]),
            ("Battery pts (teleop)", breakdown["battery_pts_tele"]),
            ("Capacity", breakdown["capacity"]),
            ("KJ from jumpstarts", breakdown["kj_jumpstart"]),
            ("KJ from wheel", breakdown["kj_wheel"]),
            ("KJ pts (capped)", breakdown["kj_pts"]),
            ("Auto exit pts", breakdown["auto_exit_pts"]),
            ("Climb pts", breakdown["climb_pts"]),
            ("Penalties", -breakdown["penalty_pts"]),
        ]
        table = self.score_breakdown_table
        table.setRowCount(len(display_rows))
        for r, (label, value) in enumerate(display_rows):
            table.setItem(r, 0, QTableWidgetItem(label))
            table.setItem(r, 1, QTableWidgetItem(str(value)))
        self.score_total_label.setText(f"Total: {breakdown['total']}")

    def setup_match_history_tab(self):
        self.match_history_tab = QWidget()
        layout = QVBoxLayout(self.match_history_tab)
        layout.setContentsMargins(12, 12, 12, 12)

        # Stats bar
        stats_layout = QHBoxLayout()
        self.mh_wins_label = QLabel("W: 0")
        self.mh_losses_label = QLabel("L: 0")
        self.mh_ties_label = QLabel("T: 0")
        self.mh_avg_label = QLabel("Avg: 0.0")
        self.mh_high_label = QLabel("High: 0")
        for lbl in (self.mh_wins_label, self.mh_losses_label, self.mh_ties_label,
                    self.mh_avg_label, self.mh_high_label):
            lbl.setStyleSheet("font-weight: bold; margin-right: 16px;")
            stats_layout.addWidget(lbl)
        stats_layout.addStretch(1)
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_match_history)
        stats_layout.addWidget(refresh_btn)
        layout.addLayout(stats_layout)

        # History table
        cols = ["ID", "Label", "Type", "Alliance", "Opponent", "Our Score",
                "Opp Score", "Outcome", "Notes"]
        self.mh_table = QTableWidget(0, len(cols))
        self.mh_table.setHorizontalHeaderLabels(cols)
        self.mh_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.mh_table.horizontalHeader().setSectionResizeMode(8, QHeaderView.ResizeMode.Stretch)
        self.mh_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.mh_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.mh_table.setStyleSheet(
            "background-color: rgb(15, 20, 25); color: rgb(220, 230, 235);"
        )
        layout.addWidget(self.mh_table, 1)

        self.main_tabs.addTab(self.match_history_tab, "Match History")

        if MatchHistory is not None:
            self._mh_store = MatchHistory.load()
            self._refresh_match_history()
        else:
            self._mh_store = None

    def _refresh_match_history(self):
        if self._mh_store is None:
            return
        self._mh_store = MatchHistory.load()
        h = self._mh_store
        self.mh_wins_label.setText(f"W: {h.wins}")
        self.mh_losses_label.setText(f"L: {h.losses}")
        self.mh_ties_label.setText(f"T: {h.ties}")
        self.mh_avg_label.setText(f"Avg: {h.avg_score:.1f}")
        self.mh_high_label.setText(f"High: {h.high_score}")

        self.mh_table.setRowCount(0)
        outcome_colors = {"WIN": "#2e7d32", "LOSS": "#b71c1c", "TIE": "#f57f17"}
        for match in reversed(h.matches):
            r = self.mh_table.rowCount()
            self.mh_table.insertRow(r)
            cells = [
                match.match_id, match.match_label, match.match_type,
                match.alliance, match.opponent_team, str(match.our_score),
                str(match.opponent_score), match.outcome, match.notes,
            ]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if c == 7:
                    color = outcome_colors.get(match.outcome, "#ffffff")
                    item.setForeground(QColor(color))
                self.mh_table.setItem(r, c, item)

    def setup_field_view(self):
        self.field_widget = FieldWidget(self)

        if hasattr(self, "field_view_placeholder"):
            container = self.field_view_placeholder
            if container.layout() is None:
                layout = QVBoxLayout(container)
                layout.setContentsMargins(0, 0, 0, 0)
            else:
                layout = container.layout()
            layout.addWidget(self.field_widget)
        elif hasattr(self, "gridLayout_4"):
            self.gridLayout_4.addWidget(self.field_widget, 6, 0, 1, 1)
        elif hasattr(self, "gridLayout_5"):
            self.gridLayout_5.addWidget(self.field_widget, 0, 0, 3, 1)

        self.update_odometry_labels(0.0, 0.0, 0.0)

    def setup_main_camera_view(self):
        self.main_camera_views = {}
        self.center_driver_camera_view = None

        if hasattr(self, "main_camera_stack_container"):
            container = self.main_camera_stack_container
            if container.layout() is None:
                layout = QVBoxLayout(container)
                layout.setContentsMargins(0, 0, 0, 0)
                layout.setSpacing(10)
            else:
                layout = container.layout()
            for feed in CAMERA_FEEDS:
                view = CameraView(feed.label)
                view.setMinimumSize(220, 180)
                view.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
                self.main_camera_views[feed.name] = view
                layout.addWidget(view, 1)

        if hasattr(self, "main_driver_camera_placeholder"):
            container = self.main_driver_camera_placeholder
            if container.layout() is None:
                layout = QVBoxLayout(container)
                layout.setContentsMargins(0, 0, 0, 0)
            else:
                layout = container.layout()

            if getattr(self, "center_driver_camera_view", None) is None:
                self.center_driver_camera_view = CameraView("Driver Camera\nWaiting for stream...")
                self.center_driver_camera_view.setMinimumSize(320, 240)
                layout.addWidget(self.center_driver_camera_view, 1)

    def setup_camera_stream(self):
        self.camera_streams = {}
        self.camera_views = {}

        for feed in CAMERA_FEEDS:
            views = []
            tab_view = getattr(self, "camera_tab_views", {}).get(feed.name)
            if tab_view is not None:
                views.append(tab_view)
            main_view = getattr(self, "main_camera_views", {}).get(feed.name)
            if main_view is not None:
                views.append(main_view)
            if feed.name == "driver":
                center_driver_view = getattr(self, "center_driver_camera_view", None)
                if center_driver_view is not None:
                    views.append(center_driver_view)

            self.camera_views[feed.name] = views

            if not views:
                continue

            stream = CameraStreamThread(feed.name, feed.stream_url, self)
            stream.frame_ready.connect(self.handle_camera_frame)
            stream.status_changed.connect(self.handle_camera_status)
            self.camera_streams[feed.name] = stream
            stream.start()

    def handle_camera_frame(self, camera_name, image):
        self._camera_latest_frames[camera_name] = image

    def handle_camera_status(self, camera_name, status):
        self._camera_status_cache[camera_name] = status
        feed = next((item for item in CAMERA_FEEDS if item.name == camera_name), None)
        status_prefix = feed.label if feed is not None else camera_name
        status_label = getattr(self, "camera_tab_status_labels", {}).get(camera_name)
        if status_label is not None:
            status_label.setText(f"{status_prefix}: {status}")

        self.append_diagnostic("camera", f"{status_prefix}: {status}")

        for view in self.camera_views.get(camera_name, []):
            if hasattr(view, "has_frame") and not view.has_frame():
                view.setText(f"{status_prefix}\n{status}")

    def _visible_camera_views(self, camera_name):
        if not hasattr(self, "main_tabs"):
            return []

        current_widget = self.main_tabs.currentWidget()
        visible_views = []

        if current_widget is self.frame:
            main_view = getattr(self, "main_camera_views", {}).get(camera_name)
            if main_view is not None:
                visible_views.append(main_view)
            if camera_name == "driver":
                center_driver_view = getattr(self, "center_driver_camera_view", None)
                center_tabs = getattr(self, "main_center_tabs", None)
                driver_tab = getattr(self, "main_driver_camera_tab", None)
                if (
                    center_driver_view is not None
                    and center_tabs is not None
                    and driver_tab is not None
                    and center_tabs.currentWidget() is driver_tab
                ):
                    visible_views.append(center_driver_view)

        if current_widget is getattr(self, "camera_tab", None):
            tab_view = getattr(self, "camera_tab_views", {}).get(camera_name)
            if tab_view is not None:
                visible_views.append(tab_view)

        return visible_views

    def _flush_camera_frames(self):
        latest_frames = getattr(self, "_camera_latest_frames", None)
        if not latest_frames:
            return

        for camera_name, image in list(latest_frames.items()):
            visible_views = self._visible_camera_views(camera_name)
            if not visible_views:
                continue
            for view in visible_views:
                view.set_frame(image)

    def _handle_tab_changed(self, index):
        del index
        self._flush_camera_frames()

        current_widget = self.main_tabs.currentWidget()
        if current_widget not in (self.frame, getattr(self, "camera_tab", None)):
            return

        for feed in CAMERA_FEEDS:
            status = self._camera_status_cache.get(feed.name)
            if not status:
                continue
            for view in self._visible_camera_views(feed.name):
                if not view.has_frame():
                    view.setText(f"{feed.label}\n{status}")

    def stop_camera_stream(self):
        render_timer = getattr(self, "_camera_render_timer", None)
        if render_timer is not None:
            render_timer.stop()
        for stream in getattr(self, "camera_streams", {}).values():
            stream.stop()
        for stream in getattr(self, "camera_streams", {}).values():
            stream.wait(1500)
