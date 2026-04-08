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

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTabWidget, QLabel, QGridLayout, QGroupBox, QPlainTextEdit, QPushButton, QHBoxLayout, QSizePolicy, QSlider, QCheckBox, QComboBox
from PyQt6.QtCore import Qt, QPointF, QRectF, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF, QImage, QPixmap

try:
    import cv2
except ImportError:
    cv2 = None

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
    def __init__(self, parent=None):
        super().__init__(parent)
        self.field_width_m = 3.6
        self.field_height_m = 3.6
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
        self.setup_diagnostics_tab()
        self.main_tabs.currentChanged.connect(self._handle_tab_changed)
        self._camera_render_timer.start()

    def _panel_style(self):
        return (
            "QFrame, QGroupBox {"
            "background-color: rgb(18, 24, 30);"
            "border: 1px solid rgb(55, 100, 102);"
            "border-radius: 10px;"
            "}"
            "QLabel { color: rgb(225, 232, 236); }"
            "QPushButton {"
            "background-color: rgb(37, 54, 64);"
            "color: rgb(240, 245, 247);"
            "border: 1px solid rgb(77, 122, 126);"
            "border-radius: 8px;"
            "padding: 6px 10px;"
            "}"
            "QPushButton:hover { background-color: rgb(49, 70, 82); }"
            "QComboBox {"
            "background-color: rgb(37, 54, 64);"
            "color: rgb(240, 245, 247);"
            "border: 1px solid rgb(77, 122, 126);"
            "border-radius: 8px;"
            "padding: 6px 10px;"
            "}"
            "QTabWidget::pane {"
            "border: 1px solid rgb(55, 100, 102);"
            "background-color: rgb(14, 19, 24);"
            "border-radius: 8px;"
            "top: -1px;"
            "}"
            "QTabBar::tab {"
            "background-color: rgb(24, 32, 40);"
            "color: rgb(210, 220, 226);"
            "border: 1px solid rgb(55, 100, 102);"
            "padding: 6px 12px;"
            "margin-right: 4px;"
            "border-top-left-radius: 8px;"
            "border-top-right-radius: 8px;"
            "}"
            "QTabBar::tab:selected {"
            "background-color: rgb(37, 54, 64);"
            "color: rgb(240, 245, 247);"
            "}"
            "QTabBar::tab:hover { background-color: rgb(44, 63, 74); }"
        )

    def setup_main_page_layout(self):
        self.frame.setStyleSheet("background-color: rgb(11, 16, 20);")
        existing_layout = self.frame.layout()
        if existing_layout is None:
            root_layout = QGridLayout(self.frame)
        else:
            root_layout = existing_layout
            while root_layout.count():
                item = root_layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.setParent(None)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setHorizontalSpacing(12)
        root_layout.setVerticalSpacing(12)

        # Hide the original geometry-based containers so they do not sit on top of
        # the rebuilt dashboard and steal mouse events.
        legacy_panels = [
            "frame_connection",
            "frame_robot_control",
            "frame_connection_2",
            "frame_connection_3",
            "frame_keyboard",
        ]
        for name in legacy_panels:
            panel = getattr(self, name, None)
            if panel is not None:
                panel.hide()

        left_column = QWidget(self.frame)
        left_layout = QVBoxLayout(left_column)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(12)

        status_group = QGroupBox("Connection")
        status_layout = QVBoxLayout(status_group)
        for widget in (
            self.status_label,
            self.address_label,
            self.ping_label,
            self.gamepad_label,
            self.control_mode_label,
        ):
            status_layout.addWidget(widget)
        buttons_row = QHBoxLayout()
        buttons_row.setSpacing(8)
        for widget in (
            self.button_b_label,
            self.button_y_label,
            self.button_a_label,
            self.button_x_label,
        ):
            buttons_row.addWidget(widget)
        status_layout.addLayout(buttons_row)
        axes_row = QHBoxLayout()
        axes_row.setSpacing(8)
        for widget in (
            self.lx_label,
            self.rx_label,
            self.ly_label,
            self.ry_label,
        ):
            axes_row.addWidget(widget)
        status_layout.addLayout(axes_row)
        left_layout.addWidget(status_group)

        options_group = QGroupBox("Options")
        options_layout = QVBoxLayout(options_group)
        for widget in (
            self.end_after_teleop,
            self.slow_drive,
            self.only_drive,
            self.disable_drive,
            self.disable_vision,
            self.check_odo,
            self.checkBox,
        ):
            options_layout.addWidget(widget)
        options_layout.addStretch(1)
        left_layout.addWidget(options_group)

        odometry_group = QGroupBox("Odometry Controls")
        odometry_layout = QVBoxLayout(odometry_group)
        odometry_layout.addWidget(self.label_odo_mode)
        pose_row = QHBoxLayout()
        for widget in (self.label_3, self.label_2, self.label_4):
            pose_row.addWidget(widget)
        odometry_layout.addLayout(pose_row)
        mode_row = QHBoxLayout()
        for widget in (self.btn_odo_optical, self.btn_odo_motor, self.btn_odo_hybrid):
            mode_row.addWidget(widget)
        odometry_layout.addLayout(mode_row)
        odometry_layout.addWidget(self.pushButton)
        odometry_layout.addStretch(1)
        left_layout.addWidget(odometry_group, 1)

        center_column = QWidget(self.frame)
        center_layout = QVBoxLayout(center_column)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(12)

        control_group = QGroupBox("Robot Control")
        control_layout = QGridLayout(control_group)
        control_layout.addWidget(self.label, 0, 0)
        control_layout.addWidget(self.robot_status, 0, 1)
        control_layout.addWidget(self.timer, 0, 2)
        control_layout.addWidget(self.btn_auto, 1, 0)
        control_layout.addWidget(self.btn_teleop, 1, 1)
        control_layout.addWidget(self.btn_rst, 1, 2)
        control_layout.addWidget(self.btn_auto_2, 2, 0)
        control_layout.addWidget(self.btn_auto_3, 2, 1, 1, 2)
        self.auto_routine_selector = QComboBox()
        self.auto_routine_selector.setObjectName("auto_routine_selector")
        self.auto_run_selected_button = QPushButton("Run Selected Auto")
        self.auto_run_selected_button.setObjectName("auto_run_selected_button")
        self.auto_cancel_button = QPushButton("Cancel Auto")
        self.auto_cancel_button.setObjectName("auto_cancel_button")
        self.auto_routine_description_label = QLabel("Selected Auto: None")
        self.auto_routine_description_label.setWordWrap(True)
        self.auto_status_label = QLabel("Auto Status: Idle")
        self.auto_status_detail_label = QLabel("Auto Detail: No routine running")
        self.auto_status_detail_label.setWordWrap(True)
        self.auto_status_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.auto_status_detail_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        control_layout.addWidget(self.auto_routine_selector, 3, 0, 1, 2)
        control_layout.addWidget(self.auto_run_selected_button, 3, 2)
        control_layout.addWidget(self.auto_cancel_button, 4, 0)
        control_layout.addWidget(self.auto_status_label, 4, 1, 1, 2)
        control_layout.addWidget(self.auto_routine_description_label, 5, 0, 1, 3)
        control_layout.addWidget(self.auto_status_detail_label, 6, 0, 1, 3)
        control_layout.setColumnStretch(0, 1)
        control_layout.setColumnStretch(1, 1)
        control_layout.setColumnStretch(2, 1)
        center_layout.addWidget(control_group)

        field_group = QGroupBox("Field / Driver View")
        field_layout = QVBoxLayout(field_group)
        field_layout.setContentsMargins(10, 10, 10, 10)
        if not hasattr(self, "main_center_tabs"):
            self.main_center_tabs = QTabWidget()
            self.main_center_tabs.setObjectName("main_center_tabs")
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
            self.main_center_tabs.setTabText(self.main_center_tabs.indexOf(self.main_driver_camera_tab), "Driver Camera")
        field_layout.addWidget(self.main_center_tabs, 1)
        center_layout.addWidget(field_group, 1)

        right_column = QWidget(self.frame)
        right_layout = QVBoxLayout(right_column)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(12)

        if hasattr(self, "main_camera_stack_container") or hasattr(self, "main_camera_placeholder"):
            camera_group = QGroupBox("Camera Stack")
            camera_layout = QVBoxLayout(camera_group)
            camera_layout.setContentsMargins(10, 10, 10, 10)
            if not hasattr(self, "main_camera_stack_container"):
                self.main_camera_stack_container = QWidget()
                self.main_camera_stack_container.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
                self.main_camera_stack_container.setStyleSheet("background-color: transparent;")
            self.main_camera_stack_container.setMinimumHeight(0)
            self.main_camera_stack_container.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
            camera_layout.addWidget(self.main_camera_stack_container, 1)
            right_layout.addWidget(camera_group, 1)

        root_layout.addWidget(left_column, 0, 0)
        root_layout.addWidget(center_column, 0, 1)
        root_layout.addWidget(right_column, 0, 2)
        root_layout.setColumnStretch(0, 2)
        root_layout.setColumnStretch(1, 5)
        root_layout.setColumnStretch(2, 2)

        left_column.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        center_column.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        right_column.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        control_group.setMinimumHeight(140)
        if hasattr(self, "main_camera_placeholder"):
            self.main_camera_placeholder.hide()
        if hasattr(self, "main_camera_stack_container"):
            self.main_camera_stack_container.setMinimumWidth(220)
            self.main_camera_stack_container.setMinimumHeight(0)

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

        self.main_tabs.addTab(self.odometry_tab, "Odometry")

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
