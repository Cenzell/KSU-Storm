import os
import math
import logging
import urllib.request
import json
import time
from dataclasses import dataclass
from pathlib import Path
import numpy as np

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTabWidget, QLabel, QGridLayout, QGroupBox, QPlainTextEdit, QPushButton, QHBoxLayout, QSizePolicy
from PyQt6.QtCore import Qt, QPointF, QRectF, QThread, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF, QImage, QPixmap

try:
    import cv2
except ImportError:
    cv2 = None

logger = logging.getLogger(__name__)

CAMERA_RECONNECT_MS = 1500
DRIVERSTATION_DIR = Path(__file__).resolve().parents[1]
FIELD_IMAGE_PATH = DRIVERSTATION_DIR / "field.png"
ROBOT_SIZE_M = 18.0 * 0.0254


@dataclass(frozen=True)
class CameraFeedConfig:
    name: str
    label: str
    stream_url: str


def build_camera_feed_configs():
    base_url = os.environ.get("KSU_CAMERA_BASE_URL", "http://10.42.0.3:8080").rstrip("/")
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
            self.status_changed.emit(self.camera_name, "Camera unavailable: OpenCV not installed")
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

        self.gridLayout.removeWidget(self.frame)
        self.gridLayout.addWidget(self.main_tabs, 1, 1, 1, 1)
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

        for tab_name in ["Settings", "Network"]:
            tab = QWidget()
            tab_layout = QVBoxLayout(tab)
            tab_layout.setContentsMargins(12, 12, 12, 12)
            tab_layout.addWidget(QLabel(f"{tab_name} page - add controls here."))
            tab_layout.addStretch(1)
            self.main_tabs.addTab(tab, tab_name)

        self.setup_odometry_tab()
        self.setup_diagnostics_tab()

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
        )

    def setup_main_page_layout(self):
        self.frame.setStyleSheet("background-color: rgb(11, 16, 20);")
        frame_connection = getattr(self, "frame_connection", None)
        frame_robot_control = getattr(self, "frame_robot_control", None)
        field_view_placeholder = getattr(self, "field_view_placeholder", None)
        main_camera_placeholder = getattr(self, "main_camera_placeholder", None)
        frame_connection_2 = getattr(self, "frame_connection_2", None)
        frame_connection_3 = getattr(self, "frame_connection_3", None)
        frame_keyboard = getattr(self, "frame_keyboard", None)

        panel_names = [
            "frame_connection",
            "frame_robot_control",
            "field_view_placeholder",
            "main_camera_placeholder",
            "frame_connection_2",
            "frame_connection_3",
            "frame_keyboard",
        ]

        for name in panel_names:
            panel = getattr(self, name, None)
            if panel is None:
                continue
            panel.setParent(None)
            panel.setStyleSheet(self._panel_style())

        root_layout = QGridLayout(self.frame)
        root_layout.setContentsMargins(14, 14, 14, 14)
        root_layout.setHorizontalSpacing(14)
        root_layout.setVerticalSpacing(14)

        left_column = QWidget(self.frame)
        left_layout = QVBoxLayout(left_column)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(14)
        if frame_connection is not None:
            left_layout.addWidget(frame_connection)
        if frame_connection_2 is not None:
            left_layout.addWidget(frame_connection_2)
        if frame_connection_3 is not None:
            left_layout.addWidget(frame_connection_3, 1)

        center_column = QWidget(self.frame)
        center_layout = QVBoxLayout(center_column)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(14)
        if frame_robot_control is not None:
            center_layout.addWidget(frame_robot_control)
        if field_view_placeholder is not None:
            center_layout.addWidget(field_view_placeholder, 1)

        right_column = QWidget(self.frame)
        right_layout = QVBoxLayout(right_column)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(14)
        if main_camera_placeholder is not None:
            right_layout.addWidget(main_camera_placeholder, 1)
        if frame_keyboard is not None:
            right_layout.addWidget(frame_keyboard)

        root_layout.addWidget(left_column, 0, 0)
        root_layout.addWidget(center_column, 0, 1)
        root_layout.addWidget(right_column, 0, 2)
        root_layout.setColumnStretch(0, 2)
        root_layout.setColumnStretch(1, 5)
        root_layout.setColumnStretch(2, 2)

        left_column.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        center_column.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        right_column.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        if frame_robot_control is not None:
            frame_robot_control.setMinimumHeight(116)
        if frame_keyboard is not None:
            frame_keyboard.setMinimumHeight(280)
        if main_camera_placeholder is not None:
            main_camera_placeholder.setMinimumWidth(220)
            main_camera_placeholder.setMinimumHeight(320)

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
        layout.addWidget(context_group, 1, 2)

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
        layout.addWidget(actions_group, 2, 2)

        self.main_tabs.addTab(self.odometry_tab, "Odometry")

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

        if hasattr(self, "main_camera_placeholder"):
            container = self.main_camera_placeholder
            if container.layout() is None:
                layout = QVBoxLayout(container)
                layout.setContentsMargins(0, 0, 0, 0)
                layout.setSpacing(6)
            else:
                layout = container.layout()
            for feed in CAMERA_FEEDS:
                view = CameraView(feed.label)
                view.setMinimumSize(120, 90)
                self.main_camera_views[feed.name] = view
                layout.addWidget(view)

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

            self.camera_views[feed.name] = views

            if not views:
                continue

            stream = CameraStreamThread(feed.name, feed.stream_url, self)
            stream.frame_ready.connect(self.handle_camera_frame)
            stream.status_changed.connect(self.handle_camera_status)
            self.camera_streams[feed.name] = stream
            stream.start()

    def handle_camera_frame(self, camera_name, image):
        for view in self.camera_views.get(camera_name, []):
            view.set_frame(image)

    def handle_camera_status(self, camera_name, status):
        feed = next((item for item in CAMERA_FEEDS if item.name == camera_name), None)
        status_prefix = feed.label if feed is not None else camera_name
        status_label = getattr(self, "camera_tab_status_labels", {}).get(camera_name)
        if status_label is not None:
            status_label.setText(f"{status_prefix}: {status}")

        self.append_diagnostic("camera", f"{status_prefix}: {status}")

        for view in self.camera_views.get(camera_name, []):
            if not view.has_frame():
                view.setText(f"{status_prefix}\n{status}")

    def stop_camera_stream(self):
        for stream in getattr(self, "camera_streams", {}).values():
            stream.stop()
        for stream in getattr(self, "camera_streams", {}).values():
            stream.wait(1500)
