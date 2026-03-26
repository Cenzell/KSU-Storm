import os
import math
import logging

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTabWidget, QLabel
from PyQt6.QtCore import Qt, QPointF, QRectF, QThread, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF, QImage, QPixmap

try:
    import cv2
except ImportError:
    cv2 = None

logger = logging.getLogger(__name__)

CAMERA_STREAM_URL = os.environ.get("KSU_CAMERA_STREAM_URL", "http://10.42.0.3:8080/stream.mjpg")
CAMERA_RECONNECT_MS = 1500


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

        painter.fillRect(draw_rect, QColor(30, 45, 55))
        painter.setPen(QPen(QColor(95, 140, 150), 2))
        painter.drawRect(draw_rect)

        painter.setPen(QPen(QColor(70, 95, 110), 1, Qt.PenStyle.DashLine))
        for i in range(1, 6):
            x = draw_rect.left() + (draw_rect.width() * i / 6.0)
            y = draw_rect.top() + (draw_rect.height() * i / 6.0)
            painter.drawLine(QPointF(x, draw_rect.top()), QPointF(x, draw_rect.bottom()))
            painter.drawLine(QPointF(draw_rect.left(), y), QPointF(draw_rect.right(), y))

        center = self._field_to_screen(self.robot_x_m, self.robot_y_m, draw_rect)
        robot_radius_px = max(6, min(draw_rect.width(), draw_rect.height()) * 0.03)

        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.setBrush(QBrush(QColor(230, 120, 40)))
        painter.drawEllipse(center, robot_radius_px, robot_radius_px)

        heading_rad = math.radians(self.robot_theta_deg)
        arrow_len = robot_radius_px * 2.2
        tip = QPointF(
            center.x() + arrow_len * math.cos(heading_rad),
            center.y() - arrow_len * math.sin(heading_rad),
        )
        left = QPointF(
            tip.x() - robot_radius_px * 0.6 * math.cos(heading_rad - 0.6),
            tip.y() + robot_radius_px * 0.6 * math.sin(heading_rad - 0.6),
        )
        right = QPointF(
            tip.x() - robot_radius_px * 0.6 * math.cos(heading_rad + 0.6),
            tip.y() + robot_radius_px * 0.6 * math.sin(heading_rad + 0.6),
        )
        painter.setBrush(QBrush(QColor(255, 220, 120)))
        painter.drawPolygon(QPolygonF([tip, left, right]))

        expected_center = self._field_to_screen(self.expected_x_m, self.expected_y_m, draw_rect)
        painter.setPen(QPen(QColor(125, 235, 240), 2, Qt.PenStyle.DashLine))
        painter.drawLine(center, expected_center)
        painter.setPen(QPen(QColor(125, 235, 240), 2))
        painter.setBrush(QBrush(QColor(60, 180, 200, 80)))
        painter.drawEllipse(expected_center, robot_radius_px * 0.8, robot_radius_px * 0.8)

        expected_heading_rad = math.radians(self.expected_theta_deg)
        expected_tip = QPointF(
            expected_center.x() + arrow_len * 0.8 * math.cos(expected_heading_rad),
            expected_center.y() - arrow_len * 0.8 * math.sin(expected_heading_rad),
        )
        painter.drawLine(expected_center, expected_tip)

        painter.setPen(QPen(QColor(235, 235, 235), 1))
        painter.drawText(8, 16, "Field View (orange=current, cyan=expected)")


class CameraStreamThread(QThread):
    frame_ready = pyqtSignal(QImage)
    status_changed = pyqtSignal(str)

    def __init__(self, stream_url, parent=None):
        super().__init__(parent)
        self.stream_url = stream_url
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        if cv2 is None:
            self.status_changed.emit("Camera unavailable: OpenCV not installed")
            return

        while self._running:
            self.status_changed.emit(f"Connecting: {self.stream_url}")
            capture = cv2.VideoCapture(self.stream_url)
            if not capture.isOpened():
                self.status_changed.emit("Camera disconnected (retrying...)")
                self.msleep(CAMERA_RECONNECT_MS)
                continue

            self.status_changed.emit("Camera connected")
            while self._running:
                ok, frame_bgr = capture.read()
                if not ok:
                    self.status_changed.emit("Camera stream dropped (reconnecting...)")
                    break

                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                h, w, c = frame_rgb.shape
                image = QImage(frame_rgb.data, w, h, c * w, QImage.Format.Format_RGB888).copy()
                self.frame_ready.emit(image)

            capture.release()
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

        self.camera_tab = QWidget()
        camera_layout = QVBoxLayout(self.camera_tab)
        camera_layout.setContentsMargins(10, 10, 10, 10)
        self.camera_status_label = QLabel(f"Source: {CAMERA_STREAM_URL}")
        self.camera_status_label.setStyleSheet("color: rgb(200, 210, 215);")
        self.camera_tab_view = CameraView("Waiting for camera stream...")
        camera_layout.addWidget(self.camera_status_label)
        camera_layout.addWidget(self.camera_tab_view, 1)
        self.main_tabs.addTab(self.camera_tab, "Camera")

        for tab_name in ["Settings", "Network", "Odometry", "Diagnostics"]:
            tab = QWidget()
            tab_layout = QVBoxLayout(tab)
            tab_layout.setContentsMargins(12, 12, 12, 12)
            tab_layout.addWidget(QLabel(f"{tab_name} page - add controls here."))
            tab_layout.addStretch(1)
            self.main_tabs.addTab(tab, tab_name)

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
        self.main_camera_view = None

        if hasattr(self, "main_camera_placeholder"):
            container = self.main_camera_placeholder
            if container.layout() is None:
                layout = QVBoxLayout(container)
                layout.setContentsMargins(0, 0, 0, 0)
            else:
                layout = container.layout()
            self.main_camera_view = CameraView("Preview")
            layout.addWidget(self.main_camera_view)

    def setup_camera_stream(self):
        self.camera_stream = None
        self.camera_views = []

        if hasattr(self, "camera_tab_view"):
            self.camera_views.append(self.camera_tab_view)
        if self.main_camera_view is not None:
            self.camera_views.append(self.main_camera_view)

        if not self.camera_views:
            return

        self.camera_stream = CameraStreamThread(CAMERA_STREAM_URL, self)
        self.camera_stream.frame_ready.connect(self.handle_camera_frame)
        self.camera_stream.status_changed.connect(self.handle_camera_status)
        self.camera_stream.start()

    def handle_camera_frame(self, image):
        for view in self.camera_views:
            view.set_frame(image)

    def handle_camera_status(self, status):
        if hasattr(self, "camera_status_label"):
            self.camera_status_label.setText(status)
        for view in self.camera_views:
            if not view.has_frame():
                view.setText(status)

    def stop_camera_stream(self):
        if getattr(self, "camera_stream", None) is not None:
            self.camera_stream.stop()
            self.camera_stream.wait(1500)
