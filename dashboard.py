import sys
import threading
import time
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QTextEdit, QHBoxLayout
from PyQt6.QtCore import QObject, pyqtSignal, QTimer, Qt
from PyQt6.QtGui import QColor, QFont, QPixmap, QPainter
import pygame as pg

class UiSignals(QObject):
    connected = pyqtSignal(bool, str)
    controller_status = pyqtSignal(bool)
    ping = pyqtSignal(float)
    telemetry = pyqtSignal(str)

class Dashboard(QMainWindow):
    def __init__(self, signals):
        super().__init__()
        self.signals = signals
        self.setWindowTitle("Robot Driver Dashboard")
        self.setGeometry(100, 100, 800, 600)
        self.init_ui()
        self.connect_signals()

    def init_ui(self):
        # Main layout and central widget
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)
        self.setCentralWidget(central_widget)

        # Indicators
        status_layout = QHBoxLayout()
        main_layout.addLayout(status_layout)

        # Connection Status
        self.connection_label = QLabel("Connection: Disconnected")
        self.connection_status_led = LedIndicator(False)
        status_layout.addWidget(self.connection_label)
        status_layout.addWidget(self.connection_status_led)

        # Controller Status
        self.controller_label = QLabel("Controller: Disconnected")
        self.controller_status_led = LedIndicator(False)
        status_layout.addWidget(self.controller_label)
        status_layout.addWidget(self.controller_status_led)
        
        # Ping
        self.ping_label = QLabel("Ping: - ms")
        main_layout.addWidget(self.ping_label)

        # Log
        self.telemetry_log = QTextEdit()
        self.telemetry_log.setReadOnly(True)
        main_layout.addWidget(self.telemetry_log)

    def connect_signals(self):
        self.signals.connected.connect(self.update_connection_status)
        self.signals.controller_status.connect(self.update_controller_status)
        self.signals.ping.connect(self.update_ping)
        self.signals.telemetry.connect(self.append_telemetry_log)

    def update_connection_status(self, is_connected, ip_address):
        if is_connected:
            self.connection_label.setText(f"Connection: Connected to {ip_address}")
            self.connection_status_led.set_color(True)
        else:
            self.connection_label.setText("Connection: Disconnected")
            self.connection_status_led.set_color(False)
            self.ping_label.setText("Ping: - ms")

    def update_controller_status(self, is_connected):
        if is_connected:
            self.controller_label.setText("Controller: Connected")
            self.controller_status_led.set_color(True)
        else:
            self.controller_label.setText("Controller: Disconnected")
            self.controller_status_led.set_color(False)
    
    def update_ping(self, rtt):
        self.ping_label.setText(f"Ping: {rtt:.2f} ms")

    def append_telemetry_log(self, text):
        self.telemetry_log.append(text)

# LED Widget
class LedIndicator(QLabel):
    def __init__(self, is_on=False, parent=None):
        super().__init__(parent)
        self.is_on = is_on
        self.setFixedSize(20, 20)

    def set_color(self, is_on):
        self.is_on = is_on
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        if self.is_on:
            color = QColor(0, 255, 0)
        else:
            color = QColor(255, 0, 0)
        painter.setBrush(color)
        painter.drawEllipse(0, 0, 20, 20)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    ui_signals = UiSignals()
    dashboard = Dashboard(ui_signals)
    dashboard.show()

    class MockDataThread(threading.Thread):
        def __init__(self, signals):
            super().__init__()
            self.signals = signals
            self.is_connected = False
            self.controller_connected = False
            self.running = True

        def run(self):
            time.sleep(2)
            self.is_connected = True
            self.signals.connected.emit(True, "10.42.0.85:5000")
            
            time.sleep(1)
            self.controller_connected = True
            self.signals.controller_status.emit(True)

            while self.running:
                if self.is_connected:
                    self.signals.ping.emit(time.time() * 100 % 100) # Fake ping
                    self.signals.telemetry.emit(f"Telemetry from robot at {time.time():.2f}")
                time.sleep(1)
        
        def stop(self):
            self.running = False

    data_thread = MockDataThread(ui_signals)
    data_thread.start()

    sys.exit(app.exec())