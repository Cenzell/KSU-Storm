import sys
import socket
import threading
import time
import pygame
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QGridLayout
from PyQt6.QtCore import QTimer, QObject, pyqtSignal, Qt

# --- Configuration ---
ROBOT_ADDRESSES = [
    "wildrobo.local:5000",
    "10.42.0.85:5000",   # Example static IP
    "127.0.0.1:5000",    # For local testing
]
GAMEPAD_POLL_RATE_MS = 20  # How often to check for joystick events (50 Hz)
PING_INTERVAL_S = 1        # How often to send a heartbeat to the robot

class WorkerSignals(QObject):
    """Defines signals available from a running worker thread."""
    connection_status = pyqtSignal(bool, str)
    # CHANGED: Added a signal for when a ping response is received
    ping_response = pyqtSignal()

# CHANGED: NEW CLASS to listen for data from the robot
class TelemetryReceiver(threading.Thread):
    """Reads incoming data from the robot in a background thread."""
    def __init__(self, conn_manager):
        super().__init__()
        self.signals = WorkerSignals()
        self.conn_manager = conn_manager
        self.running = True

    def run(self):
        buffer = ""
        while self.running:
            sock = self.conn_manager.get_socket()
            if sock:
                try:
                    data = sock.recv(1024)
                    if not data:
                        # Socket was closed by the other end
                        time.sleep(0.5)
                        continue
                    
                    buffer += data.decode('utf-8')
                    while '\n' in buffer:
                        message, buffer = buffer.split('\n', 1)
                        if message == "ACK":
                            self.signals.ping_response.emit()
                        # You could handle other telemetry messages here
                except (socket.error, OSError):
                    # An error likely means the connection is dead
                    time.sleep(0.5)
            else:
                # Not connected, wait a bit before checking again
                time.sleep(1)

    def stop(self):
        self.running = False


class ConnectionManager(threading.Thread):
    """Manages the TCP connection in a background thread."""
    def __init__(self):
        super().__init__()
        self.signals = WorkerSignals()
        self.sock = None
        self.lock = threading.Lock()
        self.running = True

    def run(self):
        print("Starting connection manager...")
        address_idx = 0
        while self.running:
            with self.lock:
                if self.sock is None:
                    address_str = ROBOT_ADDRESSES[address_idx]
                    host, port_str = address_str.split(':')
                    port = int(port_str)
                    print(f"Trying to connect to: {host}:{port}")
                    try:
                        new_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        new_sock.settimeout(2.0)
                        new_sock.connect((host, port))
                        new_sock.settimeout(None)
                        self.sock = new_sock
                        print(f"✅ Connected to {address_str}")
                        self.signals.connection_status.emit(True, address_str)
                    except (socket.error, OSError) as e:
                        self.signals.connection_status.emit(False, "")
                        address_idx = (address_idx + 1) % len(ROBOT_ADDRESSES)
            time.sleep(2)

    def get_socket(self):
        with self.lock:
            return self.sock

    def disconnect(self):
        with self.lock:
            if self.sock:
                self.sock.close()
                self.sock = None
                self.signals.connection_status.emit(False, "")
    
    def stop(self):
        self.running = False
        self.disconnect()

    
class AppWindow(QMainWindow):
    """Main application window."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Driver Station")
        self.setGeometry(100, 100, 400, 200)

        # --- UI Elements ---
        self.status_label = QLabel("Status: Disconnected")
        self.address_label = QLabel("Address: N/A")
        self.gamepad_label = QLabel("Gamepad: Not Found")
        self.ping_label = QLabel("Ping: -- ms")
        self.lx_label = QLabel("LX: 0.00")
        self.ly_label = QLabel("LY: 0.00")
        self.rx_label = QLabel("RX: 0.00")
        self.ry_label = QLabel("RY: 0.00")
        
        # --- Layout ---
        layout = QGridLayout()
        layout.addWidget(self.status_label, 0, 0)
        layout.addWidget(self.address_label, 0, 1)
        layout.addWidget(self.gamepad_label, 1, 0, 1, 2)
        layout.addWidget(self.ping_label, 2, 0, 1, 2)
        layout.addWidget(self.lx_label, 3, 0)
        layout.addWidget(self.ly_label, 3, 1)
        layout.addWidget(self.rx_label, 4, 0)
        layout.addWidget(self.ry_label, 4, 1)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # --- Gamepad / Joystick Setup ---
        self.joystick = None
        self.init_pygame_and_joystick()

        # --- Networking ---
        self.conn_manager = ConnectionManager()
        self.conn_manager.signals.connection_status.connect(self.update_connection_status)
        self.conn_manager.start()
        
        # --- Timers ---
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.poll_gamepad)
        self.gamepad_timer.start(GAMEPAD_POLL_RATE_MS)
        
        # --- State Variables ---
        self.joystick_values = {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0}
        self.last_sent_joystick_values = self.joystick_values.copy()

    def init_pygame_and_joystick(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.gamepad_label.setText(f"Gamepad: {self.joystick.get_name()}")
        else:
            self.gamepad_label.setText("Gamepad: Not Found")

    def update_connection_status(self, is_connected, address):
        if is_connected:
            self.status_label.setText("Status: <b style='color: green;'>Connected</b>")
            self.address_label.setText(f"Address: {address}")
        else:
            self.status_label.setText("Status: <b style='color: red;'>Disconnected</b>")
            self.address_label.setText("Address: N/A")

    def send_to_robot(self, message):
        sock = self.conn_manager.get_socket()
        if sock:
            try:
                sock.sendall(message.encode('utf-8'))
            except (socket.error, OSError) as e:
                print(f"Send error: {e}. Disconnecting.")
                self.conn_manager.disconnect()

    def poll_gamepad(self):
        if self.joystick is None:
            return

        pygame.event.pump()
        deadzone = 0.03
        self.joystick_values['lx'] = self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > deadzone else 0.0
        self.joystick_values['ly'] = -self.joystick.get_axis(1) if abs(self.joystick.get_axis(1)) > deadzone else 0.0
        self.joystick_values['rx'] = self.joystick.get_axis(2) if abs(self.joystick.get_axis(2)) > deadzone else 0.0
        self.joystick_values['ry'] = -self.joystick.get_axis(4) if abs(self.joystick.get_axis(4)) > deadzone else 0.0

        for event in pygame.event.get():
            # Process Axis Motion
            if event.type == pygame.JOYAXISMOTION:
                event_occurred = True

                if event.axis == 0:
                    self.joystick_values['lx'] = event.value if abs(event.value) > 0.03 else 0.0

                elif event.axis == 1:
                    self.joystick_values['ly'] = -event.value if abs(event.value) > 0.03 else 0.0  # invert Y
            
                elif event.axis in (2, 3):
                    self.joystick_values['rx'] = event.value if abs(event.value) > 0.03 else 0.0
            
                elif event.axis in (4, 5):
                    self.joystick_values['ry'] = -event.value if abs(event.value) > 0.03 else 0.0  # invert Y

            # Process Button Presses
            if event.type == pygame.JOYBUTTONDOWN:
                self.send_to_robot(f"BTN {event.button} DOWN\n")
            if event.type == pygame.JOYBUTTONUP:
                self.send_to_robot(f"BTN {event.button} UP\n")

        self.lx_label.setText(f"LX: {self.joystick_values['lx']:.2f}")
        self.ly_label.setText(f"LY: {self.joystick_values['ly']:.2f}")
        self.rx_label.setText(f"RX: {self.joystick_values['rx']:.2f}")
        self.ry_label.setText(f"RY: {self.joystick_values['ry']:.2f}")

        if self.joystick_values != self.last_sent_joystick_values:
            msg = (
                f"JOYSTICKS {self.joystick_values['lx']},{self.joystick_values['ly']},"
                f"{self.joystick_values['rx']},{self.joystick_values['ry']}\n"
            )
            self.send_to_robot(msg)
            self.last_sent_joystick_values = self.joystick_values.copy()

        if time.time() - self.last_ping_time > PING_INTERVAL_S:
            # CHANGED: Record the time just before sending the ping
            self.ping_sent_time = time.time()
            self.send_to_robot("PING\n")
            self.last_ping_time = time.time()

    def closeEvent(self, event):
        """Ensure threads are stopped when the window is closed."""
        print("Closing application...")
        # CHANGED: Stop the telemetry thread as well
        self.telemetry_receiver.stop()
        self.conn_manager.stop()
        self.telemetry_receiver.join()
        self.conn_manager.join()
        pygame.quit()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = AppWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()