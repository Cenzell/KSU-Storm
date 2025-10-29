import sys
import socket
import time
import logging
import pygame
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer
from PyQt6 import uic
import comm

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Constants
GAMEPAD_POLL_RATE_MS = 20
PING_INTERVAL_S = 1
JOYSTICK_THRESHOLD = 0.01  # Minimum change to send update

    
class AppWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("driver_station.ui", self)

        self.joystick = None
        self.init_pygame_and_joystick()

        # Connection manager
        self.conn_manager = comm.ConnectionManager()
        self.conn_manager.signals.connection_status.connect(self.update_connection_status)
        self.conn_manager.start()

        # Telemetry receiver
        self.telemetry_receiver = comm.TelemetryReceiver(self.conn_manager)
        self.telemetry_receiver.signals.ping_response.connect(self.handle_ping_response)
        self.telemetry_receiver.start()
        
        # Gamepad polling timer
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.poll_gamepad)
        self.gamepad_timer.start(GAMEPAD_POLL_RATE_MS)

        # State tracking
        self.last_ping_time = 0
        self.ping_sent_time = None
        self.joystick_values = {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0}
        self.last_sent_joystick_values = self.joystick_values.copy()
        self.current_mode = "STOPPED"
        
        # Connect mode buttons
        self.btn_auto.clicked.connect(self.set_auto_mode)
        self.btn_teleop.clicked.connect(self.set_teleop_mode)
        self.btn_rst.clicked.connect(self.reset_robot)
        
        logger.info("Driver station initialized")
    
    def set_auto_mode(self):
        """Switch robot to autonomous mode."""
        self.send_to_robot("MODE AUTO\n")
        self.current_mode = "AUTO"
        self.robot_status.setText("Autonomous")
        logger.info("Switched to AUTO mode")
    
    def set_teleop_mode(self):
        """Switch robot to teleoperated mode."""
        self.send_to_robot("MODE TELEOP\n")
        self.current_mode = "TELEOP"
        self.robot_status.setText("Teleoperated")
        logger.info("Switched to TELEOP mode")
    
    def reset_robot(self):
        """Reset robot to stopped state."""
        self.send_to_robot("RESET\n")
        self.current_mode = "STOPPED"
        self.robot_status.setText("Stopped")
        logger.info("Robot reset")
    
    def handle_ping_response(self):
        """Handle ping response from robot."""
        if self.ping_sent_time:
            rtt_ms = (time.time() - self.ping_sent_time) * 1000
            self.ping_label.setText(f"Ping: {rtt_ms:.1f} ms")
            self.ping_sent_time = None

    def init_pygame_and_joystick(self):
        """Initialize pygame and detect joystick."""
        try:
            pygame.init()
            pygame.joystick.init()
            
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.gamepad_label.setText(f"Gamepad: {self.joystick.get_name()}")
                logger.info(f"Found joystick: {self.joystick.get_name()}")
            else:
                self.gamepad_label.setText("Gamepad: Not Found")
                logger.warning("No joystick found")
        except Exception as e:
            logger.error(f"Error initializing pygame/joystick: {e}")
            self.gamepad_label.setText("Gamepad: Error")

    def update_connection_status(self, is_connected, address):
        """Update UI based on connection status."""
        if is_connected:
            self.status_label.setText("Status: <b style='color: green;'>Connected</b>")
            self.address_label.setText(f"Address: {address}")
            logger.info(f"Connected to {address}")
        else:
            self.status_label.setText("Status: <b style='color: red;'>Disconnected</b>")
            self.address_label.setText("Address: N/A")
            self.ping_label.setText("Ping: -- ms")
            self.robot_status.setText("Stopped")
            self.current_mode = "STOPPED"
            
            # Reset button colors
            self.button_a_label.setStyleSheet("color: lightgray")
            self.button_b_label.setStyleSheet("color: lightgray")
            self.button_x_label.setStyleSheet("color: lightgray")
            self.button_y_label.setStyleSheet("color: lightgray")
            
            logger.warning("Disconnected from robot")

    def send_to_robot(self, message):
        """Send a message to the robot."""
        sock = self.conn_manager.get_socket()
        if sock:
            try:
                sock.sendall(message.encode('utf-8'))
            except (socket.error, OSError) as e:
                logger.error(f"Send error: {e}. Disconnecting.")
                self.conn_manager.disconnect()

    def values_changed_significantly(self, old_values, new_values, threshold=JOYSTICK_THRESHOLD):
        """Check if joystick values changed beyond threshold."""
        return any(abs(old_values[k] - new_values[k]) > threshold for k in old_values)

    def poll_gamepad(self):
        """Poll gamepad state and send updates to robot."""
        if self.joystick is None:
            return

        try:
            pygame.event.pump()
            deadzone = 0.03
            
            # Read and apply deadzone to joystick axes
            self.joystick_values['lx'] = self.joystick.get_axis(0) if abs(self.joystick.get_axis(0)) > deadzone else 0.0
            self.joystick_values['ly'] = -self.joystick.get_axis(1) if abs(self.joystick.get_axis(1)) > deadzone else 0.0
            self.joystick_values['rx'] = self.joystick.get_axis(2) if abs(self.joystick.get_axis(2)) > deadzone else 0.0
            self.joystick_values['ry'] = -self.joystick.get_axis(4) if abs(self.joystick.get_axis(4)) > deadzone else 0.0

            # Handle button events
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    self.send_to_robot(f"BTN {event.button} DOWN\n")
                    if event.button == 0: 
                        self.button_a_label.setStyleSheet("color: green")
                    elif event.button == 1: 
                        self.button_b_label.setStyleSheet("color: red")
                    elif event.button == 2: 
                        self.button_x_label.setStyleSheet("color: blue")
                    elif event.button == 3: 
                        self.button_y_label.setStyleSheet("color: purple")
                        
                elif event.type == pygame.JOYBUTTONUP:
                    self.send_to_robot(f"BTN {event.button} UP\n")
                    if event.button in [0, 1, 2, 3]:
                        label = [self.button_a_label, self.button_b_label, 
                                self.button_x_label, self.button_y_label][event.button]
                        label.setStyleSheet("color: lightgray")

            # Update UI labels
            self.lx_label.setText(f"LX: {self.joystick_values['lx']:.2f}")
            self.ly_label.setText(f"LY: {self.joystick_values['ly']:.2f}")
            self.rx_label.setText(f"RX: {self.joystick_values['rx']:.2f}")
            self.ry_label.setText(f"RY: {self.joystick_values['ry']:.2f}")

            # Send joystick values if changed significantly
            if self.values_changed_significantly(self.last_sent_joystick_values, self.joystick_values):
                msg = (f"JOYSTICKS {self.joystick_values['lx']:.3f},{self.joystick_values['ly']:.3f},"
                       f"{self.joystick_values['rx']:.3f},{self.joystick_values['ry']:.3f}\n")
                self.send_to_robot(msg)
                self.last_sent_joystick_values = self.joystick_values.copy()

            # Send periodic ping
            if time.time() - self.last_ping_time > PING_INTERVAL_S:
                self.ping_sent_time = time.time()
                self.send_to_robot("PING\n")
                self.last_ping_time = time.time()
                
        except Exception as e:
            logger.error(f"Error polling gamepad: {e}")

    def closeEvent(self, event):
        """Clean up resources on application close."""
        logger.info("Closing application...")
        
        try:
            # Stop threads
            self.telemetry_receiver.stop()
            self.conn_manager.stop()
            
            # Wait for threads to finish (with timeout)
            self.telemetry_receiver.join(timeout=2)
            self.conn_manager.join(timeout=2)
            
            # Clean up joystick
            if self.joystick:
                self.joystick.quit()
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
        finally:
            pygame.quit()
            event.accept()
            logger.info("Application closed")


def main():
    app = QApplication(sys.argv)
    window = AppWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
