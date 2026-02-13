import os
import sys
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
JOYSTICK_THRESHOLD = 0.01  # Minimum change to send update

    
class AppWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("driver_station.ui", self)

        self.joystick = None
        self.init_pygame_and_joystick()

        # Connection manager (ZMQ-based)
        self.conn_manager = comm.ConnectionManager()
        self.conn_manager.signals.connection_status.connect(self.update_connection_status)
        self.conn_manager.start()

        # Telemetry receiver
        self.telemetry_receiver = comm.TelemetryReceiver(self.conn_manager)
        self.telemetry_receiver.start()
        
        # Connect signals
        self.conn_manager.signals.ping_response.connect(self.handle_ping_response)
        self.conn_manager.signals.telemetry_update.connect(self.handle_telemetry)
        
        # Gamepad polling timer
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.poll_gamepad)
        self.gamepad_timer.start(GAMEPAD_POLL_RATE_MS)

        # Match timer
        self.match_timer = QTimer()
        self.match_timer.timeout.connect(self.update_match_time)
        self.match_time_seconds = 0
        self.match_running = False
        self.auto_duration = 30  # 30 seconds for auto
        self.teleop_duration = 210  # 3 minutes 30 seconds (210 seconds) for teleop
        self.match_start_time = 0

        # State tracking
        self.joystick_values = {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0}
        self.last_sent_joystick_values = self.joystick_values.copy()
        self.current_mode = "STOPPED"
        
        # Keyboard control state
        self.keyboard_enabled = True
        self.keys_pressed = set()
        self.keyboard_speed = 0.7  # Default keyboard speed (0.0 to 1.0)
        
        # Connect mode buttons
        self.btn_auto.clicked.connect(self.set_auto_mode)
        self.btn_teleop.clicked.connect(self.set_teleop_mode)
        self.btn_rst.clicked.connect(self.reset_robot)
        
        # Setup keyboard speed slider if it exists in UI
        if hasattr(self, 'keyboard_speed_slider'):
            self.keyboard_speed_slider.setMinimum(0)
            self.keyboard_speed_slider.setMaximum(100)
            self.keyboard_speed_slider.setValue(100)
            self.keyboard_speed_slider.valueChanged.connect(self.update_keyboard_speed)
            self.keyboard_speed_label.setText(f"Keyboard Speed: {self.keyboard_speed:.0%}")
        
        logger.info("Driver station initialized")
        logger.info("Keyboard controls: WASD=move, QE=rotate, Shift=speed boost, Space=stop")
    
    def set_auto_mode(self):
        """Switch robot to autonomous mode."""
        client = self.conn_manager.get_client()
        if client:
            client.set_mode("AUTO")
            self.current_mode = "AUTO"
            self.robot_status.setText("Autonomous")
            self.start_match_timer()
            logger.info("Switched to AUTO mode")
    
    def set_teleop_mode(self):
        """Switch robot to teleoperated mode (manual start)."""
        client = self.conn_manager.get_client()
        if client:
            client.set_mode("TELEOP")
            self.current_mode = "TELEOP"
            self.robot_status.setText("Teleoperated")
            # Reset timer when manually starting teleop
            self.match_time_seconds = 0
            if not self.match_running:
                self.start_match_timer()
            logger.info("Switched to TELEOP mode (manual)")
    
    def auto_switch_to_teleop(self):
        """Automatically switch from AUTO to TELEOP after 30 seconds."""
        client = self.conn_manager.get_client()
        if client:
            client.set_mode("TELEOP")
            self.current_mode = "TELEOP"
            self.robot_status.setText("Teleoperated")
            # Reset timer for teleop phase
            self.match_time_seconds = 0
            logger.info("Auto-switched from AUTO to TELEOP at 30 seconds")
    
    def reset_robot(self):
        """Reset robot to stopped state."""
        client = self.conn_manager.get_client()
        if client:
            client.reset_robot()
            self.current_mode = "STOPPED"
            self.robot_status.setText("Stopped")
            self.stop_match_timer()
            logger.info("Robot reset")
    
    def start_match_timer(self):
        """Start the match timer."""
        self.match_time_seconds = 0
        self.match_running = True
        self.match_timer.start(1000)  # Update every second
        self.update_match_time()
        logger.info("Match timer started")
    
    def stop_match_timer(self):
        """Stop the match timer."""
        self.match_running = False
        self.match_timer.stop()
        self.match_time_seconds = 0
        if hasattr(self, 'timer'):
            self.timer.setText("Time")
        logger.info("Match timer stopped")
    
    def update_match_time(self):
        """Update the match timer display."""
        if not self.match_running:
            return
        
        minutes = self.match_time_seconds // 60
        seconds = self.match_time_seconds % 60
        
        # AUTO phase logic (30 seconds)
        if self.current_mode == "AUTO":
            if self.match_time_seconds >= self.auto_duration:
                # Auto switch to teleop
                self.auto_switch_to_teleop()
                return
            
            time_str = f"Time: {minutes}:{seconds:02d} (Auto)"
        
        # TELEOP phase logic (3:30 = 210 seconds)
        elif self.current_mode == "TELEOP":
            if self.match_time_seconds >= self.teleop_duration:
                # Switch to overtime
                overtime_seconds = self.match_time_seconds - self.teleop_duration
                overtime_minutes = overtime_seconds // 60
                overtime_secs = overtime_seconds % 60
                time_str = f"Time: OVERTIME +{overtime_minutes}:{overtime_secs:02d}"
                
                # Optional: Change status color to indicate overtime
                self.robot_status.setStyleSheet("color: red; font-weight: bold;")
            else:
                time_str = f"Time: {minutes}:{seconds:02d} (Teleop)"

                

        # STOPPED or other modes
        else:
            time_str = f"Time: {minutes}:{seconds:02d}"
        
        if hasattr(self, 'timer'):
            self.timer.setText(time_str)
        
        self.match_time_seconds += 1
    
    def handle_ping_response(self, ping_ms):
        """Handle ping response from robot."""
        self.ping_label.setText(f"Ping: {ping_ms:.1f} ms")
    
    def handle_telemetry(self, data):
        """Handle telemetry data from robot."""
        # Update UI with telemetry data
        # Example: battery, sensor readings, motor status, etc.
        logger.debug(f"Telemetry: {data}")
    
    def update_keyboard_speed(self, value):
        """Update keyboard speed from slider."""
        self.keyboard_speed = value / 100.0
        if hasattr(self, 'keyboard_speed_label'):
            self.keyboard_speed_label.setText(f"Keyboard Speed: {self.keyboard_speed:.0%}")
        logger.debug(f"Keyboard speed set to {self.keyboard_speed:.0%}")
    
    def keyPressEvent(self, event):
        """Handle keyboard key press events."""
        if not self.keyboard_enabled:
            return
        
        key = event.key()
        
        # Add key to pressed set
        self.keys_pressed.add(key)
        
        # Don't process if auto-repeat
        if event.isAutoRepeat():
            return
        
        from PyQt6.QtCore import Qt
        
        # Log key presses for debugging
        key_names = {
            Qt.Key.Key_W: "W", Qt.Key.Key_A: "A", 
            Qt.Key.Key_S: "S", Qt.Key.Key_D: "D",
            Qt.Key.Key_Q: "Q", Qt.Key.Key_E: "E",
            Qt.Key.Key_Space: "Space", Qt.Key.Key_Shift: "Shift"
        }
        
        if key in key_names:
            logger.debug(f"Key pressed: {key_names[key]}")
    
    def keyReleaseEvent(self, event):
        """Handle keyboard key release events."""
        if not self.keyboard_enabled:
            return
        
        key = event.key()
        
        # Remove key from pressed set
        self.keys_pressed.discard(key)
        
        # Don't process if auto-repeat
        if event.isAutoRepeat():
            return
    
    def calculate_keyboard_input(self):
        """Calculate joystick values from keyboard input."""
        from PyQt6.QtCore import Qt
        
        lx = 0.0  # Left/right strafe
        ly = 0.0  # Forward/backward
        rx = 0.0  # Rotation
        
        # Base speed (can be boosted with Shift)
        speed = self.keyboard_speed
        if Qt.Key.Key_Shift in self.keys_pressed:
            speed = 1.0  # Full speed with shift

        if self.slow_drive.isChecked():
            speed *= 0.2
        
        # Movement keys
        if Qt.Key.Key_W in self.keys_pressed:
            ly += speed
        if Qt.Key.Key_S in self.keys_pressed:
            ly -= speed
        if Qt.Key.Key_A in self.keys_pressed:
            lx -= speed
        if Qt.Key.Key_D in self.keys_pressed:
            lx += speed
        
        # Rotation keys
        if Qt.Key.Key_Q in self.keys_pressed:
            rx -= speed
        if Qt.Key.Key_E in self.keys_pressed:
            rx += speed
        
        # Emergency stop
        if Qt.Key.Key_Space in self.keys_pressed:
            lx = ly = rx = 0.0
        
        return lx, ly, rx, 0.0  # ry not used for keyboard

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

    def values_changed_significantly(self, old_values, new_values, threshold=JOYSTICK_THRESHOLD):
        """Check if joystick values changed beyond threshold."""
        return any(abs(old_values[k] - new_values[k]) > threshold for k in old_values)

    def poll_gamepad(self):
        """Poll gamepad state and send updates to robot."""
        client = self.conn_manager.get_client()
        if not client:
            return

        try:
            # Check if we have keyboard input
            keyboard_input = self.calculate_keyboard_input()
            has_keyboard_input = any(abs(v) > 0.01 for v in keyboard_input)
            
            # Update control mode indicator
            if has_keyboard_input:
                if hasattr(self, 'control_mode_label'):
                    self.control_mode_label.setText("Control: <b style='color: blue;'>Keyboard</b>")
            elif self.joystick is not None:
                if hasattr(self, 'control_mode_label'):
                    self.control_mode_label.setText("Control: <b style='color: green;'>Gamepad</b>")
            else:
                if hasattr(self, 'control_mode_label'):
                    self.control_mode_label.setText("Control: None")
            
            # Use keyboard input if active, otherwise use joystick
            if has_keyboard_input:
                self.joystick_values['lx'] = keyboard_input[0]
                self.joystick_values['ly'] = keyboard_input[1]
                self.joystick_values['rx'] = keyboard_input[2]
                self.joystick_values['ry'] = keyboard_input[3]
            elif self.joystick is not None:
                # Poll joystick only if no keyboard input
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
                        client.send_button(event.button, "DOWN")
                        if event.button == 0: 
                            self.button_a_label.setStyleSheet("color: green")
                        elif event.button == 1: 
                            self.button_b_label.setStyleSheet("color: red")
                        elif event.button == 2: 
                            self.button_x_label.setStyleSheet("color: blue")
                        elif event.button == 3: 
                            self.button_y_label.setStyleSheet("color: purple")
                            
                    elif event.type == pygame.JOYBUTTONUP:
                        client.send_button(event.button, "UP")
                        if event.button in [0, 1, 2, 3]:
                            label = [self.button_a_label, self.button_b_label, 
                                    self.button_x_label, self.button_y_label][event.button]
                            label.setStyleSheet("color: lightgray")
            else:
                # No input - zero everything
                self.joystick_values = {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0}

            if self.slow_drive.isChecked():
                self.joystick_values['lx'] *= 0.2
                self.joystick_values['ly'] *= 0.2
                self.joystick_values['rx'] *= 0.2
                self.joystick_values['ry'] *= 0.2

            # Update UI labels
            self.lx_label.setText(f"LX: {self.joystick_values['lx']:.2f}")
            self.ly_label.setText(f"LY: {self.joystick_values['ly']:.2f}")
            self.rx_label.setText(f"RX: {self.joystick_values['rx']:.2f}")
            self.ry_label.setText(f"RY: {self.joystick_values['ry']:.2f}")

            # Send joystick values if changed significantly
            if self.values_changed_significantly(self.last_sent_joystick_values, self.joystick_values):
                client.send_joystick(
                    self.joystick_values['lx'],
                    self.joystick_values['ly'],
                    self.joystick_values['rx'],
                    self.joystick_values['ry']
                )
                self.last_sent_joystick_values = self.joystick_values.copy()
                
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
    os.system('cls' if os.name == 'nt' else 'clear')
    app = QApplication(sys.argv)
    window = AppWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()