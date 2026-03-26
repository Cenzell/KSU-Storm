import os
import sys
import time
import logging
import math
from pathlib import Path
import pygame
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QIcon
from PyQt6 import uic

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parents[1]
LIB_DIR = PROJECT_ROOT / "lib"
UI_DIR = BASE_DIR / "ui"
UI_FILE = UI_DIR / "driver_station.ui"

for path in (LIB_DIR, UI_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

import comm
from driver_ui import DriverUIHelpers

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Constants
GAMEPAD_POLL_RATE_MS = 20
JOYSTICK_THRESHOLD = 0.01  # Minimum change to send update
MAX_LINEAR_SPEED_MPS = 1.2
MAX_ANGULAR_SPEED_DPS = 180.0
EXPECTED_POSE_HORIZON_S = 0.35
SLOW_DRIVE_SCALE = 0.2
AXIS_DEADZONE = 0.03

FACE_BUTTON_COLORS = {
    0: "green",   # A
    1: "red",     # B
    2: "blue",    # X
    3: "purple",  # Y
}


class AppWindow(DriverUIHelpers, QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(str(UI_FILE), self)
        self.setup_tabs()

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
        self.current_pose = {"x": 0.0, "y": 0.0, "theta_deg": 0.0}
        self.expected_pose = self.current_pose.copy()

        # Add field view to odometry panel
        self.setup_field_view()
        self.setup_main_camera_view()
        self.setup_camera_stream()
        self.current_pose = {
            "x": self.field_widget.field_width_m / 2.0,
            "y": self.field_widget.field_height_m / 2.0,
            "theta_deg": 0.0
        }
        self.expected_pose = self.current_pose.copy()
        
        # Keyboard control state
        self.keyboard_enabled = True
        self.keys_pressed = set()
        self.keyboard_speed = 0.7  # Default keyboard speed (0.0 to 1.0)
        
        # Connect mode buttons
        self.btn_auto.clicked.connect(self.set_auto_mode)
        self.btn_teleop.clicked.connect(self.set_teleop_mode)
        self.btn_rst.clicked.connect(self.reset_robot)
        if hasattr(self, 'pushButton'):
            self.pushButton.clicked.connect(self.reset_odometry)
        if hasattr(self, 'btn_odo_optical'):
            self.btn_odo_optical.clicked.connect(lambda: self.set_odometry_mode("OPTICAL"))
        if hasattr(self, 'btn_odo_motor'):
            self.btn_odo_motor.clicked.connect(lambda: self.set_odometry_mode("MOTOR"))
        if hasattr(self, 'btn_odo_hybrid'):
            self.btn_odo_hybrid.clicked.connect(lambda: self.set_odometry_mode("HYBRID"))
        
        # Setup keyboard speed slider if it exists in UI
        if hasattr(self, 'keyboard_speed_slider'):
            self.keyboard_speed_slider.setMinimum(0)
            self.keyboard_speed_slider.setMaximum(100)
            self.keyboard_speed_slider.setValue(100)
            self.keyboard_speed_slider.valueChanged.connect(self.update_keyboard_speed)
            self.keyboard_speed_label.setText(f"Keyboard Speed: {self.keyboard_speed:.0%}")
        
        logger.info("Driver station initialized")
        logger.info("Keyboard controls: WASD=move, QE=rotate, Shift=speed boost, Space=stop")

    def update_odometry_labels(self, x_m, y_m, theta_deg):
        if hasattr(self, 'label_3'):
            self.label_3.setText(f"X: {x_m:.2f} m")
        if hasattr(self, 'label_2'):
            self.label_2.setText(f"Y: {y_m:.2f} m")
        if hasattr(self, 'label_4'):
            self.label_4.setText(f"Theta: {theta_deg:.1f} deg")

    def update_expected_pose(self):
        """Project a short-horizon expected pose from current command inputs."""
        base_x = float(self.current_pose["x"])
        base_y = float(self.current_pose["y"])
        base_theta_deg = float(self.current_pose["theta_deg"])

        if self.current_mode != "TELEOP":
            self.expected_pose = {"x": base_x, "y": base_y, "theta_deg": base_theta_deg}
            self.field_widget.set_expected_pose(base_x, base_y, base_theta_deg)
            return

        lx = float(self.joystick_values.get("lx", 0.0))
        ly = float(self.joystick_values.get("ly", 0.0))
        rx = float(self.joystick_values.get("rx", 0.0))

        v_forward = ly * MAX_LINEAR_SPEED_MPS
        v_strafe = lx * MAX_LINEAR_SPEED_MPS
        omega_deg = rx * MAX_ANGULAR_SPEED_DPS

        theta_rad = math.radians(base_theta_deg)
        v_field_x = (v_forward * math.cos(theta_rad)) - (v_strafe * math.sin(theta_rad))
        v_field_y = (v_forward * math.sin(theta_rad)) + (v_strafe * math.cos(theta_rad))

        expected_x = base_x + (v_field_x * EXPECTED_POSE_HORIZON_S)
        expected_y = base_y + (v_field_y * EXPECTED_POSE_HORIZON_S)
        expected_theta_deg = (base_theta_deg + (omega_deg * EXPECTED_POSE_HORIZON_S)) % 360.0

        expected_x = max(0.0, min(self.field_widget.field_width_m, expected_x))
        expected_y = max(0.0, min(self.field_widget.field_height_m, expected_y))

        self.expected_pose = {"x": expected_x, "y": expected_y, "theta_deg": expected_theta_deg}
        self.field_widget.set_expected_pose(expected_x, expected_y, expected_theta_deg)

    def set_odometry_mode(self, mode):
        """Set the odometry source mode on the robot."""
        client = self.conn_manager.get_client()
        if client:
            response = client.send_command('odometry_mode', mode=mode)
            if response and response.get('status') == 'success':
                if hasattr(self, 'label_odo_mode'):
                    self.label_odo_mode.setText(f"Odometry Mode: {mode.title()}")
            else:
                logger.warning(f"Failed to set odometry mode: {mode}")

    def reset_odometry(self):
        """Reset odometry pose on robot and local field widget."""
        client = self.conn_manager.get_client()
        if client:
            response = client.send_command('reset_odometry')
            if response and response.get('status') == 'success':
                logger.info("Odometry reset requested")
        center_x = self.field_widget.field_width_m / 2.0
        center_y = self.field_widget.field_height_m / 2.0
        self.current_pose = {"x": center_x, "y": center_y, "theta_deg": 0.0}
        self.expected_pose = self.current_pose.copy()
        self.field_widget.set_pose(center_x, center_y, 0.0)
        self.field_widget.set_expected_pose(center_x, center_y, 0.0)
        self.update_odometry_labels(center_x, center_y, 0.0)
    
    def set_auto_mode(self):
        """Switch robot to autonomous mode."""
        if self._set_robot_mode("AUTO"):
            self.start_match_timer()
            logger.info("Switched to AUTO mode")
    
    def set_teleop_mode(self):
        """Switch robot to teleoperated mode (manual start)."""
        if self._set_robot_mode("TELEOP"):
            # Reset timer when manually starting teleop
            self.match_time_seconds = 0
            if not self.match_running:
                self.start_match_timer()
            logger.info("Switched to TELEOP mode (manual)")
    
    def auto_switch_to_teleop(self):
        """Automatically switch from AUTO to TELEOP after 30 seconds."""
        if self._set_robot_mode("TELEOP"):
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

    def _set_robot_mode(self, mode):
        mode = str(mode).upper()
        client = self.conn_manager.get_client()
        if not client:
            return False

        response = client.set_mode(mode)
        if not response or response.get("status") != "success":
            logger.warning(f"Failed to set mode: {mode}")
            return False

        self.current_mode = mode
        if mode == "AUTO":
            self.robot_status.setText("Autonomous")
        elif mode == "TELEOP":
            self.robot_status.setText("Teleoperated")
        else:
            self.robot_status.setText("Stopped")
        return True

    def _set_control_mode_label(self, mode_name, color=None):
        if not hasattr(self, "control_mode_label"):
            return
        if color is None:
            self.control_mode_label.setText(f"Control: {mode_name}")
        else:
            self.control_mode_label.setText(f"Control: <b style='color: {color};'>{mode_name}</b>")

    def _set_face_button_style(self, button_index, active):
        labels = {
            0: self.button_a_label,
            1: self.button_b_label,
            2: self.button_x_label,
            3: self.button_y_label,
        }
        label = labels.get(button_index)
        if label is None:
            return
        label.setStyleSheet(f"color: {FACE_BUTTON_COLORS[button_index] if active else 'lightgray'}")

    def _scaled_axes(self, lx, ly, rx, ry):
        if self.slow_drive.isChecked():
            return (
                lx * SLOW_DRIVE_SCALE,
                ly * SLOW_DRIVE_SCALE,
                rx * SLOW_DRIVE_SCALE,
                ry * SLOW_DRIVE_SCALE,
            )
        return lx, ly, rx, ry
    
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
        try:
            field = data.get('field', {})
            pose = data.get('pose', {})
            odometry_mode = data.get('odometry_mode')

            width_m = float(field.get('width_m', self.field_widget.field_width_m))
            height_m = float(field.get('height_m', self.field_widget.field_height_m))
            x_m = float(pose.get('x', self.current_pose["x"]))
            y_m = float(pose.get('y', self.current_pose["y"]))
            theta_deg = float(pose.get('theta_deg', self.current_pose["theta_deg"]))

            self.field_widget.set_field_size(width_m, height_m)
            self.field_widget.set_pose(x_m, y_m, theta_deg)
            self.update_odometry_labels(x_m, y_m, theta_deg)
            self.current_pose = {"x": x_m, "y": y_m, "theta_deg": theta_deg}
            self.update_expected_pose()

            if odometry_mode and hasattr(self, 'label_odo_mode'):
                self.label_odo_mode.setText(f"Odometry Mode: {str(odometry_mode).title()}")
        except Exception as e:
            logger.error(f"Error parsing telemetry pose: {e}")

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
        lx = 0.0  # Left/right strafe
        ly = 0.0  # Forward/backward
        rx = 0.0  # Rotation
        
        # Base speed (can be boosted with Shift)
        speed = self.keyboard_speed
        if Qt.Key.Key_Shift in self.keys_pressed:
            speed = 1.0  # Full speed with shift

        if self.slow_drive.isChecked():
            speed *= SLOW_DRIVE_SCALE
        
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
                self._set_control_mode_label("Keyboard", color="blue")
            elif self.joystick is not None:
                self._set_control_mode_label("Gamepad", color="green")
            else:
                self._set_control_mode_label("None")
            
            # Use keyboard input if active, otherwise use joystick
            if has_keyboard_input:
                self.joystick_values['lx'] = keyboard_input[0]
                self.joystick_values['ly'] = keyboard_input[1]
                self.joystick_values['rx'] = keyboard_input[2]
                self.joystick_values['ry'] = keyboard_input[3]
            elif self.joystick is not None:
                # Poll joystick only if no keyboard input
                pygame.event.pump()
                # Read and apply deadzone to joystick axes
                axis_lx = self.joystick.get_axis(0)
                axis_ly = self.joystick.get_axis(1)
                axis_rx = self.joystick.get_axis(2)
                axis_ry = self.joystick.get_axis(4)

                self.joystick_values['lx'] = axis_lx if abs(axis_lx) > AXIS_DEADZONE else 0.0
                self.joystick_values['ly'] = -axis_ly if abs(axis_ly) > AXIS_DEADZONE else 0.0
                self.joystick_values['rx'] = axis_rx if abs(axis_rx) > AXIS_DEADZONE else 0.0
                self.joystick_values['ry'] = -axis_ry if abs(axis_ry) > AXIS_DEADZONE else 0.0

                # Handle button events
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        client.send_button(event.button, "DOWN")
                        if event.button in FACE_BUTTON_COLORS:
                            self._set_face_button_style(event.button, active=True)
                            
                    elif event.type == pygame.JOYBUTTONUP:
                        client.send_button(event.button, "UP")
                        if event.button in FACE_BUTTON_COLORS:
                            self._set_face_button_style(event.button, active=False)
            else:
                # No input - zero everything
                self.joystick_values = {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0}

            lx, ly, rx, ry = self._scaled_axes(
                self.joystick_values['lx'],
                self.joystick_values['ly'],
                self.joystick_values['rx'],
                self.joystick_values['ry'],
            )
            self.joystick_values['lx'] = lx
            self.joystick_values['ly'] = ly
            self.joystick_values['rx'] = rx
            self.joystick_values['ry'] = ry

            # Update UI labels
            self.lx_label.setText(f"LX: {self.joystick_values['lx']:.2f}")
            self.ly_label.setText(f"LY: {self.joystick_values['ly']:.2f}")
            self.rx_label.setText(f"RX: {self.joystick_values['rx']:.2f}")
            self.ry_label.setText(f"RY: {self.joystick_values['ry']:.2f}")
            self.update_expected_pose()

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
            self.stop_camera_stream()

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
    icon_path = os.path.join(os.path.dirname(__file__), "app_icon.png")
    if os.path.exists(icon_path):
        app.setWindowIcon(QIcon(icon_path))
    window = AppWindow()
    if os.path.exists(icon_path):
        window.setWindowIcon(QIcon(icon_path))
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
