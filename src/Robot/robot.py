import os
import math
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List

from pathlib import Path
import sys

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parents[1]
LIB_DIR = PROJECT_ROOT / "lib"

lib_str = str(LIB_DIR)
if lib_str not in sys.path:
    sys.path.insert(0, lib_str)

from serial_bridge import SerialBridge

import zmq

try:
    from hardware import PwmMotor
except Exception:
    PwmMotor = None

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Constants
COMMAND_PORT = 5555
TELEMETRY_PORT = 5556
TELEMETRY_RATE_HZ = 10
# Must be greater than driver ping interval (comm.py PING_INTERVAL_S=1s),
# otherwise idle teleop will flap between lost/restored each second.
HEARTBEAT_TIMEOUT_S = 2.5
WATCHDOG_CHECK_INTERVAL_S = 0.1
MAX_LINEAR_SPEED_MPS = 1.2
MAX_ANGULAR_SPEED_DPS = 180.0
FIELD_WIDTH_M = 3.6
FIELD_HEIGHT_M = 3.6
ENABLE_CAMERA_BROADCAST = os.environ.get("KSU_ENABLE_CAMERA_BROADCAST", "1").strip().lower() not in ("0", "false", "no")
USE_PCA9685_PWM = os.environ.get("KSU_PWM_BACKEND", "pi").strip().lower() in ("pca", "pca9685")

# Motor mapping (speed order [FL, FR, RL, RR]).
# Each tuple is (pwm, dir). When using PCA backend, pwm is PCA channel [0..15].
if USE_PCA9685_PWM:
    MOTOR_PIN_MAP = (
        (0, 5),    # Front Left  -> PCA CH0, DIR GPIO5
        (1, 6),    # Front Right -> PCA CH1, DIR GPIO6
        (2, 16),   # Rear Left   -> PCA CH2, DIR GPIO16
        (3, 20),   # Rear Right  -> PCA CH3, DIR GPIO20
    )
else:
    MOTOR_PIN_MAP = (
        (12, 5),   # Front Left
        (13, 6),   # Front Right
        (18, 16),  # Rear Left
        (19, 20),  # Rear Right
    )
# For mirrored left/right drivetrain layouts, right side is commonly inverted.
# Order: [FL, FR, RL, RR]
MOTOR_DIRECTION_MULTIPLIER = (
    float(os.environ.get("KSU_MOTOR_FL_SIGN", "1.0")),
    float(os.environ.get("KSU_MOTOR_FR_SIGN", "1.0")),
    float(os.environ.get("KSU_MOTOR_RL_SIGN", "1.0")),
    float(os.environ.get("KSU_MOTOR_RR_SIGN", "1.0")),
)
JOYSTICK_DEADBAND = 0.06
INPUT_EXPO = 1.4
# Most setups already map forward to positive LY in driver.py.
# Override with KSU_JOYSTICK_Y_SIGN=1.0 if your controller is already forward-positive.
JOYSTICK_Y_SIGN = float(os.environ.get("KSU_JOYSTICK_Y_SIGN", "-1.0"))

# Global state
last_heartbeat = time.time()
heartbeat_lock = threading.Lock()
connection_lost = False
robot_mode = "STOPPED"  # STOPPED, AUTO, TELEOP
motor_controller = None
ZERO_MOTOR_SPEEDS = [0.0, 0.0, 0.0, 0.0]
VALID_ROBOT_MODES = {"AUTO", "TELEOP", "STOPPED"}
VALID_ODOMETRY_MODES = {"OPTICAL", "MOTOR", "HYBRID", "PRE_START"}


class MotorController:
    """Drive controller for 4 PWM+DIR channels (2x MDD10A)."""
    def __init__(self):
        self.available = PwmMotor is not None
        self.motors = []
        self.lock = threading.Lock()

        if not self.available:
            logger.warning("Motor hardware unavailable (hardware.py / gpiozero import failed). Running in simulation mode.")
            return

        for pwm_pin, dir_pin in MOTOR_PIN_MAP:
            self.motors.append(PwmMotor(pwm_pin, dir_pin, True))
        logger.info("Motor controller initialized for 2x MDD10A")
        logger.info(
            "Wheel mapping [FL, FR, RL, RR]=%s using backend=%s, signs=%s",
            MOTOR_PIN_MAP,
            "pca9685" if USE_PCA9685_PWM else "pi",
            MOTOR_DIRECTION_MULTIPLIER,
        )

    @staticmethod
    def _clamp(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))

    def set_speeds(self, speeds: List[float]) -> None:
        if not self.available:
            return

        if len(speeds) != 4:
            raise ValueError("Expected 4 motor speeds [FL, FR, RL, RR]")

        with self.lock:
            for i, speed in enumerate(speeds):
                command = self._clamp(speed) * float(MOTOR_DIRECTION_MULTIPLIER[i])
                self.motors[i].set_speed(command)

    def stop(self) -> None:
        self.set_speeds(ZERO_MOTOR_SPEEDS)


def ensure_motor_controller() -> MotorController:
    global motor_controller
    if motor_controller is None:
        motor_controller = MotorController()
    return motor_controller


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


@dataclass
class JoystickData:
    """Container for joystick input data."""

    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0

    def __post_init__(self) -> None:
        self.lx = _clamp_unit(self.lx)
        self.ly = _clamp_unit(self.ly)
        self.rx = _clamp_unit(self.rx)
        self.ry = _clamp_unit(self.ry)

def watchdog_thread(server: "RobotServer") -> None:
    """Monitor heartbeat and trigger emergency stop if connection is lost."""
    global connection_lost

    logger.info("Watchdog thread started")

    while server.running:
        try:
            with heartbeat_lock:
                time_since_heartbeat = time.time() - last_heartbeat

                if time_since_heartbeat > HEARTBEAT_TIMEOUT_S:
                    if not connection_lost:
                        server.all_stop()
                elif connection_lost:
                    logger.info("Connection restored")
                    connection_lost = False

            time.sleep(WATCHDOG_CHECK_INTERVAL_S)

        except Exception as e:
            logger.error(f"Watchdog error: {e}")
            time.sleep(WATCHDOG_CHECK_INTERVAL_S)

def calculate_motor_speeds(data: JoystickData) -> List[float]:
    """
    Calculate mecanum drive motor speeds from joystick input.
    Returns: List of 4 motor speeds [FL, FR, RL, RR]
    """
    def apply_deadband(value, deadband):
        value = float(value)
        if abs(value) < deadband:
            return 0.0

        # Rescale to keep full-range response after deadband.
        sign = 1.0 if value >= 0.0 else -1.0
        scaled = (abs(value) - deadband) / (1.0 - deadband)
        return sign * scaled

    def shape_input(value, expo):
        value = max(-1.0, min(1.0, float(value)))
        sign = 1.0 if value >= 0.0 else -1.0
        return sign * (abs(value) ** expo)

    x = shape_input(apply_deadband(data.lx, JOYSTICK_DEADBAND), INPUT_EXPO)  # strafe
    y = shape_input(apply_deadband(data.ly, JOYSTICK_DEADBAND), INPUT_EXPO)  # forward
    z = shape_input(apply_deadband(data.rx, JOYSTICK_DEADBAND), INPUT_EXPO)  # rotate

    motor1_speed = y + x + z  # Front Left
    motor2_speed = y - x - z  # Front Right
    motor3_speed = y - x + z  # Rear Left
    motor4_speed = y + x - z  # Rear Right

    speeds = [motor1_speed, motor2_speed, motor3_speed, motor4_speed]
    
    # Normalize speeds
    max_speed = max(abs(s) for s in speeds)
    if max_speed > 1.0:
        speeds = [s / max_speed for s in speeds]

    return speeds


def set_motor_speeds(speeds: List[float]) -> None:
    """Set motor speeds in order [FL, FR, RL, RR], each in [-1.0, 1.0]."""
    controller = ensure_motor_controller()
    try:
        controller.set_speeds(speeds)
    except Exception as e:
        logger.error(f"Failed to set motor speeds: {e}")


def update_heartbeat() -> None:
    """Update the last heartbeat timestamp."""
    global last_heartbeat, connection_lost
    with heartbeat_lock:
        last_heartbeat = time.time()
        if connection_lost:
            logger.info("Connection restored via command")
            connection_lost = False


class RobotServer:
    """ZMQ-based robot server"""
    def __init__(self):
        self.context = zmq.Context()
    
        # REP socket for commands
        self.command_socket = self.context.socket(zmq.REP)
        self.command_socket.bind(f"tcp://*:{COMMAND_PORT}")
    
        # PUB socket for telemetry
        self.telemetry_socket = self.context.socket(zmq.PUB)
        self.telemetry_socket.bind(f"tcp://*:{TELEMETRY_PORT}")
    
        self.running = True
        self.camera_thread = None
        self.bridge = None
    
        self.pose_x_m = FIELD_WIDTH_M / 2.0
        self.pose_y_m = FIELD_HEIGHT_M / 2.0
        self.pose_theta_deg = 0.0
        self.last_pose_update = time.time()
        self.odometry_mode = "PRE_START"
    
        self.telemetry_data: Dict[str, Any] = {
            "battery": 12.5,
            "mode": robot_mode,
            "odometry_mode": self.odometry_mode,
            "motor_speeds": ZERO_MOTOR_SPEEDS.copy(),
            "field": {
                "width_m": FIELD_WIDTH_M,
                "height_m": FIELD_HEIGHT_M,
            },
            "pose": {
                "x": self.pose_x_m,
                "y": self.pose_y_m,
                "theta_deg": self.pose_theta_deg,
            },
            "sensors": {
                "ultrasonic": 0,
                "ir": 0,
                "gyro": 0.0,
            },
            "encoders": [],
            "relay": 0,
            "mcu": {
                "bridge_connected": False,
            },
        }
    
        try:
            self.bridge = SerialBridge(
                port=os.environ.get("KSU_MCU_PORT", "/dev/ttyACM0"),
                baudrate=int(os.environ.get("KSU_MCU_BAUD", "115200")),
            )
            self.bridge.connect()
            logger.info("MCU serial bridge connected")
        except Exception as e:
            logger.warning("MCU bridge not available: %s", e)
            self.bridge = None
    
        logger.info(f"Robot server initialized on ports {COMMAND_PORT}/{TELEMETRY_PORT}")

    def all_stop(self) -> None:
        """Emergency stop for both MCU-backed and local motor control paths."""
        global connection_lost, robot_mode
    
        if not connection_lost:
            logger.warning("!!!! CONNECTION LOST - EMERGENCY STOP !!!!")
    
        connection_lost = True
        robot_mode = "STOPPED"
        self._stop_drive()
        self.telemetry_data["mode"] = robot_mode

    def _stop_drive(self) -> None:
        """Stop all drivetrain/mechanism outputs."""
        zero_drive = [0.0, 0.0, 0.0, 0.0]

        if self.bridge:
            drive_ok = self.bridge.set_drive_motors(zero_drive)
            mech_ok = self.bridge.set_mech_motors([0.0, 0.0, 0.0])

            if not drive_ok:
                logger.warning("Failed to send zero drive command to MCU")
            if not mech_ok:
                logger.warning("Failed to send zero mechanism command to MCU")
        else:
            set_motor_speeds(zero_drive)

        self.telemetry_data["motor_speeds"] = zero_drive.copy()

    def start_camera_broadcast(self):
        """Start MJPEG camera broadcast in a background thread."""
        if not ENABLE_CAMERA_BROADCAST:
            logger.info("Camera broadcast disabled via KSU_ENABLE_CAMERA_BROADCAST")
            return

        try:
            try:
                import subsystems.camera as camera_module
            except Exception:
                from .subsystems import camera as camera_module
        except Exception as e:
            logger.warning(f"Camera module unavailable: {e}")
            return

        def run_camera_server():
            try:
                camera_module.main()
            except Exception as e:
                logger.error(f"Camera broadcast stopped: {e}")

        self.camera_thread = threading.Thread(target=run_camera_server, daemon=True, name="camera-broadcast")
        self.camera_thread.start()
        stream_port = getattr(camera_module, "PORT", 8080)
        logger.info(f"Camera broadcast started on port {stream_port}")

    def _integrate_pose(self, lx: float, ly: float, rx: float) -> None:
        """Simple dead-reckoning from joystick commands."""
        now = time.time()
        dt = max(0.0, min(0.2, now - self.last_pose_update))
        self.last_pose_update = now
        if dt <= 0:
            return

        # Robot-frame velocities from joystick commands.
        v_forward = ly * MAX_LINEAR_SPEED_MPS
        v_strafe = lx * MAX_LINEAR_SPEED_MPS
        omega_deg = rx * MAX_ANGULAR_SPEED_DPS

        theta_rad = math.radians(self.pose_theta_deg)
        # Convert robot-frame velocities to field-frame velocities.
        v_field_x = (v_forward * math.cos(theta_rad)) - (v_strafe * math.sin(theta_rad))
        v_field_y = (v_forward * math.sin(theta_rad)) + (v_strafe * math.cos(theta_rad))

        self.pose_x_m = max(0.0, min(FIELD_WIDTH_M, self.pose_x_m + (v_field_x * dt)))
        self.pose_y_m = max(0.0, min(FIELD_HEIGHT_M, self.pose_y_m + (v_field_y * dt)))
        self.pose_theta_deg = (self.pose_theta_deg + (omega_deg * dt)) % 360.0

    def _reset_pose(self) -> None:
        """Reset pose to center field facing +X."""
        self.pose_x_m = FIELD_WIDTH_M / 2.0
        self.pose_y_m = FIELD_HEIGHT_M / 2.0
        self.pose_theta_deg = 0.0
        self.last_pose_update = time.time()
    
    def _read_drive_inputs(self, command: Dict[str, Any]) -> JoystickData:
        return JoystickData(
            lx=float(command.get("lx", 0.0)),
            ly=float(command.get("ly", 0.0)) * JOYSTICK_Y_SIGN,
            rx=float(command.get("rx", 0.0)),
            ry=float(command.get("ry", 0.0)),
        )

    def _update_telemetry_pose(self) -> None:
        self.telemetry_data["pose"] = {
            "x": self.pose_x_m,
            "y": self.pose_y_m,
            "theta_deg": self.pose_theta_deg,
        }

    def handle_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Process incoming command."""
        global robot_mode

        cmd_type = str(command.get("type", "")).strip().lower()
        update_heartbeat()

        try:
            if cmd_type == "ping":
                if self.bridge:
                    try:
                        self.bridge.send({"type": "ping"})
                    except Exception as e:
                        logger.debug(f"MCU ping send failed: {e}")

                return {"status": "success", "timestamp": time.time()}

            elif cmd_type == "joystick":
                joystick_data = self._read_drive_inputs(command)

                if robot_mode == "TELEOP" and not connection_lost:
                    motor_speeds = calculate_motor_speeds(joystick_data)
                    self._integrate_pose(joystick_data.lx, joystick_data.ly, joystick_data.rx)

                    if self.bridge:
                        ok = self.bridge.set_drive_motors(motor_speeds)
                        if not ok:
                            return {
                                "status": "error",
                                "message": "Failed to send drive command to MCU",
                            }
                    else:
                        set_motor_speeds(motor_speeds)

                    self.telemetry_data["motor_speeds"] = motor_speeds
                    logger.debug(f"Motors: {motor_speeds}")

                return {"status": "success"}

            elif cmd_type == "button":
                button_id = command.get("button_id")
                action = command.get("action")
                logger.info(f"Button {button_id} {action}")

                # TODO: map buttons to bridge actions here if needed.
                return {"status": "success"}

            elif cmd_type == "mode":
                new_mode = str(command.get("mode", "STOPPED")).upper()

                if new_mode not in VALID_ROBOT_MODES:
                    return {"status": "error", "message": f"Invalid mode: {new_mode}"}

                robot_mode = new_mode
                self.telemetry_data["mode"] = robot_mode
                logger.info(f"Mode changed to: {robot_mode}")

                if self.bridge:
                    ok = self.bridge.set_mode(robot_mode)
                    if not ok:
                        logger.warning("Failed to forward mode to MCU: %s", robot_mode)

                if robot_mode == "STOPPED":
                    self._stop_drive()

                return {"status": "success", "mode": robot_mode}

            elif cmd_type == "reset":
                robot_mode = "STOPPED"
                self.telemetry_data["mode"] = robot_mode

                self._stop_drive()
                self._reset_pose()

                if self.bridge:
                    ok = self.bridge.reset()
                    if not ok:
                        logger.warning("Failed to send reset command to MCU")

                    mode_ok = self.bridge.set_mode(robot_mode)
                    if not mode_ok:
                        logger.warning("Failed to send STOPPED mode to MCU after reset")

                logger.info("Robot reset")
                return {"status": "success"}

            elif cmd_type == "reset_odometry":
                self._reset_pose()
                logger.info("Odometry reset")
                return {"status": "success"}

            elif cmd_type == "odometry_mode":
                mode = str(command.get("mode", "PRE_START")).upper()

                if mode not in VALID_ODOMETRY_MODES:
                    return {"status": "error", "message": f"Invalid odometry mode: {mode}"}

                self.odometry_mode = mode
                self.telemetry_data["odometry_mode"] = self.odometry_mode
                return {"status": "success", "odometry_mode": self.odometry_mode}

            else:
                logger.warning(f"Unknown command: {cmd_type}")
                return {"status": "error", "message": f"Unknown command: {cmd_type}"}

        except Exception as e:
            logger.error(f"Error handling command: {e}")
            return {"status": "error", "message": str(e)}
    
    def command_loop(self) -> None:
        """Handle incoming commands"""
        logger.info("Command handler ready")
        
        while self.running:
            try:
                command = self.command_socket.recv_json()
                response = self.handle_command(command)
                self.command_socket.send_json(response)
            except Exception as e:
                logger.error(f"Command loop error: {e}")
                try:
                    self.command_socket.send_json({
                        'status': 'error',
                        'message': str(e)
                    })
                except Exception:
                    pass
    
    def telemetry_loop(self) -> None:
        """Broadcast telemetry."""
        logger.info("Telemetry broadcaster ready")

        while self.running:
            try:
                self.telemetry_data["timestamp"] = time.time()
                self.telemetry_data["mode"] = robot_mode
                self.telemetry_data["odometry_mode"] = self.odometry_mode

                self._update_telemetry_pose()

                if self.bridge:
                    try:
                        mcu = self.bridge.get_latest_telemetry()
                    except Exception as e:
                        logger.error(f"Failed to read MCU telemetry: {e}")
                        mcu = {"bridge_connected": False}

                    self.telemetry_data["mcu"] = mcu
                    self.telemetry_data["encoders"] = mcu.get("encoders", [])
                    self.telemetry_data["relay"] = mcu.get("relay", 0)
                else:
                    self.telemetry_data["mcu"] = {"bridge_connected": False}
                    self.telemetry_data["encoders"] = []
                    self.telemetry_data["relay"] = 0

                self.telemetry_socket.send_json(self.telemetry_data)
                time.sleep(1.0 / TELEMETRY_RATE_HZ)

            except Exception as e:
                logger.error(f"Telemetry error: {e}")
                time.sleep(1.0 / TELEMETRY_RATE_HZ)

    def start(self) -> None:
        """Start server threads."""
        self.start_camera_broadcast()

        watchdog = threading.Thread(
            target=watchdog_thread,
            args=(self,),
            daemon=True,
            name="watchdog",
        )
        watchdog.start()

        telemetry_thread = threading.Thread(
            target=self.telemetry_loop,
            daemon=True,
            name="telemetry",
        )
        telemetry_thread.start()

        try:
            self.command_loop()
        except KeyboardInterrupt:
            logger.info("Server shutdown requested")
            self.running = False

    def cleanup(self) -> None:
        """Clean up resources."""
        self.running = False

        try:
            self._stop_drive()
        except Exception as e:
            logger.error(f"Failed to stop outputs during cleanup: {e}")

        if self.bridge:
            try:
                self.bridge.close()
            except Exception as e:
                logger.error(f"Failed to close MCU bridge: {e}")

        try:
            ensure_motor_controller().stop()
        except Exception as e:
            logger.debug(f"Local motor cleanup skipped/failed: {e}")

        try:
            self.command_socket.close(0)
        except Exception:
            pass

        try:
            self.telemetry_socket.close(0)
        except Exception:
            pass

        try:
            self.context.term()
        except Exception:
            pass


def main():
    os.system('cls' if os.name == 'nt' else 'clear')
    """Start the robot server"""
    logger.info("Starting robot server...")
    
    server = RobotServer()
    
    try:
        server.start()
    finally:
        server.cleanup()
        logger.info("Robot server stopped")


if __name__ == "__main__":
    main()
