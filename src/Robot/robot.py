import os
import math
import logging
import threading
import time
from typing import Any, Dict

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
from constants import (
    COMMAND_PORT,
    ENABLE_CAMERA_BROADCAST,
    FIELD_HEIGHT_M,
    FIELD_WIDTH_M,
    HEARTBEAT_TIMEOUT_S,
    JOYSTICK_Y_SIGN,
    MAX_ANGULAR_SPEED_DPS,
    MAX_LINEAR_SPEED_MPS,
    ROBOT_FOOTPRINT_SIZE_M,
    TELEMETRY_PORT,
    TELEMETRY_RATE_HZ,
    VALID_ALLIANCES,
    VALID_ODOMETRY_MODES,
    VALID_ROBOT_MODES,
    WATCHDOG_CHECK_INTERVAL_S,
    ZERO_MOTOR_SPEEDS,
)
from subsystems.drive import (
    JoystickData,
    calculate_motor_speeds,
    ensure_motor_controller,
    set_motor_speeds,
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Global state
last_heartbeat = time.time()
heartbeat_lock = threading.Lock()
heartbeat_seen = False
connection_lost = False
robot_mode = "STOPPED"  # STOPPED, AUTO, TELEOP

def watchdog_thread(server: "RobotServer") -> None:
    """Monitor heartbeat and trigger emergency stop if connection is lost."""
    global connection_lost, heartbeat_seen

    logger.info("Watchdog thread started")

    while server.running:
        try:
            should_sleep = False
            with heartbeat_lock:
                if not heartbeat_seen:
                    should_sleep = True
                else:
                    time_since_heartbeat = time.time() - last_heartbeat

                    if time_since_heartbeat > HEARTBEAT_TIMEOUT_S:
                        if not connection_lost:
                            server.all_stop()
                    elif connection_lost:
                        logger.info("Connection restored")
                        connection_lost = False

            if should_sleep:
                time.sleep(WATCHDOG_CHECK_INTERVAL_S)
                continue

            time.sleep(WATCHDOG_CHECK_INTERVAL_S)

        except Exception as e:
            logger.error(f"Watchdog error: {e}")
            time.sleep(WATCHDOG_CHECK_INTERVAL_S)


def update_heartbeat() -> None:
    """Update the last heartbeat timestamp."""
    global last_heartbeat, connection_lost, heartbeat_seen
    with heartbeat_lock:
        last_heartbeat = time.time()
        if not heartbeat_seen:
            heartbeat_seen = True
            logger.info("Driver heartbeat detected; watchdog armed")
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
        self.alliance = "RED"
    
        self.pose_x_m, self.pose_y_m, self.pose_theta_deg = self._alliance_start_pose()
        self.last_pose_update = time.time()
        self.odometry_mode = "PRE_START"
    
        self.telemetry_data: Dict[str, Any] = {
            "battery": 12.5,
            "mode": robot_mode,
            "odometry_mode": self.odometry_mode,
            "alliance": self.alliance,
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

    def _alliance_start_pose(self) -> tuple[float, float, float]:
        half_robot = ROBOT_FOOTPRINT_SIZE_M / 2.0
        start_y = half_robot
        start_theta_deg = 90.0

        if self.alliance == "BLUE":
            start_x = FIELD_WIDTH_M - half_robot
        else:
            start_x = half_robot

        return start_x, start_y, start_theta_deg

    def _reset_pose(self) -> None:
        """Reset pose to the selected alliance starting corner."""
        self.pose_x_m, self.pose_y_m, self.pose_theta_deg = self._alliance_start_pose()
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
        self.telemetry_data["alliance"] = self.alliance

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

            elif cmd_type == "alliance":
                new_alliance = str(command.get("alliance", self.alliance)).upper()

                if new_alliance not in VALID_ALLIANCES:
                    return {"status": "error", "message": f"Invalid alliance: {new_alliance}"}

                self.alliance = new_alliance
                self.telemetry_data["alliance"] = self.alliance
                logger.info("Alliance set to: %s", self.alliance)
                return {"status": "success", "alliance": self.alliance}

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
