import os
import math
import logging
import threading
import time
from typing import Any, Dict, Optional

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
    AUTO_LOOP_INTERVAL_S,
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
from subsystems.autonomous import (
    build_distance_target,
    compute_drive_to_pose_command,
    compute_turn_command,
    parse_routine,
    pose_from_values,
)
from subsystems.drive import (
    JoystickData,
    calculate_motor_speeds,
    ensure_motor_controller,
    set_motor_speeds,
)

try:
    from hardware.optical_odometry_sensor import OpticalOdometrySensor
except Exception:
    OpticalOdometrySensor = None

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
ENABLE_OPTICAL_ODOMETRY = os.environ.get("KSU_ENABLE_OPTICAL_ODOMETRY", "1").strip().lower() not in ("0", "false", "no")
OPTICAL_ODOMETRY_AUTO_CALIBRATE = os.environ.get("KSU_OPTICAL_ODOMETRY_AUTO_CALIBRATE", "0").strip().lower() not in ("0", "false", "no")
INCHES_TO_METERS = 0.0254

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
        self.optical_sensor: Optional[OpticalOdometrySensor] = None
        self.optical_sensor_connected = False
        self.optical_origin_x_m = 0.0
        self.optical_origin_y_m = 0.0
        self.optical_origin_heading_deg = 0.0
        self.last_optical_pose: Dict[str, float] = {
            "x_in": 0.0,
            "y_in": 0.0,
            "heading_deg": 0.0,
        }
    
        self.pose_x_m, self.pose_y_m, self.pose_theta_deg = self._alliance_start_pose()
        self.last_pose_update = time.time()
        self.odometry_mode = "PRE_START"
        self.auto_lock = threading.Lock()
        self.auto_state: Dict[str, Any] = {
            "active": False,
            "status": "idle",
            "routine_name": None,
            "steps": [],
            "step_index": 0,
            "step_started_at": 0.0,
            "step_start_pose": None,
            "last_error": None,
        }
    
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
                "optical_odometry": {
                    "connected": False,
                    "x_in": 0.0,
                    "y_in": 0.0,
                    "heading_deg": 0.0,
                },
            },
            "encoders": [],
            "relay": 0,
            "mcu": {
                "bridge_connected": False,
            },
            "auto": self._build_auto_status(),
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

        self._initialize_optical_sensor()
        self._sync_optical_origin_with_pose(reset_sensor=True)
    
        logger.info(f"Robot server initialized on ports {COMMAND_PORT}/{TELEMETRY_PORT}")

    def all_stop(self) -> None:
        """Emergency stop for both MCU-backed and local motor control paths."""
        global connection_lost, robot_mode
    
        if not connection_lost:
            logger.warning("!!!! CONNECTION LOST - EMERGENCY STOP !!!!")
    
        connection_lost = True
        robot_mode = "STOPPED"
        self._cancel_auto("Emergency stop")
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

    def _set_drive_outputs(self, motor_speeds: list[float]) -> bool:
        if self.bridge:
            ok = self.bridge.set_drive_motors(motor_speeds)
            if not ok:
                return False
        else:
            set_motor_speeds(motor_speeds)

        self.telemetry_data["motor_speeds"] = [float(speed) for speed in motor_speeds]
        return True

    def _apply_drive_command(self, joystick_data: JoystickData) -> bool:
        motor_speeds = calculate_motor_speeds(joystick_data)
        if self.odometry_mode not in {"OPTICAL", "HYBRID"} or not self._update_optical_pose():
            self._integrate_pose(joystick_data.lx, joystick_data.ly, joystick_data.rx)

        return self._set_drive_outputs(motor_speeds)

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

    def _initialize_optical_sensor(self) -> None:
        if not ENABLE_OPTICAL_ODOMETRY:
            logger.info("Optical odometry disabled via KSU_ENABLE_OPTICAL_ODOMETRY")
            return

        if OpticalOdometrySensor is None:
            logger.warning("Optical odometry sensor module unavailable")
            return

        try:
            sensor = OpticalOdometrySensor()
            if not sensor.connect():
                logger.warning("Optical odometry sensor not connected")
                return

            if OPTICAL_ODOMETRY_AUTO_CALIBRATE:
                logger.info("Calibrating optical odometry sensor")
                sensor.calibrate()
            else:
                sensor.reset_tracking()

            self.optical_sensor = sensor
            self.optical_sensor_connected = True
            logger.info("Optical odometry sensor connected")
        except Exception as exc:
            logger.warning("Failed to initialize optical odometry sensor: %s", exc)
            self.optical_sensor = None
            self.optical_sensor_connected = False

    def _sync_optical_origin_with_pose(self, reset_sensor: bool) -> None:
        self.optical_origin_x_m = self.pose_x_m
        self.optical_origin_y_m = self.pose_y_m
        self.optical_origin_heading_deg = self.pose_theta_deg

        if self.optical_sensor_connected and self.optical_sensor is not None and reset_sensor:
            try:
                self.optical_sensor.reset_tracking()
            except Exception as exc:
                logger.warning("Failed to reset optical odometry tracking: %s", exc)
                self.optical_sensor_connected = False

    def _update_optical_pose(self) -> bool:
        if not self.optical_sensor_connected or self.optical_sensor is None:
            return False

        try:
            pose = self.optical_sensor.read_pose()
            self.last_optical_pose = pose

            optical_x_m = pose["x_in"] * INCHES_TO_METERS
            optical_y_m = pose["y_in"] * INCHES_TO_METERS
            optical_heading_deg = pose["heading_deg"]

            self.pose_x_m = max(0.0, min(FIELD_WIDTH_M, self.optical_origin_x_m + optical_x_m))
            self.pose_y_m = max(0.0, min(FIELD_HEIGHT_M, self.optical_origin_y_m + optical_y_m))
            self.pose_theta_deg = (self.optical_origin_heading_deg + optical_heading_deg) % 360.0
            return True
        except Exception as exc:
            logger.warning("Optical odometry read failed: %s", exc)
            self.optical_sensor_connected = False
            return False

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
        self._sync_optical_origin_with_pose(reset_sensor=True)
    
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
        self.telemetry_data["sensors"]["optical_odometry"] = {
            "connected": self.optical_sensor_connected,
            "x_in": self.last_optical_pose.get("x_in", 0.0),
            "y_in": self.last_optical_pose.get("y_in", 0.0),
            "heading_deg": self.last_optical_pose.get("heading_deg", 0.0),
        }
        self.telemetry_data["auto"] = self._build_auto_status()

    def _build_auto_status(self) -> Dict[str, Any]:
        with self.auto_lock:
            steps = self.auto_state["steps"]
            step_index = int(self.auto_state["step_index"])
            current_step = steps[step_index] if self.auto_state["active"] and step_index < len(steps) else None
            return {
                "active": bool(self.auto_state["active"]),
                "status": str(self.auto_state["status"]),
                "routine_name": self.auto_state["routine_name"],
                "step_index": step_index,
                "step_count": len(steps),
                "step_name": current_step.get("name") if current_step else None,
                "step_action": current_step.get("action") if current_step else None,
                "last_error": self.auto_state["last_error"],
            }

    def _set_robot_mode(self, new_mode: str) -> None:
        global robot_mode

        robot_mode = new_mode
        self.telemetry_data["mode"] = robot_mode
        logger.info("Mode changed to: %s", robot_mode)

        if self.bridge:
            ok = self.bridge.set_mode(robot_mode)
            if not ok:
                logger.warning("Failed to forward mode to MCU: %s", robot_mode)

        if robot_mode != "AUTO":
            self._cancel_auto("Robot mode changed")

        if robot_mode == "STOPPED":
            self._stop_drive()

    def _start_auto_routine(self, routine_name: str, raw_steps: Any) -> Dict[str, Any]:
        steps = parse_routine(routine_name, raw_steps)

        with self.auto_lock:
            self.auto_state = {
                "active": True,
                "status": "running",
                "routine_name": str(routine_name),
                "steps": steps,
                "step_index": 0,
                "step_started_at": 0.0,
                "step_start_pose": None,
                "last_error": None,
            }

        self._set_robot_mode("AUTO")
        logger.info("Auto routine started: %s (%d steps)", routine_name, len(steps))
        return self._build_auto_status()

    def _cancel_auto(self, reason: str) -> Dict[str, Any]:
        with self.auto_lock:
            was_active = bool(self.auto_state["active"])
            self.auto_state["active"] = False
            self.auto_state["status"] = "cancelled"
            self.auto_state["last_error"] = str(reason)
            self.auto_state["step_started_at"] = 0.0
            self.auto_state["step_start_pose"] = None

        if was_active:
            logger.info("Auto routine cancelled: %s", reason)
        self._stop_drive()
        return self._build_auto_status()

    def _complete_auto(self) -> None:
        with self.auto_lock:
            self.auto_state["active"] = False
            self.auto_state["status"] = "completed"
            self.auto_state["step_started_at"] = 0.0
            self.auto_state["step_start_pose"] = None
            self.auto_state["last_error"] = None

        logger.info("Auto routine completed")
        self._stop_drive()

    def _fail_auto(self, error_message: str) -> None:
        with self.auto_lock:
            self.auto_state["active"] = False
            self.auto_state["status"] = "error"
            self.auto_state["last_error"] = str(error_message)
            self.auto_state["step_started_at"] = 0.0
            self.auto_state["step_start_pose"] = None

        logger.warning("Auto routine failed: %s", error_message)
        self._stop_drive()

    def _advance_auto_step(self) -> None:
        with self.auto_lock:
            self.auto_state["step_index"] += 1
            self.auto_state["step_started_at"] = 0.0
            self.auto_state["step_start_pose"] = None
            finished = self.auto_state["step_index"] >= len(self.auto_state["steps"])

        if finished:
            self._complete_auto()

    def _capture_pose(self) -> Dict[str, float]:
        return pose_from_values(self.pose_x_m, self.pose_y_m, self.pose_theta_deg)

    def _execute_auto_step(self, step: Dict[str, Any], elapsed_s: float, start_pose: Optional[Dict[str, float]]) -> bool:
        action = step["action"]

        if action == "wait":
            if elapsed_s >= float(step["duration_s"]):
                self._stop_drive()
                return True
            return False

        if action == "reset_odometry":
            self._reset_pose()
            return True

        if action == "set_odometry_mode":
            mode = str(step["mode"]).upper()
            if mode not in VALID_ODOMETRY_MODES:
                raise ValueError(f"Invalid auto odometry mode: {mode}")
            self.odometry_mode = mode
            self.telemetry_data["odometry_mode"] = self.odometry_mode
            return True

        if action == "drive_distance":
            if start_pose is None:
                raise ValueError("Auto drive_distance missing start pose")

            hold_heading_deg = step.get("hold_heading_deg")
            target_heading_deg = float(hold_heading_deg) if hold_heading_deg is not None else float(start_pose["theta_deg"])
            target_x_m, target_y_m = build_distance_target(
                start_pose,
                float(step["forward_m"]),
                float(step["strafe_m"]),
            )
            lx, ly, rx, done = compute_drive_to_pose_command(
                self._capture_pose(),
                target_x_m,
                target_y_m,
                target_heading_deg,
                float(step["speed"]),
                float(step["turn_speed"]),
                float(step["position_tolerance_m"]),
                float(step["angle_tolerance_deg"]),
            )
            if done:
                self._stop_drive()
                return True
            if not self._apply_drive_command(JoystickData(lx=lx, ly=ly, rx=rx)):
                raise RuntimeError("Failed to send auto drive_distance command to MCU")
            return False

        if action == "turn_to_angle":
            rx, done = compute_turn_command(
                self.pose_theta_deg,
                float(step["angle_deg"]),
                float(step["speed"]),
                float(step["angle_tolerance_deg"]),
            )
            if done:
                self._stop_drive()
                return True
            if not self._apply_drive_command(JoystickData(rx=rx)):
                raise RuntimeError("Failed to send auto turn command to MCU")
            return False

        if action == "drive_to_pose":
            target_heading_deg = step.get("heading_deg")
            if target_heading_deg is not None:
                target_heading_deg = float(target_heading_deg)
            lx, ly, rx, done = compute_drive_to_pose_command(
                self._capture_pose(),
                float(step["x_m"]),
                float(step["y_m"]),
                target_heading_deg,
                float(step["speed"]),
                float(step["turn_speed"]),
                float(step["position_tolerance_m"]),
                float(step["angle_tolerance_deg"]),
            )
            if done:
                self._stop_drive()
                return True
            if not self._apply_drive_command(JoystickData(lx=lx, ly=ly, rx=rx)):
                raise RuntimeError("Failed to send auto drive_to_pose command to MCU")
            return False

        raise ValueError(f"Unsupported auto action: {action}")

    def auto_loop(self) -> None:
        logger.info("Auto runner ready")

        while self.running:
            try:
                with self.auto_lock:
                    if not self.auto_state["active"]:
                        step = None
                    else:
                        step_index = int(self.auto_state["step_index"])
                        step = self.auto_state["steps"][step_index]
                        if self.auto_state["step_started_at"] <= 0.0:
                            self.auto_state["step_started_at"] = time.time()
                            self.auto_state["step_start_pose"] = self._capture_pose()
                            logger.info(
                                "Auto step %d/%d: %s",
                                step_index + 1,
                                len(self.auto_state["steps"]),
                                step["name"],
                            )
                        step_started_at = float(self.auto_state["step_started_at"])
                        step_start_pose = self.auto_state["step_start_pose"]

                if step is None:
                    time.sleep(AUTO_LOOP_INTERVAL_S)
                    continue

                if robot_mode != "AUTO":
                    time.sleep(AUTO_LOOP_INTERVAL_S)
                    continue

                if connection_lost:
                    self._fail_auto("Connection lost")
                    time.sleep(AUTO_LOOP_INTERVAL_S)
                    continue

                elapsed_s = time.time() - step_started_at
                if elapsed_s > float(step["timeout_s"]):
                    self._fail_auto(f"Auto step timed out: {step['name']}")
                    time.sleep(AUTO_LOOP_INTERVAL_S)
                    continue

                if self._execute_auto_step(step, elapsed_s, step_start_pose):
                    self._advance_auto_step()

                time.sleep(AUTO_LOOP_INTERVAL_S)
            except Exception as e:
                self._fail_auto(str(e))
                time.sleep(AUTO_LOOP_INTERVAL_S)

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
                    if not self._apply_drive_command(joystick_data):
                        return {
                            "status": "error",
                            "message": "Failed to send drive command to MCU",
                        }

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

                self._set_robot_mode(new_mode)

                return {"status": "success", "mode": robot_mode}

            elif cmd_type == "reset":
                self._cancel_auto("Robot reset")
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

            elif cmd_type == "auto_run":
                routine = command.get("routine", {})
                if isinstance(routine, dict):
                    routine_name = str(routine.get("name", command.get("name", "auto_routine")))
                    raw_steps = routine.get("steps", command.get("steps"))
                else:
                    routine_name = str(command.get("name", "auto_routine"))
                    raw_steps = command.get("steps")
                return {
                    "status": "success",
                    "auto": self._start_auto_routine(routine_name, raw_steps),
                }

            elif cmd_type == "auto_cancel":
                return {"status": "success", "auto": self._cancel_auto("Cancelled by command")}

            elif cmd_type == "auto_status":
                return {"status": "success", "auto": self._build_auto_status()}

            elif cmd_type == "auto_drive_distance":
                step = {
                    "action": "drive_distance",
                    "name": command.get("name", "drive_distance"),
                    "distance_m": command.get("distance_m", command.get("forward_m", 0.0)),
                    "forward_m": command.get("forward_m", command.get("distance_m", 0.0)),
                    "strafe_m": command.get("strafe_m", 0.0),
                    "hold_heading_deg": command.get("hold_heading_deg"),
                    "speed": command.get("speed"),
                    "turn_speed": command.get("turn_speed"),
                    "timeout_s": command.get("timeout_s"),
                    "position_tolerance_m": command.get("position_tolerance_m"),
                    "angle_tolerance_deg": command.get("angle_tolerance_deg"),
                }
                return {
                    "status": "success",
                    "auto": self._start_auto_routine("auto_drive_distance", [step]),
                }

            elif cmd_type == "auto_turn_to_angle":
                step = {
                    "action": "turn_to_angle",
                    "name": command.get("name", "turn_to_angle"),
                    "angle_deg": command.get("angle_deg"),
                    "speed": command.get("speed"),
                    "timeout_s": command.get("timeout_s"),
                    "angle_tolerance_deg": command.get("angle_tolerance_deg"),
                }
                return {
                    "status": "success",
                    "auto": self._start_auto_routine("auto_turn_to_angle", [step]),
                }

            elif cmd_type == "auto_drive_to_pose":
                step = {
                    "action": "drive_to_pose",
                    "name": command.get("name", "drive_to_pose"),
                    "x_m": command.get("x_m"),
                    "y_m": command.get("y_m"),
                    "heading_deg": command.get("heading_deg"),
                    "speed": command.get("speed"),
                    "turn_speed": command.get("turn_speed"),
                    "timeout_s": command.get("timeout_s"),
                    "position_tolerance_m": command.get("position_tolerance_m"),
                    "angle_tolerance_deg": command.get("angle_tolerance_deg"),
                }
                return {
                    "status": "success",
                    "auto": self._start_auto_routine("auto_drive_to_pose", [step]),
                }

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

                if self.odometry_mode in {"OPTICAL", "HYBRID"}:
                    self._update_optical_pose()

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

        auto_thread = threading.Thread(
            target=self.auto_loop,
            daemon=True,
            name="auto",
        )
        auto_thread.start()

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
