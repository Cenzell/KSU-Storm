from __future__ import annotations

import json
import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _find_repo_root() -> Path:
    candidates: List[Path] = []

    env_root = os.environ.get("KSU_STORM_REPO_ROOT", "").strip()
    if env_root:
        candidates.append(Path(env_root).expanduser())

    current = Path(__file__).resolve()
    candidates.extend(current.parents)
    candidates.extend(Path.cwd().resolve().parents)
    candidates.append(Path.cwd().resolve())

    for candidate in candidates:
        if (candidate / "lib" / "serial_bridge.py").exists() and (candidate / "src" / "Robot").exists():
            return candidate
    raise RuntimeError("Could not locate KSU-Storm repository root from ROS 2 package")


REPO_ROOT = _find_repo_root()
LIB_DIR = REPO_ROOT / "lib"
ROBOT_SRC_DIR = REPO_ROOT / "src" / "Robot"
HARDWARE_DIR = ROBOT_SRC_DIR / "hardware"

for path in (LIB_DIR, ROBOT_SRC_DIR, HARDWARE_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from serial_bridge import SerialBridge

try:
    from hardware import PwmMotor
except Exception:
    PwmMotor = None


LOGGER = logging.getLogger(__name__)

TELEMETRY_RATE_HZ = 10.0
HEARTBEAT_TIMEOUT_S = 2.5
WATCHDOG_CHECK_INTERVAL_S = 0.1
MAX_LINEAR_SPEED_MPS = 1.2
MAX_ANGULAR_SPEED_RADPS = math.radians(180.0)
FIELD_WIDTH_M = 3.6
FIELD_HEIGHT_M = 3.6
USE_PCA9685_PWM = os.environ.get("KSU_PWM_BACKEND", "pi").strip().lower() in ("pca", "pca9685")

if USE_PCA9685_PWM:
    MOTOR_PIN_MAP = (
        (0, 5),
        (1, 6),
        (2, 16),
        (3, 20),
    )
else:
    MOTOR_PIN_MAP = (
        (12, 5),
        (13, 6),
        (18, 16),
        (19, 20),
    )

MOTOR_DIRECTION_MULTIPLIER = (
    float(os.environ.get("KSU_MOTOR_FL_SIGN", "1.0")),
    float(os.environ.get("KSU_MOTOR_FR_SIGN", "1.0")),
    float(os.environ.get("KSU_MOTOR_RL_SIGN", "1.0")),
    float(os.environ.get("KSU_MOTOR_RR_SIGN", "1.0")),
)
VALID_ROBOT_MODES = {"AUTO", "TELEOP", "STOPPED"}
VALID_ODOMETRY_MODES = {"OPTICAL", "MOTOR", "HYBRID", "PRE_START"}
ZERO_MOTOR_SPEEDS = [0.0, 0.0, 0.0, 0.0]


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


@dataclass
class DriveCommand:
    strafe: float = 0.0
    forward: float = 0.0
    yaw: float = 0.0

    def normalized(self) -> "DriveCommand":
        return DriveCommand(
            strafe=_clamp_unit(self.strafe),
            forward=_clamp_unit(self.forward),
            yaw=_clamp_unit(self.yaw),
        )


class MotorController:
    """Drive controller for four PWM+DIR channels."""

    def __init__(self, logger: logging.Logger):
        self._logger = logger
        self.available = PwmMotor is not None
        self.motors: List[PwmMotor] = []
        self.lock = threading.Lock()

        if not self.available:
            self._logger.warning(
                "Motor hardware unavailable (hardware.py import failed). Running ROS node in simulation mode."
            )
            return

        for pwm_pin, dir_pin in MOTOR_PIN_MAP:
            self.motors.append(PwmMotor(pwm_pin, dir_pin, True))

        self._logger.info(
            "Motor controller initialized with backend=%s, map=%s, signs=%s",
            "pca9685" if USE_PCA9685_PWM else "pi",
            MOTOR_PIN_MAP,
            MOTOR_DIRECTION_MULTIPLIER,
        )

    def set_speeds(self, speeds: List[float]) -> None:
        if not self.available:
            return

        if len(speeds) != 4:
            raise ValueError("Expected 4 motor speeds [FL, FR, RL, RR]")

        with self.lock:
            for index, speed in enumerate(speeds):
                command = _clamp_unit(speed) * float(MOTOR_DIRECTION_MULTIPLIER[index])
                self.motors[index].set_speed(command)

    def stop(self) -> None:
        self.set_speeds(ZERO_MOTOR_SPEEDS)


def calculate_motor_speeds(command: DriveCommand) -> List[float]:
    normalized = command.normalized()
    fl = normalized.forward + normalized.strafe + normalized.yaw
    fr = normalized.forward - normalized.strafe - normalized.yaw
    rl = normalized.forward - normalized.strafe + normalized.yaw
    rr = normalized.forward + normalized.strafe - normalized.yaw
    speeds = [fl, fr, rl, rr]

    max_speed = max(abs(speed) for speed in speeds)
    if max_speed > 1.0:
        speeds = [speed / max_speed for speed in speeds]
    return speeds


class RobotNode(Node):
    """Initial ROS 2 wrapper around the existing robot runtime."""

    def __init__(self) -> None:
        super().__init__("ksu_storm_robot")

        self.declare_parameter("telemetry_rate_hz", TELEMETRY_RATE_HZ)
        self.declare_parameter("heartbeat_timeout_s", HEARTBEAT_TIMEOUT_S)
        self.declare_parameter("watchdog_interval_s", WATCHDOG_CHECK_INTERVAL_S)
        self.declare_parameter("max_linear_speed_mps", MAX_LINEAR_SPEED_MPS)
        self.declare_parameter("max_angular_speed_radps", MAX_ANGULAR_SPEED_RADPS)
        self.declare_parameter("field_width_m", FIELD_WIDTH_M)
        self.declare_parameter("field_height_m", FIELD_HEIGHT_M)
        self.declare_parameter("mcu_port", os.environ.get("KSU_MCU_PORT", "/dev/ttyACM0"))
        self.declare_parameter("mcu_baud", int(os.environ.get("KSU_MCU_BAUD", "115200")))

        self.telemetry_rate_hz = float(self.get_parameter("telemetry_rate_hz").value)
        self.heartbeat_timeout_s = float(self.get_parameter("heartbeat_timeout_s").value)
        self.watchdog_interval_s = float(self.get_parameter("watchdog_interval_s").value)
        self.max_linear_speed_mps = float(self.get_parameter("max_linear_speed_mps").value)
        self.max_angular_speed_radps = float(self.get_parameter("max_angular_speed_radps").value)
        self.field_width_m = float(self.get_parameter("field_width_m").value)
        self.field_height_m = float(self.get_parameter("field_height_m").value)

        self.robot_mode = "STOPPED"
        self.odometry_mode = "PRE_START"
        self.connection_lost = False
        self.last_command_time = time.time()
        self.last_pose_update = time.time()
        self.last_drive_command = DriveCommand()

        self.pose_x_m = self.field_width_m / 2.0
        self.pose_y_m = self.field_height_m / 2.0
        self.pose_theta_rad = 0.0

        self.motor_controller = MotorController(self.get_logger())
        self.bridge = self._connect_serial_bridge()

        self.telemetry_pub = self.create_publisher(String, "telemetry/json", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)

        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)
        self.create_subscription(String, "robot_mode", self._mode_callback, 10)
        self.create_subscription(String, "odometry_mode", self._odometry_mode_callback, 10)

        self.create_service(Trigger, "reset_robot", self._reset_robot)
        self.create_service(Trigger, "reset_odometry", self._reset_odometry)

        self.watchdog_timer = self.create_timer(self.watchdog_interval_s, self._watchdog_tick)
        self.telemetry_timer = self.create_timer(1.0 / max(self.telemetry_rate_hz, 1.0), self._telemetry_tick)

        self.get_logger().info("KSU Storm ROS 2 robot node started")

    def _connect_serial_bridge(self) -> SerialBridge | None:
        port = str(self.get_parameter("mcu_port").value)
        baud = int(self.get_parameter("mcu_baud").value)
        try:
            bridge = SerialBridge(port=port, baudrate=baud)
            bridge.connect()
            self.get_logger().info("Connected to MCU serial bridge on %s @ %d", port, baud)
            return bridge
        except Exception as exc:
            self.get_logger().warning("MCU serial bridge unavailable: %s", exc)
            return None

    def _touch_heartbeat(self) -> None:
        self.last_command_time = time.time()
        if self.connection_lost:
            self.connection_lost = False
            self.get_logger().info("Command stream restored")

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._touch_heartbeat()
        command = DriveCommand(
            strafe=msg.linear.y / self.max_linear_speed_mps,
            forward=msg.linear.x / self.max_linear_speed_mps,
            yaw=msg.angular.z / self.max_angular_speed_radps,
        ).normalized()

        self.last_drive_command = command

        if self.robot_mode != "TELEOP":
            return

        motor_speeds = calculate_motor_speeds(command)
        self._integrate_pose(command)
        self._apply_drive(motor_speeds)

    def _mode_callback(self, msg: String) -> None:
        self._touch_heartbeat()
        mode = str(msg.data).strip().upper()
        if mode not in VALID_ROBOT_MODES:
            self.get_logger().warning("Ignoring invalid robot mode: %s", mode)
            return

        self.robot_mode = mode
        self.get_logger().info("Robot mode -> %s", mode)

        if self.bridge:
            try:
                if not self.bridge.set_mode(mode):
                    self.get_logger().warning("Failed to forward mode %s to MCU", mode)
            except Exception as exc:
                self.get_logger().warning("Mode forward failed: %s", exc)

        if mode == "STOPPED":
            self._all_stop()

    def _odometry_mode_callback(self, msg: String) -> None:
        mode = str(msg.data).strip().upper()
        if mode not in VALID_ODOMETRY_MODES:
            self.get_logger().warning("Ignoring invalid odometry mode: %s", mode)
            return
        self.odometry_mode = mode

    def _reset_robot(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.robot_mode = "STOPPED"
        self._all_stop()
        self._reset_pose()

        if self.bridge:
            try:
                self.bridge.reset()
                self.bridge.set_mode(self.robot_mode)
            except Exception as exc:
                self.get_logger().warning("Reset forwarding failed: %s", exc)

        response.success = True
        response.message = "Robot reset"
        return response

    def _reset_odometry(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._reset_pose()
        response.success = True
        response.message = "Odometry reset"
        return response

    def _integrate_pose(self, command: DriveCommand) -> None:
        now = time.time()
        dt = max(0.0, min(0.2, now - self.last_pose_update))
        self.last_pose_update = now
        if dt <= 0.0:
            return

        v_forward = command.forward * self.max_linear_speed_mps
        v_strafe = command.strafe * self.max_linear_speed_mps
        omega = command.yaw * self.max_angular_speed_radps

        field_vx = (v_forward * math.cos(self.pose_theta_rad)) - (v_strafe * math.sin(self.pose_theta_rad))
        field_vy = (v_forward * math.sin(self.pose_theta_rad)) + (v_strafe * math.cos(self.pose_theta_rad))

        self.pose_x_m = max(0.0, min(self.field_width_m, self.pose_x_m + field_vx * dt))
        self.pose_y_m = max(0.0, min(self.field_height_m, self.pose_y_m + field_vy * dt))
        self.pose_theta_rad = (self.pose_theta_rad + omega * dt) % (2.0 * math.pi)

    def _reset_pose(self) -> None:
        self.pose_x_m = self.field_width_m / 2.0
        self.pose_y_m = self.field_height_m / 2.0
        self.pose_theta_rad = 0.0
        self.last_pose_update = time.time()

    def _apply_drive(self, motor_speeds: List[float]) -> None:
        if self.bridge:
            try:
                if not self.bridge.set_drive_motors(motor_speeds):
                    self.get_logger().warning("Failed to send drive command to MCU")
            except Exception as exc:
                self.get_logger().warning("MCU drive command failed: %s", exc)
        else:
            try:
                self.motor_controller.set_speeds(motor_speeds)
            except Exception as exc:
                self.get_logger().warning("Local motor command failed: %s", exc)

    def _stop_drive(self) -> None:
        if self.bridge:
            try:
                drive_ok = self.bridge.set_drive_motors(ZERO_MOTOR_SPEEDS)
                mech_ok = self.bridge.set_mech_motors([0.0, 0.0, 0.0])
                if not drive_ok:
                    self.get_logger().warning("Failed to send zero drive command to MCU")
                if not mech_ok:
                    self.get_logger().warning("Failed to send zero mechanism command to MCU")
            except Exception as exc:
                self.get_logger().warning("Drive stop failed: %s", exc)
        else:
            self.motor_controller.stop()

    def _all_stop(self) -> None:
        self.connection_lost = True
        self.robot_mode = "STOPPED"
        self.last_drive_command = DriveCommand()
        self._stop_drive()

    def _watchdog_tick(self) -> None:
        if (time.time() - self.last_command_time) > self.heartbeat_timeout_s:
            if not self.connection_lost:
                self.get_logger().warning("Watchdog timeout hit, stopping robot")
                self._all_stop()

    def _telemetry_tick(self) -> None:
        telemetry = self._build_telemetry()
        telemetry_msg = String()
        telemetry_msg.data = json.dumps(telemetry, separators=(",", ":"))
        self.telemetry_pub.publish(telemetry_msg)

        self._publish_odometry()
        self._publish_diagnostics(telemetry)

    def _build_telemetry(self) -> Dict[str, Any]:
        if self.bridge:
            try:
                mcu = self.bridge.get_latest_telemetry()
            except Exception as exc:
                self.get_logger().warning("Failed to read MCU telemetry: %s", exc)
                mcu = {"bridge_connected": False}
        else:
            mcu = {"bridge_connected": False}

        return {
            "timestamp": time.time(),
            "mode": self.robot_mode,
            "odometry_mode": self.odometry_mode,
            "field": {
                "width_m": self.field_width_m,
                "height_m": self.field_height_m,
            },
            "pose": {
                "x": self.pose_x_m,
                "y": self.pose_y_m,
                "theta_deg": math.degrees(self.pose_theta_rad),
            },
            "motor_speeds": calculate_motor_speeds(self.last_drive_command),
            "mcu": mcu,
            "encoders": mcu.get("encoders", []),
            "relay": mcu.get("relay", 0),
            "connection_lost": self.connection_lost,
        }

    def _publish_odometry(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.pose_x_m
        msg.pose.pose.position.y = self.pose_y_m
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.pose_theta_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.pose_theta_rad / 2.0)
        msg.twist.twist.linear.x = self.last_drive_command.forward * self.max_linear_speed_mps
        msg.twist.twist.linear.y = self.last_drive_command.strafe * self.max_linear_speed_mps
        msg.twist.twist.angular.z = self.last_drive_command.yaw * self.max_angular_speed_radps
        self.odom_pub.publish(msg)

    def _publish_diagnostics(self, telemetry: Dict[str, Any]) -> None:
        status = DiagnosticStatus()
        status.name = "ksu_storm_robot"
        status.hardware_id = "ksu_storm"
        status.level = DiagnosticStatus.ERROR if telemetry["connection_lost"] else DiagnosticStatus.OK
        status.message = "watchdog_timeout" if telemetry["connection_lost"] else "running"
        status.values = [
            KeyValue(key="mode", value=str(self.robot_mode)),
            KeyValue(key="odometry_mode", value=str(self.odometry_mode)),
            KeyValue(key="bridge_connected", value=str(telemetry["mcu"].get("bridge_connected", False))),
        ]

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status = [status]
        self.diagnostics_pub.publish(diag)

    def destroy_node(self) -> bool:
        try:
            self._stop_drive()
        except Exception:
            pass

        if self.bridge:
            try:
                self.bridge.close()
            except Exception:
                pass

        return super().destroy_node()


def main(args: List[str] | None = None) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        LOGGER.info("ROS 2 robot node shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()
