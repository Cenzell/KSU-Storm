from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from constants import (
    AUTO_ANGLE_TOLERANCE_DEG,
    AUTO_ANGULAR_KP,
    AUTO_LINEAR_KP,
    AUTO_MAX_ANGULAR_COMMAND,
    AUTO_MAX_LINEAR_COMMAND,
    AUTO_POSITION_TOLERANCE_M,
    AUTO_STEP_TIMEOUT_S,
)


Pose = Dict[str, float]
AutoStep = Dict[str, Any]


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, float(value)))


def normalize_angle_deg(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def angle_error_deg(target_deg: float, current_deg: float) -> float:
    return normalize_angle_deg(float(target_deg) - float(current_deg))


def pose_from_values(x_m: float, y_m: float, theta_deg: float) -> Pose:
    return {
        "x": float(x_m),
        "y": float(y_m),
        "theta_deg": float(theta_deg),
    }


def _get_value(raw_step: Dict[str, Any], key: str, default: Any) -> Any:
    value = raw_step.get(key, default)
    return default if value is None else value


def parse_routine(name: str, raw_steps: Any) -> List[AutoStep]:
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("Auto routine must include a non-empty steps list")

    return [normalize_step(step, index) for index, step in enumerate(raw_steps)]


def normalize_step(raw_step: Any, index: int) -> AutoStep:
    if not isinstance(raw_step, dict):
        raise ValueError(f"Step {index} must be an object")

    action = str(raw_step.get("action", "")).strip().lower()
    if not action:
        raise ValueError(f"Step {index} is missing an action")

    timeout_s = float(_get_value(raw_step, "timeout_s", AUTO_STEP_TIMEOUT_S))
    if timeout_s <= 0:
        raise ValueError(f"Step {index} timeout must be positive")

    step: AutoStep = {
        "action": action,
        "timeout_s": timeout_s,
        "name": str(raw_step.get("name", action)),
    }

    if action == "wait":
        step["duration_s"] = float(_get_value(raw_step, "duration_s", _get_value(raw_step, "seconds", 1.0)))
        if step["duration_s"] < 0:
            raise ValueError(f"Step {index} wait duration cannot be negative")
        return step

    if action == "drive_distance":
        if "distance_m" in raw_step and "forward_m" not in raw_step:
            step["forward_m"] = float(raw_step["distance_m"])
        else:
            step["forward_m"] = float(_get_value(raw_step, "forward_m", 0.0))
        step["strafe_m"] = float(_get_value(raw_step, "strafe_m", 0.0))
        step["hold_heading_deg"] = raw_step.get("hold_heading_deg")
        step["speed"] = float(_get_value(raw_step, "speed", AUTO_MAX_LINEAR_COMMAND))
        step["turn_speed"] = float(_get_value(raw_step, "turn_speed", AUTO_MAX_ANGULAR_COMMAND))
        step["position_tolerance_m"] = float(_get_value(raw_step, "position_tolerance_m", AUTO_POSITION_TOLERANCE_M))
        step["angle_tolerance_deg"] = float(_get_value(raw_step, "angle_tolerance_deg", AUTO_ANGLE_TOLERANCE_DEG))
        return step

    if action == "turn_to_angle":
        if "angle_deg" not in raw_step:
            raise ValueError(f"Step {index} turn_to_angle requires angle_deg")
        step["angle_deg"] = float(raw_step["angle_deg"])
        step["speed"] = float(_get_value(raw_step, "speed", AUTO_MAX_ANGULAR_COMMAND))
        step["angle_tolerance_deg"] = float(_get_value(raw_step, "angle_tolerance_deg", AUTO_ANGLE_TOLERANCE_DEG))
        return step

    if action == "drive_to_pose":
        if "x_m" not in raw_step or "y_m" not in raw_step:
            raise ValueError(f"Step {index} drive_to_pose requires x_m and y_m")
        step["x_m"] = float(raw_step["x_m"])
        step["y_m"] = float(raw_step["y_m"])
        step["heading_deg"] = raw_step.get("heading_deg")
        if step["heading_deg"] is not None:
            step["heading_deg"] = float(step["heading_deg"])
        step["speed"] = float(_get_value(raw_step, "speed", AUTO_MAX_LINEAR_COMMAND))
        step["turn_speed"] = float(_get_value(raw_step, "turn_speed", AUTO_MAX_ANGULAR_COMMAND))
        step["position_tolerance_m"] = float(_get_value(raw_step, "position_tolerance_m", AUTO_POSITION_TOLERANCE_M))
        step["angle_tolerance_deg"] = float(_get_value(raw_step, "angle_tolerance_deg", AUTO_ANGLE_TOLERANCE_DEG))
        return step

    if action == "set_odometry_mode":
        step["mode"] = str(raw_step.get("mode", "PRE_START")).upper()
        return step

    if action == "reset_odometry":
        return step

    raise ValueError(f"Unsupported auto action: {action}")


def build_distance_target(start_pose: Pose, forward_m: float, strafe_m: float) -> Tuple[float, float]:
    heading_rad = math.radians(float(start_pose["theta_deg"]))
    field_dx = (float(forward_m) * math.cos(heading_rad)) - (float(strafe_m) * math.sin(heading_rad))
    field_dy = (float(forward_m) * math.sin(heading_rad)) + (float(strafe_m) * math.cos(heading_rad))
    return float(start_pose["x"]) + field_dx, float(start_pose["y"]) + field_dy


def compute_drive_to_pose_command(
    current_pose: Pose,
    target_x_m: float,
    target_y_m: float,
    target_heading_deg: Optional[float],
    max_linear_command: float,
    max_angular_command: float,
    position_tolerance_m: float,
    angle_tolerance_deg: float,
) -> Tuple[float, float, float, bool]:
    dx = float(target_x_m) - float(current_pose["x"])
    dy = float(target_y_m) - float(current_pose["y"])
    distance_error = math.hypot(dx, dy)

    heading_rad = math.radians(float(current_pose["theta_deg"]))
    forward_error = (dx * math.cos(heading_rad)) + (dy * math.sin(heading_rad))
    strafe_error = (-dx * math.sin(heading_rad)) + (dy * math.cos(heading_rad))

    ly = clamp(forward_error * AUTO_LINEAR_KP, -abs(max_linear_command), abs(max_linear_command))
    lx = clamp(strafe_error * AUTO_LINEAR_KP, -abs(max_linear_command), abs(max_linear_command))

    rx = 0.0
    angle_done = True
    if target_heading_deg is not None:
        heading_error = angle_error_deg(target_heading_deg, float(current_pose["theta_deg"]))
        rx = clamp(
            heading_error * AUTO_ANGULAR_KP,
            -abs(max_angular_command),
            abs(max_angular_command),
        )
        angle_done = abs(heading_error) <= float(angle_tolerance_deg)

    position_done = distance_error <= float(position_tolerance_m)
    done = position_done and angle_done

    if done:
        return 0.0, 0.0, 0.0, True

    if position_done:
        lx = 0.0
        ly = 0.0

    return lx, ly, rx, False


def compute_turn_command(
    current_heading_deg: float,
    target_heading_deg: float,
    max_angular_command: float,
    angle_tolerance_deg: float,
) -> Tuple[float, bool]:
    heading_error = angle_error_deg(target_heading_deg, current_heading_deg)
    if abs(heading_error) <= float(angle_tolerance_deg):
        return 0.0, True

    rx = clamp(
        heading_error * AUTO_ANGULAR_KP,
        -abs(max_angular_command),
        abs(max_angular_command),
    )
    return rx, False
