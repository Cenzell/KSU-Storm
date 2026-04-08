from __future__ import annotations

from typing import Any, Dict


AUTO_ROUTINES: Dict[str, Dict[str, Any]] = {
    "do_nothing": {
        "label": "Do Nothing",
        "description": "Stay still for the full autonomous period.",
        "routine": {
            "name": "do_nothing",
            "steps": [
                {"action": "wait", "name": "wait_start", "duration_s": 1.0, "timeout_s": 2.0},
            ],
        },
    },
    "drive_forward_1m": {
        "label": "Drive Forward 1 m",
        "description": "Drive straight ahead one meter while holding heading.",
        "routine": {
            "name": "drive_forward_1m",
            "steps": [
                {"action": "set_odometry_mode", "name": "use_hybrid_odometry", "mode": "HYBRID"},
                {"action": "drive_distance", "name": "forward_1m", "forward_m": 1.0, "speed": 0.35, "timeout_s": 5.0},
            ],
        },
    },
    "turn_180": {
        "label": "Turn To 180",
        "description": "Rotate in place to face 180 degrees field heading.",
        "routine": {
            "name": "turn_180",
            "steps": [
                {"action": "set_odometry_mode", "name": "use_hybrid_odometry", "mode": "HYBRID"},
                {"action": "turn_to_angle", "name": "turn_180", "angle_deg": 180.0, "speed": 0.3, "timeout_s": 4.0},
            ],
        },
    },
    "drive_turn_drive": {
        "label": "Drive, Turn, Drive",
        "description": "Drive forward, rotate, then drive again as a basic multi-step demo.",
        "routine": {
            "name": "drive_turn_drive",
            "steps": [
                {"action": "set_odometry_mode", "name": "use_hybrid_odometry", "mode": "HYBRID"},
                {"action": "drive_distance", "name": "forward_start", "forward_m": 1.0, "speed": 0.35, "timeout_s": 5.0},
                {"action": "turn_to_angle", "name": "turn_to_180", "angle_deg": 180.0, "speed": 0.25, "timeout_s": 4.0},
                {"action": "wait", "name": "settle", "duration_s": 0.4, "timeout_s": 1.0},
                {"action": "drive_distance", "name": "forward_finish", "forward_m": 0.5, "speed": 0.3, "timeout_s": 4.0},
            ],
        },
    },
}


def routine_keys() -> list[str]:
    return list(AUTO_ROUTINES.keys())


def routine_label(key: str) -> str:
    config = AUTO_ROUTINES.get(key, {})
    return str(config.get("label", key))


def routine_description(key: str) -> str:
    config = AUTO_ROUTINES.get(key, {})
    return str(config.get("description", ""))


def routine_payload(key: str) -> Dict[str, Any]:
    config = AUTO_ROUTINES.get(key)
    if config is None:
        raise KeyError(f"Unknown auto routine: {key}")
    return dict(config["routine"])
