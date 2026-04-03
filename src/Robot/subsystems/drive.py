from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import List

from constants import (
    INPUT_EXPO,
    JOYSTICK_DEADBAND,
    MOTOR_DIRECTION_MULTIPLIER,
    MOTOR_PIN_MAP,
    USE_PCA9685_PWM,
    ZERO_MOTOR_SPEEDS,
)

try:
    from hardware import PwmMotor
except Exception:
    PwmMotor = None


logger = logging.getLogger(__name__)


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def _apply_deadband(value: float, deadband: float) -> float:
    value = float(value)
    if abs(value) < deadband:
        return 0.0

    sign = 1.0 if value >= 0.0 else -1.0
    scaled = (abs(value) - deadband) / (1.0 - deadband)
    return sign * scaled


def _shape_input(value: float, expo: float) -> float:
    value = _clamp_unit(value)
    sign = 1.0 if value >= 0.0 else -1.0
    return sign * (abs(value) ** expo)


@dataclass
class JoystickData:
    """Normalized joystick axes in the [-1.0, 1.0] range."""

    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0

    def __post_init__(self) -> None:
        self.lx = _clamp_unit(self.lx)
        self.ly = _clamp_unit(self.ly)
        self.rx = _clamp_unit(self.rx)
        self.ry = _clamp_unit(self.ry)


def calculate_motor_speeds(data: JoystickData) -> List[float]:
    """Calculate normalized mecanum motor speeds in [FL, FR, RL, RR] order."""
    x = _shape_input(_apply_deadband(data.lx, JOYSTICK_DEADBAND), INPUT_EXPO)
    y = _shape_input(_apply_deadband(data.ly, JOYSTICK_DEADBAND), INPUT_EXPO)
    z = _shape_input(_apply_deadband(data.rx, JOYSTICK_DEADBAND), INPUT_EXPO)

    speeds = [
        y + x + z,  # Front Left
        y - x - z,  # Front Right
        y - x + z,  # Rear Left
        y + x - z,  # Rear Right
    ]

    max_speed = max(abs(speed) for speed in speeds)
    if max_speed > 1.0:
        speeds = [speed / max_speed for speed in speeds]

    return speeds


class MotorController:
    """Drive controller for 4 PWM+DIR channels (2x MDD10A)."""

    def __init__(self) -> None:
        self.available = PwmMotor is not None
        self.motors = []
        self.lock = threading.Lock()

        if not self.available:
            logger.warning(
                "Motor hardware unavailable (hardware.py / gpiozero import failed). "
                "Running in simulation mode."
            )
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


motor_controller: MotorController | None = None


def ensure_motor_controller() -> MotorController:
    global motor_controller
    if motor_controller is None:
        motor_controller = MotorController()
    return motor_controller


def set_motor_speeds(speeds: List[float]) -> None:
    """Set motor speeds in order [FL, FR, RL, RR], each in [-1.0, 1.0]."""
    controller = ensure_motor_controller()
    try:
        controller.set_speeds(speeds)
    except Exception as exc:
        logger.error("Failed to set motor speeds: %s", exc)
