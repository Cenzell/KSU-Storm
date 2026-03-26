from __future__ import annotations

import os
from time import time
from typing import Optional

from gpiozero import OutputDevice, PhaseEnableMotor, RotaryEncoder, Servo

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
except Exception:
    board = None
    busio = None
    PCA9685 = None


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def _is_truthy_env(var_name: str, default: str = "0") -> bool:
    return os.environ.get(var_name, default).strip().lower() not in ("0", "false", "no", "off")


PWM_BACKEND = os.environ.get("KSU_PWM_BACKEND", "pi").strip().lower()
USE_PCA9685 = PWM_BACKEND in ("pca", "pca9685")
PCA9685_ADDRESS = int(os.environ.get("KSU_PCA9685_ADDRESS", "0x40"), 0)
PCA9685_FREQUENCY_HZ = int(os.environ.get("KSU_PCA9685_FREQUENCY_HZ", "1000"))
PCA9685_INVERT_DIR = _is_truthy_env("KSU_PCA9685_INVERT_DIR", "0")

_pca_controller: Optional[PCA9685] = None


def _get_pca_controller() -> PCA9685:
    global _pca_controller
    if _pca_controller is not None:
        return _pca_controller

    if board is None or busio is None or PCA9685 is None:
        raise RuntimeError(
            "PCA9685 backend selected but dependencies are missing. "
            "Install adafruit-circuitpython-pca9685 and Blinka."
        )

    i2c = busio.I2C(board.SCL, board.SDA)
    controller = PCA9685(i2c, address=PCA9685_ADDRESS)
    controller.frequency = PCA9685_FREQUENCY_HZ
    _pca_controller = controller
    return _pca_controller


class PwmMotor:
    def __init__(self, pwm_pin: int, dir_pin: int, is_pwm: bool = True):
        self.backend = PWM_BACKEND
        self._motor: Optional[PhaseEnableMotor] = None
        self._dir: Optional[OutputDevice] = None
        self._pwm_channel = None
        self._invert_dir = PCA9685_INVERT_DIR

        if USE_PCA9685:
            if pwm_pin < 0 or pwm_pin > 15:
                raise ValueError(f"PCA9685 channel must be in [0, 15], got {pwm_pin}")
            controller = _get_pca_controller()
            self._pwm_channel = controller.channels[pwm_pin]
            self._dir = OutputDevice(dir_pin)
        else:
            self._motor = PhaseEnableMotor(phase=dir_pin, enable=pwm_pin, pwm=is_pwm)

    def set_speed(self, speed: float) -> None:
        """Set motor command in [-1.0, 1.0]."""
        speed = _clamp_unit(speed)
        if USE_PCA9685:
            if self._pwm_channel is None or self._dir is None:
                return
            forward = speed >= 0.0
            self._dir.value = (not forward) if self._invert_dir else forward
            self._pwm_channel.duty_cycle = int(abs(speed) * 0xFFFF)
            return

        if self._motor is not None:
            self._motor.value = speed

    def full_forward(self) -> None:
        self.set_speed(1.0)

    def full_backward(self) -> None:
        self.set_speed(-1.0)


class EncoderMotor:
    def __init__(self, pwm_pin: int, dir_pin: int, enc_a: int, enc_b: int):
        self.motor: PhaseEnableMotor = PhaseEnableMotor(phase=dir_pin, enable=pwm_pin)
        self.encoder: RotaryEncoder = RotaryEncoder(enc_a, enc_b, max_steps=0)
        self.encoder.steps = 0
        self.min_position: Optional[int] = None
        self.max_position: Optional[int] = None
        self.position_tolerance = 20
        self.target_position: Optional[int] = None
        self.default_speed = 0.4

    def set_speed(self, speed: float) -> None:
        self.motor.value = _clamp_unit(speed)

    def get_current_position(self) -> int:
        return int(self.encoder.steps)

    def set_min_position(self, min_position: int) -> None:
        self.min_position = int(min_position)

    def set_max_position(self, max_position: int) -> None:
        self.max_position = int(max_position)

    def min_pos(self) -> None:
        if self.min_position is None:
            raise ValueError("Minimum position not set")
        self.move_to_position(self.min_position)

    def max_pos(self) -> None:
        if self.max_position is None:
            raise ValueError("Maximum position not set")
        self.move_to_position(self.max_position)

    def move_to_position(self, target_position: int, speed: Optional[float] = None) -> None:
        if self.min_position is not None and target_position < self.min_position:
            raise ValueError("Target position is less than minimum position")
        if self.max_position is not None and target_position > self.max_position:
            raise ValueError("Target position is greater than maximum position")

        self.target_position = int(target_position)
        self._update_movement(self.default_speed if speed is None else speed)

    def move_steps(self, steps: int, speed: Optional[float] = None) -> None:
        self.target_position = self.get_current_position() + int(steps)
        if self.min_position is not None and self.target_position < self.min_position:
            raise ValueError("Target position is less than minimum position")
        if self.max_position is not None and self.target_position > self.max_position:
            raise ValueError("Target position is greater than maximum position")

        self._update_movement(self.default_speed if speed is None else speed)

    def _update_movement(self, speed: float) -> None:
        current_position = self.get_current_position()
        if self.target_position is None:
            self.set_speed(0.0)
            return

        distance_to_target = current_position - self.target_position
        if abs(distance_to_target) <= self.position_tolerance:
            self.set_speed(0.0)
            return

        direction = 1.0 if distance_to_target > 0 else -1.0
        self.set_speed(direction * speed)

    async def update(self) -> None:
        self._update_movement(self.default_speed)


class ServoMotor:
    def __init__(self, pin: int):
        self.servo: Servo = Servo(pin)
        self.end_time = 0.0
        self.stop()

    def stop(self) -> None:
        self.servo.value = None

    def set_min(self) -> None:
        self.servo.value = -1.0

    def set_max(self) -> None:
        self.servo.value = 0.0

    def set_value(self, value: float) -> None:
        self.servo.value = _clamp_unit(value)

    def move(self, speed: float, length: float) -> None:
        self.servo.value = _clamp_unit(speed)
        self.end_time = time() + float(length)

    async def update(self) -> None:
        if time() >= self.end_time:
            self.stop()
