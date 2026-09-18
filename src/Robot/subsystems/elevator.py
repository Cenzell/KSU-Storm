"""Dual-motor lift mechanism (e.g. an elevator/carriage), independent of any game.

Not every season's game needs a lift mechanism. This subsystem is kept
generic and game-agnostic so it can sit dormant (see ENABLE_ELEVATOR in
constants.py) in a season that doesn't use it, and be pointed at whatever
motor/encoder indices a future game's hardware uses without rewriting the
control logic.
"""

from __future__ import annotations

from typing import Optional, Sequence

from subsystems.pid import PIDLoop


class ElevatorSubsystem:
    """One PID loop driving two motors from their averaged encoder position.

    The two motors are assumed to move a single mechanism together (e.g. the
    left/right sides of a lift carriage); `right_encoder_sign` normalizes the
    right encoder's counting direction to match the left, and
    `right_motor_sign` corrects for the right motor being mounted/wired
    opposite the left.
    """

    TOLERANCE = 20.0  # ticks

    def __init__(
        self,
        left_motor_index: int,
        right_motor_index: int,
        left_encoder_index: int,
        right_encoder_index: int,
        right_encoder_sign: float = 1.0,
        right_motor_sign: float = -1.0,
        kp: float = 0.002,
        ki: float = 0.0,
        kd: float = 0.0,
        max_output: float = 0.6,
        decel_zone: float = 800.0,
    ) -> None:
        self.left_motor_index = left_motor_index
        self.right_motor_index = right_motor_index
        self.left_encoder_index = left_encoder_index
        self.right_encoder_index = right_encoder_index
        self.right_encoder_sign = right_encoder_sign
        self.right_motor_sign = right_motor_sign
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.decel_zone = decel_zone
        self.setpoint: float = 0.0
        self.active: bool = False
        self._loop = PIDLoop()

    def reset(self) -> None:
        self._loop.reset()

    def set_setpoint(self, setpoint: float) -> None:
        self.setpoint = float(setpoint)
        self.reset()
        self.active = True

    def disable(self) -> None:
        self.active = False
        self.reset()

    def _read_positions(self, encoders: Sequence[float]) -> Optional[tuple[float, float]]:
        if len(encoders) <= max(self.left_encoder_index, self.right_encoder_index):
            return None
        left_pos = float(encoders[self.left_encoder_index])
        right_pos = self.right_encoder_sign * float(encoders[self.right_encoder_index])
        return left_pos, right_pos

    def compute(self, encoders: Sequence[float]) -> float:
        """Return a single output in [-max_output, max_output] for both motors."""
        positions = self._read_positions(encoders)
        if positions is None:
            return 0.0

        left_pos, right_pos = positions
        avg_pos = (left_pos + right_pos) / 2.0
        if abs(self.setpoint - avg_pos) <= self.TOLERANCE:
            self._loop.last_output = 0.0
            self._loop.current_pos = avg_pos
            return 0.0
        return self._loop.compute(avg_pos, self.setpoint,
                                   self.kp, self.ki, self.kd,
                                   self.max_output, self.decel_zone)

    def apply(self, encoders: Sequence[float], mech_cmd: list[float]) -> None:
        """Write this mechanism's outputs into the shared mech command vector.

        No-op while inactive, leaving mech_cmd untouched so a manual-mode
        command on the same channels isn't overwritten.
        """
        if not self.active:
            return
        output = self.compute(encoders)
        mech_cmd[self.left_motor_index] = output
        mech_cmd[self.right_motor_index] = self.right_motor_sign * output

    def as_dict(self) -> dict:
        return {
            "active": self.active,
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
            "max_output": self.max_output,
            "decel_zone": self.decel_zone,
            "setpoint": self.setpoint,
            "current_pos": self._loop.current_pos,
            "output": self._loop.last_output,
        }
