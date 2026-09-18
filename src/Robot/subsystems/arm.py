"""Single-motor pivoting mechanism (e.g. an arm), independent of any game.

Not every season's game needs a pivoting arm. This subsystem is kept
generic and game-agnostic so it can sit dormant (see ENABLE_ARM in
constants.py) in a season that doesn't use it, and be pointed at whatever
motor/encoder index and gearing a future game's hardware uses without
rewriting the control logic.
"""

from __future__ import annotations

from typing import Optional, Sequence

from subsystems.pid import PIDLoop


class ArmSubsystem:
    """One PID loop for a single motor, with setpoints expressed in degrees.

    Degrees are converted to ticks using `ticks_per_rev`, which must match
    the installed motor + gearbox (e.g. 50:1 gearbox + 28 CPR encoder +
    quadrature = 50 * 28 * 4 = 5600).
    """

    TOLERANCE_DEG = 1.0  # degrees — within this the output is zeroed

    def __init__(
        self,
        motor_index: int,
        encoder_index: int,
        ticks_per_rev: float,
        kp: float = 0.01,
        ki: float = 0.0,
        kd: float = 0.0,
        max_output: float = 0.5,
        decel_zone_deg: float = 15.0,
    ) -> None:
        self.motor_index = motor_index
        self.encoder_index = encoder_index
        self.ticks_per_rev = ticks_per_rev
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.decel_zone_deg = decel_zone_deg
        self.setpoint_deg: float = 0.0
        self.active: bool = False
        self._loop = PIDLoop()

    def _ticks_to_deg(self, ticks: float) -> float:
        return ticks * (360.0 / self.ticks_per_rev)

    def _deg_to_ticks(self, deg: float) -> float:
        return deg * (self.ticks_per_rev / 360.0)

    def reset(self) -> None:
        self._loop.reset()

    def set_setpoint_degrees(self, degrees: float) -> None:
        self.setpoint_deg = float(degrees)
        self.reset()
        self.active = True

    def disable(self) -> None:
        self.active = False
        self.reset()

    def current_deg(self, encoders: Sequence[float]) -> float:
        if len(encoders) <= self.encoder_index:
            return 0.0
        return self._ticks_to_deg(float(encoders[self.encoder_index]))

    def compute(self, encoders: Sequence[float]) -> float:
        """Return motor output in [-max_output, max_output]."""
        if len(encoders) <= self.encoder_index:
            return 0.0

        current_deg = self.current_deg(encoders)
        error_deg = self.setpoint_deg - current_deg

        if abs(error_deg) <= self.TOLERANCE_DEG:
            self._loop.last_output = 0.0
            self._loop.current_pos = current_deg
            return 0.0

        decel_zone_ticks = self._deg_to_ticks(self.decel_zone_deg) if self.decel_zone_deg > 0 else 0.0
        return self._loop.compute(
            current_pos=self._deg_to_ticks(current_deg),
            setpoint=self._deg_to_ticks(self.setpoint_deg),
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            max_output=self.max_output,
            decel_zone=decel_zone_ticks,
        )

    def apply(self, encoders: Sequence[float], mech_cmd: list[float]) -> None:
        """Write this mechanism's output into the shared mech command vector.

        No-op while inactive, leaving mech_cmd untouched so a manual-mode
        command on the same channel isn't overwritten.
        """
        if not self.active:
            return
        mech_cmd[self.motor_index] = self.compute(encoders)

    def as_dict(self, encoders: Optional[Sequence[float]] = None) -> dict:
        current = self.current_deg(encoders or [])
        return {
            "active": self.active,
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
            "max_output": self.max_output,
            "decel_zone_deg": self.decel_zone_deg,
            "setpoint_deg": self.setpoint_deg,
            "current_deg": current,
            "output": self._loop.last_output,
        }
