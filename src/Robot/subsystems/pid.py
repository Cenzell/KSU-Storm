"""Generic single-axis PID loop shared by mechanism subsystems (elevator, arm, ...)."""

from __future__ import annotations

import time


class PIDLoop:
    """Single-axis PID loop with independent integrator state."""

    def __init__(self) -> None:
        self._integral: float = 0.0
        self._last_error: float = 0.0
        self._last_time: float = 0.0
        self.current_pos: float = 0.0
        self.last_output: float = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = 0.0
        self.last_output = 0.0

    def compute(self, current_pos: float, setpoint: float,
                kp: float, ki: float, kd: float,
                max_output: float, decel_zone: float) -> float:
        self.current_pos = current_pos
        now = time.time()
        dt = now - self._last_time if self._last_time else 0.02
        dt = max(0.001, min(dt, 0.5))
        self._last_time = now

        error = setpoint - current_pos
        self._integral += error * dt

        # Anti-windup: keep integral contribution within ±max_output
        if ki > 0:
            self._integral = max(-max_output / ki, min(max_output / ki, self._integral))

        derivative = (error - self._last_error) / dt
        self._last_error = error

        raw = kp * error + ki * self._integral + kd * derivative

        # Deceleration zone: cap output proportionally as we near the setpoint.
        # This prevents arriving at full speed and overshooting.
        if decel_zone > 0:
            proximity = min(1.0, abs(error) / decel_zone)
            effective_max = max(0.05, max_output * proximity)
        else:
            effective_max = max_output

        self.last_output = max(-effective_max, min(effective_max, raw))
        return self.last_output
