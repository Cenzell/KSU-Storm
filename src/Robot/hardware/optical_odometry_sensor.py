"""Reusable wrapper for the SparkFun Qwiic OTOS sensor."""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

import qwiic_otos


class OpticalOdometrySensor:
    """High-level helper around ``qwiic_otos.QwiicOTOS``."""

    def __init__(self, sensor: Optional[qwiic_otos.QwiicOTOS] = None):
        self.sensor = sensor if sensor is not None else qwiic_otos.QwiicOTOS()

    def connect(self) -> bool:
        if not self.sensor.is_connected():
            return False
        self.sensor.begin()
        return True

    def calibrate(self, countdown_seconds: int = 5, sleep_s: float = 1.0) -> None:
        for _ in range(max(0, int(countdown_seconds))):
            time.sleep(float(sleep_s))
        self.sensor.calibrateImu()
        self.sensor.resetTracking()

    def reset_tracking(self) -> None:
        self.sensor.resetTracking()

    def read_pose(self) -> Dict[str, float]:
        position: Any = self.sensor.getPosition()
        return {
            "x_in": float(position.x),
            "y_in": float(position.y),
            "heading_deg": float(position.h),
        }
