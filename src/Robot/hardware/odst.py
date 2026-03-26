
"""Basic reader for the SparkFun Qwiic OTOS optical odometry sensor."""

from __future__ import annotations

import sys
import time
from typing import Optional

from optical_odometry_sensor import OpticalOdometrySensor

CALIBRATION_COUNTDOWN_SECONDS = 5
READ_INTERVAL_SECONDS = 0.5


def create_sensor() -> Optional[OpticalOdometrySensor]:
    """Create and validate the OTOS sensor connection."""
    sensor = OpticalOdometrySensor()
    if not sensor.connect():
        print(
            "The device is not connected to the system. Please check your connection.",
            file=sys.stderr,
        )
        return None
    return sensor


def calibrate_sensor(sensor: OpticalOdometrySensor) -> None:
    """Run calibration and reset tracking to start from a known state."""
    print("Ensure the OTOS is flat and stationary during calibration!")
    for seconds_remaining in range(CALIBRATION_COUNTDOWN_SECONDS, 0, -1):
        print(f"Calibrating in {seconds_remaining} seconds...")
        time.sleep(1)

    print("Calibrating IMU...")
    sensor.calibrate(countdown_seconds=0)


def print_position(sensor: OpticalOdometrySensor) -> None:
    """Read and print the current sensor position."""
    pose = sensor.read_pose()
    print()
    print("Position:")
    print(f"X (Inches): {pose['x_in']}")
    print(f"Y (Inches): {pose['y_in']}")
    print(f"Heading (Degrees): {pose['heading_deg']}")


def run_example() -> int:
    """Initialize, calibrate, and continuously stream OTOS position."""
    print("\nQwiic OTOS Example 1 - Basic Readings\n")
    sensor = create_sensor()
    if sensor is None:
        return 1

    calibrate_sensor(sensor)

    while True:
        print_position(sensor)
        time.sleep(READ_INTERVAL_SECONDS)


if __name__ == "__main__":
    try:
        sys.exit(run_example())
    except KeyboardInterrupt:
        print("\nEnding Example")
        sys.exit(0)
