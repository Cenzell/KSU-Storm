import os


def _is_truthy_env(var_name: str, default: str) -> bool:
    return os.environ.get(var_name, default).strip().lower() not in ("0", "false", "no")


COMMAND_PORT = 5555
TELEMETRY_PORT = 5556
TELEMETRY_RATE_HZ = 10
HEARTBEAT_TIMEOUT_S = 0.8   # §4.4.7 — must halt within 1 s of losing signal
WATCHDOG_CHECK_INTERVAL_S = 0.1
AUTO_LOOP_INTERVAL_S = 0.05
MAX_LINEAR_SPEED_MPS = 1.2
MAX_ANGULAR_SPEED_DPS = 180.0
FIELD_WIDTH_M = 4.826
FIELD_HEIGHT_M = 2.438
ROBOT_FOOTPRINT_SIZE_M = 18.0 * 0.0254
AUTO_LINEAR_KP = 1.6
AUTO_ANGULAR_KP = 0.025
AUTO_MAX_LINEAR_COMMAND = 0.45
AUTO_MAX_ANGULAR_COMMAND = 0.4
AUTO_POSITION_TOLERANCE_M = 0.05
AUTO_ANGLE_TOLERANCE_DEG = 4.0
AUTO_STEP_TIMEOUT_S = 8.0

ENABLE_CAMERA_BROADCAST = _is_truthy_env("KSU_ENABLE_CAMERA_BROADCAST", "1")
USE_PCA9685_PWM = os.environ.get("KSU_PWM_BACKEND", "pca").strip().lower() in ("pca", "pca9685")

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

# Arm motor encoder resolution.
# Set to actual ticks-per-revolution including gearbox.
# Example: 50:1 gearbox + 28 CPR encoder + quadrature = 50 * 28 * 4 = 5600
ARM_TICKS_PER_REV = 1440  # TODO: measure and update for your actual motor + gearbox

JOYSTICK_DEADBAND = 0.06
INPUT_EXPO = 1.4
JOYSTICK_Y_SIGN = float(os.environ.get("KSU_JOYSTICK_Y_SIGN", "1.0"))

ZERO_MOTOR_SPEEDS = [0.0, 0.0, 0.0, 0.0]
VALID_ROBOT_MODES = {"AUTO", "TELEOP", "STOPPED"}
VALID_ODOMETRY_MODES = {"OPTICAL", "MOTOR", "HYBRID", "PRE_START"}
VALID_ALLIANCES = {"RED", "BLUE"}
