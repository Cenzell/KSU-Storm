import os


def _is_truthy_env(var_name: str, default: str) -> bool:
    return os.environ.get(var_name, default).strip().lower() not in ("0", "false", "no")


COMMAND_PORT = 5555
TELEMETRY_PORT = 5556
TELEMETRY_RATE_HZ = 10
HEARTBEAT_TIMEOUT_S = 2.5
WATCHDOG_CHECK_INTERVAL_S = 0.1
MAX_LINEAR_SPEED_MPS = 1.2
MAX_ANGULAR_SPEED_DPS = 180.0
FIELD_WIDTH_M = 4.826
FIELD_HEIGHT_M = 2.438
ROBOT_FOOTPRINT_SIZE_M = 18.0 * 0.0254

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

JOYSTICK_DEADBAND = 0.06
INPUT_EXPO = 1.4
JOYSTICK_Y_SIGN = float(os.environ.get("KSU_JOYSTICK_Y_SIGN", "1.0"))

ZERO_MOTOR_SPEEDS = [0.0, 0.0, 0.0, 0.0]
VALID_ROBOT_MODES = {"AUTO", "TELEOP", "STOPPED"}
VALID_ODOMETRY_MODES = {"OPTICAL", "MOTOR", "HYBRID", "PRE_START"}
VALID_ALLIANCES = {"RED", "BLUE"}
