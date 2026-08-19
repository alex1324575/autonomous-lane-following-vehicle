"""Vehicle-specific configuration for the OpenMV lane-following firmware.

All pulse widths are in nanoseconds. Values are recovered from the final
capstone iteration and must be verified against the actual vehicle before use.
"""

# Camera and image processing
FRAME_WIDTH = 80                 # OpenMV QQQVGA
FRAME_HEIGHT = 60
LANE_THRESHOLD = (215, 255)      # Light line on a darker track
ROI_HEIGHT = 30                  # Near-field region used for lane markers
BLOB_PIXELS_THRESHOLD = 7
BLOB_AREA_THRESHOLD = 7

# OpenMV pin mapping recovered from the final source revision
SERVO_PIN = "P2"
MOTOR_PWM_PIN = "P8"
MOTOR_INA_PIN = "P10"
MOTOR_INB_PIN = "P9"
IR_PIN = "P5"
IR_GROUND_PIN = "P6"

# PWM and actuator calibration
SERVO_FREQUENCY_HZ = 100
MOTOR_FREQUENCY_HZ = 1600
STEERING_LEFT_NS = 1_100_000
STEERING_CENTER_NS = 1_500_000
STEERING_RIGHT_NS = 1_900_000
MAX_STEERING_DEGREES = 45
STEERING_DEADBAND_DEGREES = 5

MOTOR_BRAKE_NS = 0
MOTOR_BASE_NS = 85_000
MOTOR_MIN_NS = 55_000
MOTOR_MAX_NS = 100_000

# Steering PID gains from the final archived main.py revision
STEERING_KP = 3.0
STEERING_KI = 0.0
STEERING_KD = 0.6
STEERING_INTEGRAL_LIMIT = 25.0

# Wheel-speed sensing and optional speed feedback
GEAR_MARKS = 3
GEAR_TO_WHEEL_RATIO = 1.0
RPM_WINDOW_MS = 250
TARGET_RPM = 40.0
ENABLE_SPEED_PID = False          # Enable only after recalibration
SPEED_KP = 1.0
SPEED_KI = 0.0
SPEED_KD = 0.0
SPEED_INTEGRAL_LIMIT = 5_000.0

# Safety and diagnostics
MAX_MISSED_LANE_FRAMES = 3
DEBUG = True

