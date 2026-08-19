### === CONSTANTS === ###
# === CONSTANT PULSE WIDTH VALUES (in nanoseconds, assuming 100 Hz frequency) === #
STEERING_CENTER = 1500000  # Neutral steering
STEERING_LEFT   = 1100000  # Max left turn
STEERING_RIGHT  = 1900000  # Max right turn

MOTOR_FORWARD     = 1650000    # Full forward throttle. (TODO)
MOTOR_MEDIUM      = 1600000    # Medium speed. (TODO)
MOTOR_MEDIUM_SLOW = 92500    # Medium slow speed. (TODO)
MOTOR_SLOW        = 85000     # Slower speed for initial full-run tests.
MOTOR_VERY_SLOW   = 60000      # Very slow speed for debugging.
MOTOR_BRAKE       = 0     # Brake motor.

# === CONSTANT TARGET RPM VALUES === # (Needs to be found through experimentation.)
RPM_FORWARD     = 100   # Full forward throttle. (TODO)
RPM_MEDIUM      = 80    # Medium rpm. (TODO)
RPM_MEDIUM_SLOW = 60    # Medium slow rpm. (TODO)
RPM_SLOW        = 40    # Slower rpm for initial full-run tests. (TODO)
RPM_VERY_SLOW   = 20    # Very slow rpm for debugging. (TODO)
RPM_BRAKE       = 0     # Brake Motor rpm.

# === CONSTANTS FOR IR SENSOR CALCULATIONS === #
NUM_SPOTS_ON_GEAR = 3
GEAR_ROTS_TO_WHEEL_ROT = 1
TIMER_PERIOD = 250 # in milliseconds

# === CONSTANTS FOR DUTY CYCLE AND FREQUENCY === #
DUTY_CYCLE_FACTOR = 65535

SERVO_DUTY_CYCLE_PERCENTAGE = 50    # Desired Duty Cycle for Servo (in %)
MOTOR_DUTY_CYCLE_PERCENTAGE = 50    # Desired Duty Cycle for Motor (in %)

SERVO_FREQ = 100    # Desired frequency for servo PWM (in Hz)
MOTOR_FREQ = 1600     # Desired frequency for motor PWN (in Hz)

# === CONSTANTS FOR PID CALCULATION (not Kp, Kd, and Ki values) === #
US_TO_SEC_MULTPLIER = 1000000

D_BUFF_SIZE = 4            # Size of circular buffer used for D-term.


# === CONSTANTS FOR IMAGE === #
FRAME_WIDTH = 80  # QQQVGA width
FRAME_HEIGHT = 60  # QQQVGA height
CENTER_REGION = (FRAME_WIDTH // 3, 2 * FRAME_WIDTH // 3)  # Center Third
CENTER_POSITION = FRAME_WIDTH / 2


# === LINE COLOR THRESHOLD CONSTANTS (white line on darker surface) ===
THRESHOLD = (215, 255)


# === CONSTANT FOR ERROR COLLECTION IN PID === #
TIME_DELTA = 15000   # in microseconds.
