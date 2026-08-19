# === IMPORTS === #
import sensor, time
import const
from PID import PID
from helpers import set_led, calc_error_lane, get_location
from helpers import calc_servo_pw
from machine import LED, PWM, Pin, Timer

from math import isnan

# === GLOBAL VARIABLES (FOR SPEED CONTROL) === #
count = 0
flag = 0

# === CAMERA SETUP === #
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# === PWM SETUP === #
ina = Pin("P10", Pin.OUT)
ina.value(1)
inb = Pin("P9", Pin.OUT)
inb.value(0)

servo_duty_cycle_factor = round((const.SERVO_DUTY_CYCLE_PERCENTAGE / 100)
                                 * const.DUTY_CYCLE_FACTOR)
servo = PWM("P2", freq=const.SERVO_FREQ, duty_u16=servo_duty_cycle_factor)

motor_duty_cycle_factor = round((const.MOTOR_DUTY_CYCLE_PERCENTAGE / 100)
                                 * const.DUTY_CYCLE_FACTOR)
motor = PWM("P8", freq=const.MOTOR_FREQ, duty_u16=motor_duty_cycle_factor)


# === IR SENSOR SETUP === #
pin = Pin("P5", Pin.IN, Pin.PULL_UP)
gnd_pin = Pin("P6", Pin.OUT)
gnd_pin.value(0)

# Define callback/handler for Pin ISR.
def isr(p):
    global count
    if (not flag):
        count += 1

# Define callback/handler for Timer ISR.
def tick(t):
    global flag
    flag = 1

# Define custom rounding function for clamping (to prevent servo oscillations)
def myRound(x, base):
    return int(base * round(float(x)/base))

# Define clamping function for servo angle.
def servoClamp(x):
    #if (abs(x) < 5): return 0
    #if (abs(x) < 10): return myRound(x, 2)

    return max(min(x, 45), -45)

# Configure interrupt handler for Pin.
pin.irq(handler=isr, trigger=Pin.IRQ_FALLING)

# Initializing Timer that has a period of TIMER_PERIOD milliseconds.
# (The Timer object will trigger an interrupt every TIMER_PERIOD milliseconds).
tim = Timer(-1, mode=Timer.PERIODIC, period=const.TIMER_PERIOD, callback=tick)


# === LED SETUP === #
led_red = LED("LED_RED")
led_green = LED("LED_GREEN")
led_blue = LED("LED_BLUE")
led_list = [led_red, led_green, led_blue]

# === CLOCK === #
clock = time.clock()

# === PID SETUP === #
p_i_d_steering = PID(p=3.0, i=0.0, d=0.0, i_max=25, i_resetting=False) # For MOTOR_VERY_SLOW
p_i_d_motor    = PID(p=1.0, i=0.0, d=0.0, i_max=500, i_resetting=False)

# === MAIN PROGRAM === #
steering_pw = const.STEERING_CENTER
base_motor_pw = const.MOTOR_VERY_SLOW
target_rpm = const.RPM_VERY_SLOW
current_rpm = const.RPM_VERY_SLOW

servo.duty_ns(const.STEERING_CENTER)
motor.duty_ns(const.MOTOR_BRAKE)
time.sleep(5)

# === MAIN LOOP === #
while True:
    ## Calculate current rpm, when flag is set. ##
    if (flag):
        gear_rots_per_min = (count / const.NUM_SPOTS_ON_GEAR) * (1000 / const.TIMER_PERIOD)
        current_rpm = gear_rots_per_min / const.GEAR_ROTS_TO_WHEEL_ROT

        # Setting a variable to 0 is always individually atomic in MicroPython.
        count = 0
        flag = 0    # This statement MUST be last, to avoid a critical section!


    clock.tick()
    img = sensor.snapshot().binary([const.THRESHOLD])

    # Initializing lane detection values. Lane center is the average of the two lanes,
    # frame center is the middle of the camera.
    lane_center = [0,0]
    error_steering = 0
    turning_angle = 0

    # Get lines in image
    lines = img.find_lines(merge_distance=1, max_theta_diff=15)

    left_lines = []
    right_lines = []

    # === Filter & Classify lines ===
    for line in lines:
        if not ((0 <= line.theta() <= 45) or (135 <= line.theta() <= 180)): continue
        img.draw_line(line.line(), color=127)  # Draw the detected line
        location = get_location(line)
        if location == "Left":
            left_lines.append(line)
        elif location == "Right":
            right_lines.append(line)

    # === If both lines are found, compute center ===
    if left_lines and right_lines:
        # Take average x and y-positions from each group
        left_xs = [(l.x1() + l.x2()) // 2 for l in left_lines]
        right_xs = [(l.x1() + l.x2()) // 2 for l in right_lines]

        left_ys = [(l.y1() + l.y2()) // 2 for l in left_lines]
        right_ys = [(l.y1() + l.y2()) // 2 for l in right_lines]

        left_mean_x = sum(left_xs) // len(left_xs)
        right_mean_x = sum(right_xs) // len(right_xs)

        left_mean_y = sum(left_ys) // len(left_ys)
        right_mean_y = sum(right_ys) // len(right_ys)

        lane_center[0] = (left_mean_x + right_mean_x) // 2
        lane_center[1] = (left_mean_y + right_mean_y) // 2

        error_steering = calc_error_lane(lane_center)
        turning_angle = servoClamp(p_i_d_steering.get_pid(error_steering)) # Clamp angle.
        steering_pw = calc_servo_pw(turning_angle)

        set_led("green", led_list)

    else:
        # Lane not detected fully
        steering_pw = const.STEERING_CENTER
        set_led(None, led_list)

    # Compare current rpm with target rpm, to get error for speed.
    # Then, run error for speed through a PID, to output adjustment motor pw.
    # Finally, add adjustment to "base pw", for the target speed.

    # error_motor = target_rpm - current_rpm
    # pw_adjust = min(max(p_i_d_motor.get_pid(error_motor), -5000), 5000) # Clamp pw adjustment.

    #if pw_adjust is None or isnan(pw_adjust):
    #    pw_adjust = 0
    pw_adjust = 0
    motor_pw = base_motor_pw + pw_adjust

    # === Apply PWM ===
    servo.duty_ns(steering_pw)
    # motor.duty_ns(int(motor_pw)) # changed to int
    motor.duty_ns(motor_pw)


    # === Debug Print ===
    print(f"""
        Lane Center: {lane_center},
        Error_Steering: {error_steering}, Turning: {turning_angle:.2f},
        RPM: {current_rpm}, Target RPM: {target_rpm},
        PWM Servo: {steering_pw}, Motor: {motor_pw}, FPS: {clock.fps():.2f}
    """)
