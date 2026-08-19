import sensor, image, time
from machine import Pin, PWM
import pid, const, helpers
import pyb

# === Camera Setup ===
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time=2000)

# === PWM Setup ===
servo_pwm = PWM(Pin('P7'), freq=const.SERVO_FREQ)
motor_pwm = PWM(Pin('P8'), freq=const.MOTOR_FREQ)

# === LED Setup ===
red_led   = pyb.LED(1)
green_led = pyb.LED(2)
blue_led  = pyb.LED(3)
leds = [red_led, green_led, blue_led]

# === Set Neutral Initially ===
servo_pwm.duty_ns(const.STEERING_CENTER)
motor_pwm.duty_ns(const.MOTOR_BRAKE)

# === Instantiate PID Controller ===
pid_ctrl = pid.PID(p=0.7, i=0.03, d=0.1, i_max=30, i_resetting=False)

# === Main Loop ===
clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot().binary([const.THRESHOLD])

    # Get lines in image
    lines = img.find_lines(merge_distance=1, max_theta_diff=15)

    left_lines = []
    right_lines = []

    # === Filter & Classify lines ===
    for line in lines:
        if not (45 < abs(line.theta()) < 135):  # near vertical
            continue
        location = helpers.get_location(line)
        if location == "Left":
            left_lines.append(line)
        elif location == "Right":
            right_lines.append(line)

    # === If both lines are found, compute center ===
    if left_lines and right_lines:
        # Take average x-position from each group
        left_xs = [(l.x1() + l.x2()) // 2 for l in left_lines]
        right_xs = [(l.x1() + l.x2()) // 2 for l in right_lines]

        left_mean = sum(left_xs) // len(left_xs)
        right_mean = sum(right_xs) // len(right_xs)

        lane_center_x = (left_mean + right_mean) // 2
        error = const.CENTER_POSITION - lane_center_x

        # Calculate steering angle with your PID
        steering_angle = pid_ctrl.get_pid(error)

        # Convert to pulse widths
        servo_pw = helpers.calc_servo_pw(steering_angle)
        motor_pw = const.MOTOR_MEDIUM if abs(steering_angle) < 10 else const.MOTOR_SLOW

        # Apply PWM
        servo_pwm.duty_ns(servo_pw)
        motor_pwm.duty_ns(motor_pw)

        # LED feedback
        if abs(steering_angle) < 5:
            helpers.set_led("green", leds)
        elif steering_angle > 0:
            helpers.set_led("blue", leds)
        else:
            helpers.set_led("red", leds)

        print("Angle: {:.1f} | ServoPW: {} | MotorPW: {} | FPS: {:.1f}".format(
            steering_angle, servo_pw, motor_pw, clock.fps()))

    else:
        # One or both lane lines missing: stop
        servo_pwm.duty_ns(const.STEERING_CENTER)
        motor_pwm.duty_ns(const.MOTOR_BRAKE)
        helpers.set_led(None, leds)
        print("Lane lost — Stopping. FPS: {:.1f}".format(clock.fps()))
