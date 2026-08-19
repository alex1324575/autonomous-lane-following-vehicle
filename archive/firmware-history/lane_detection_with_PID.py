# === IMPORTS === #
import sensor, time
import const
from PID import PID
from helpers import set_led, calc_phi, get_location
from helpers import calc_servo_pw
from machine import LED, PWM, Pin

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
servo = PWM("P7", freq=const.SERVO_FREQ, duty_u16=servo_duty_cycle_factor)

motor_duty_cycle_factor = round((const.MOTOR_DUTY_CYCLE_PERCENTAGE / 100)
                                 * const.DUTY_CYCLE_FACTOR)
motor = PWM("P8", freq=const.MOTOR_FREQ, duty_u16=motor_duty_cycle_factor)

# === LED SETUP === #
led_red = LED("LED_RED")
led_green = LED("LED_GREEN")
led_blue = LED("LED_BLUE")
led_list = [led_red, led_green, led_blue]

# === CLOCK === #
clock = time.clock()

# === PID SETUP === #
p_i_d = PID(p=1.0, i=0.0, d=0.0, i_max=25, i_resetting=False)

# === MAIN PROGRAM === #
count = 0
next_steering_pw = const.STEERING_CENTER
next_motor_pw = const.MOTOR_VERY_SLOW

servo.duty_ns(const.STEERING_CENTER)
motor.duty_ns(const.MOTOR_BRAKE)
time.sleep(5)

# === MAIN LOOP === #
while True:
    clock.tick()
    img = sensor.snapshot().binary([const.THRESHOLD])

    left_pos = None # initializing lane detection values. lane center is the average of the two lanes, frame center is the middle of the camera
    right_pos = None
    lane_center = None
    frame_center = img.width() // 2
    error = 0

    roi = (0, img.height() - 20, img.width(), 20)  # Bottom region of interest
    blobs = img.find_blobs([(255, 255)], roi=roi, pixels_threshold=30, area_threshold=30, merge=True)

    for b in blobs:
        cx = b.cx()
        img.draw_rectangle(b.rect(), color=127)
        img.draw_cross(cx, b.cy(), color=127)

        if cx < frame_center:
            if left_pos is None or cx > left_pos:
                left_pos = cx
        else:
            if right_pos is None or cx < right_pos:
                right_pos = cx

    if left_pos is not None and right_pos is not None:
        lane_center = (left_pos + right_pos) // 2
        error = frame_center - lane_center
        turning_angle = pid.get_pid(error)
        steering_pw = calc_servo_pw(turning_angle)

        set_led("green", led_list)
    else:
        # Lane not detected fully
        steering_pw = const.STEERING_CENTER
        set_led(None, led_list)

    # Drive forward slowly
    motor_pw = const.MOTOR_VERY_SLOW

    # === Apply PWM ===
    servo.duty_ns(steering_pw)
    motor.duty_ns(motor_pw)

    # === Debug Print ===
    print(f"""
        Left: {left_pos}, Right: {right_pos}, Lane Center: {lane_center}, Frame Center: {frame_center}
        Error: {error}, Turning: {turning_angle if left_pos and right_pos else 0:.2f}
        PWM Servo: {steering_pw}, Motor: {motor_pw}, FPS: {clock.fps():.2f}
    """)