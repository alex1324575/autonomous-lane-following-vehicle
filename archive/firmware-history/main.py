import sensor, time
import const
from PID import PID
from helpers import set_led, calc_error_lane
from helpers import calc_servo_pw
from machine import LED, PWM, Pin, Timer
from math import isnan
count = 0
flag = 0
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
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
pin = Pin("P5", Pin.IN, Pin.PULL_UP)
gnd_pin = Pin("P6", Pin.OUT)
gnd_pin.value(0)
def isr(p):
    global count
    if (not flag):
        count += 1
def tick(t):
    global flag
    flag = 1
def myRound(x, base):
    return int(base * round(float(x)/base))
def servoClamp(x):
    if (abs(x) < 5): return 0
    if (abs(x) < 10): return myRound(x, 1)
    return max(min(x, 45), -45)
pin.irq(handler=isr, trigger=Pin.IRQ_FALLING)
tim = Timer(-1, mode=Timer.PERIODIC, period=const.TIMER_PERIOD, callback=tick)
led_red = LED("LED_RED")
led_green = LED("LED_GREEN")
led_blue = LED("LED_BLUE")
led_list = [led_red, led_green, led_blue]
clock = time.clock()
p_i_d_steering = PID(p=3.0, i=0.0, d=0.6, i_max=25, i_resetting=False) ##change from p=2.5 and d=0.4
p_i_d_motor	= PID(p=1.0, i=0.0, d=0.0, i_max=500, i_resetting=False)
steering_pw = const.STEERING_CENTER
base_motor_pw = const.MOTOR_SLOW
target_rpm = const.RPM_SLOW
current_rpm = const.RPM_SLOW
servo.duty_ns(const.STEERING_CENTER)
motor.duty_ns(const.MOTOR_BRAKE)
time.sleep(5)
while True:
    if (flag):
        gear_rots_per_min = (count / const.NUM_SPOTS_ON_GEAR) * (1000 / const.TIMER_PERIOD)
        current_rpm = gear_rots_per_min / const.GEAR_ROTS_TO_WHEEL_ROT
        count = 0
        flag = 0
    clock.tick()
    img = sensor.snapshot().binary([const.THRESHOLD])
    left_pos = None
    right_pos = None
    turning_angle = 0
    lane_center = [0,0]
    frame_center = (const.FRAME_WIDTH // 2, const.FRAME_HEIGHT // 2)
    error_steering = 0
    error_motor = 0
    roi = (0, const.FRAME_HEIGHT - 30, const.FRAME_WIDTH, 20) ##changed frame_height from -20 to -30
    blobs = img.find_blobs([(255, 255)], roi=roi, pixels_threshold=7, area_threshold=7, merge=True)
    for b in blobs:
        cx = b.cx()
        cy = b.cy()
        img.draw_rectangle(b.rect(), color=127)
        img.draw_cross(cx, cy, color=127)
        if cx < frame_center[0]:
            if left_pos is None or cx > left_pos[0]:
                left_pos = (cx, cy)
        else:
            if right_pos is None or cx <= right_pos[0]:
                right_pos = (cx, cy)

    phi = 0.0

    if left_pos is not None and right_pos is not None:
        lane_center[0] = (left_pos[0] + right_pos[0]) // 2
        lane_center[1] = (left_pos[1] + right_pos[1]) // 2
        error_steering = calc_error_lane(lane_center)
        phi = p_i_d_steering.get_pid(error_steering)
        turning_angle = servoClamp(phi)
        steering_pw = calc_servo_pw(turning_angle)
        set_led("green", led_list)
    else:
        steering_pw = const.STEERING_CENTER
        set_led(None, led_list)
    pw_adjust = 0
    motor_pw = base_motor_pw + pw_adjust
    servo.duty_ns(steering_pw)
    motor.duty_ns(motor_pw)
    print(f"""
        Left: {left_pos}, Right: {right_pos}, Lane Center: {lane_center}, Frame Center: {frame_center}
        Error: {error_steering}, Phi: {phi:.2f}, Turning: {turning_angle:.2f},
        RPM: {current_rpm}, Target RPM: {target_rpm},
        PWM Servo: {steering_pw}, Motor: {motor_pw}, FPS: {clock.fps():.2f}
    """)
