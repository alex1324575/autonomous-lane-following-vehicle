"""OpenMV entry point for the autonomous lane-following vehicle.

Deploy this file with config.py, control.py, and pid.py to the OpenMV board.
Always calibrate with the vehicle lifted and confirm the lane-loss brake.
"""

import sensor
import time
from machine import LED, PWM, Pin, Timer

import config
from control import clamp, find_lane_center, rpm_from_edges, steering_error_degrees, steering_pulse_ns
from pid import PID


edge_count = 0
rpm_window_ready = False


def on_ir_edge(_pin):
    global edge_count
    if not rpm_window_ready:
        edge_count += 1


def on_rpm_window(_timer):
    global rpm_window_ready
    rpm_window_ready = True


def set_status(leds, color=None):
    for led in leds:
        led.off()
    if color == "red":
        leds[0].on()
    elif color == "green":
        leds[1].on()
    elif color == "blue":
        leds[2].on()


def initialize_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQQVGA)
    sensor.skip_frames(time=2_000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)


def main():
    global edge_count, rpm_window_ready

    initialize_camera()

    motor_ina = Pin(config.MOTOR_INA_PIN, Pin.OUT)
    motor_inb = Pin(config.MOTOR_INB_PIN, Pin.OUT)
    motor_ina.value(1)
    motor_inb.value(0)

    servo = PWM(config.SERVO_PIN, freq=config.SERVO_FREQUENCY_HZ,
                duty_u16=32768)
    motor = PWM(config.MOTOR_PWM_PIN, freq=config.MOTOR_FREQUENCY_HZ,
                duty_u16=32768)
    servo.duty_ns(config.STEERING_CENTER_NS)
    motor.duty_ns(config.MOTOR_BRAKE_NS)

    ir_sensor = Pin(config.IR_PIN, Pin.IN, Pin.PULL_UP)
    ir_ground = Pin(config.IR_GROUND_PIN, Pin.OUT)
    ir_ground.value(0)
    ir_sensor.irq(handler=on_ir_edge, trigger=Pin.IRQ_FALLING)
    Timer(-1, mode=Timer.PERIODIC, period=config.RPM_WINDOW_MS,
          callback=on_rpm_window)

    leds = [LED("LED_RED"), LED("LED_GREEN"), LED("LED_BLUE")]
    steering_pid = PID(config.STEERING_KP, config.STEERING_KI,
                       config.STEERING_KD, config.STEERING_INTEGRAL_LIMIT)
    speed_pid = PID(config.SPEED_KP, config.SPEED_KI,
                    config.SPEED_KD, config.SPEED_INTEGRAL_LIMIT)
    clock = time.clock()
    missed_lane_frames = 0
    current_rpm = 0.0

    # Hold the vehicle stationary long enough for a safe initial check.
    time.sleep(2)

    while True:
        if rpm_window_ready:
            current_rpm = rpm_from_edges(edge_count)
            edge_count = 0
            rpm_window_ready = False

        clock.tick()
        image = sensor.snapshot().binary([config.LANE_THRESHOLD])
        lane_center, left_marker, right_marker = find_lane_center(image)

        if lane_center is None:
            missed_lane_frames += 1
            steering_pid.reset()
            servo_pulse = config.STEERING_CENTER_NS
            motor_pulse = (config.MOTOR_BRAKE_NS
                           if missed_lane_frames >= config.MAX_MISSED_LANE_FRAMES
                           else config.MOTOR_BASE_NS)
            set_status(leds)
            error = 0.0
            steering_command = 0.0
        else:
            missed_lane_frames = 0
            error = steering_error_degrees(lane_center)
            steering_command = steering_pid.update(error)
            servo_pulse = steering_pulse_ns(steering_command)

            motor_pulse = config.MOTOR_BASE_NS
            if config.ENABLE_SPEED_PID:
                speed_adjustment = speed_pid.update(config.TARGET_RPM - current_rpm)
                motor_pulse = clamp(config.MOTOR_BASE_NS + speed_adjustment,
                                    config.MOTOR_MIN_NS, config.MOTOR_MAX_NS)

            if steering_command > config.STEERING_DEADBAND_DEGREES:
                set_status(leds, "blue")
            elif steering_command < -config.STEERING_DEADBAND_DEGREES:
                set_status(leds, "red")
            else:
                set_status(leds, "green")

        servo.duty_ns(int(servo_pulse))
        motor.duty_ns(int(motor_pulse))

        if config.DEBUG:
            print("lane={}, left={}, right={}, err={:.2f}, steer={:.2f}, rpm={:.1f}, "
                  "servo={}, motor={}, fps={:.1f}".format(
                      lane_center, left_marker, right_marker, error,
                      steering_command, current_rpm, int(servo_pulse),
                      int(motor_pulse), clock.fps()))


main()

