

# === IMPORTS === #
import sensor, time
import const
from PID import PID
from helpers import set_led, calc_phi, get_location
from helpers import calc_servo_pw
from machine import LED, PWM, Pin

### === SETUP === ###
# === CAMERA SETUP === #
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)    # Faster processing than RGB
sensor.set_framesize(sensor.QQQVGA)       # Low resolution = faster FPS
sensor.skip_frames(time=2000)             # Allow time to auto-adjust exposure
sensor.set_auto_gain(False)               # Fix brightness
sensor.set_auto_whitebal(False)           # Fix white balance

# === PWM SETUP (for controlling servo and motor) === #
# INA (Input A) on Pin 10. (Currently set to constant '1', since
# car is only going forward for now.)
ina = Pin("P10", Pin.OUT)
ina.value(1)

# INB (Input B) on Pin 9. (Currently set to constant '0', since
# car is only going forward for now.)
inb = Pin("P9", Pin.OUT)
inb.value(0)

# Servo on Pin 2 at 100Hz, 50% duty cycle
servo_duty_cycle_factor = round((const.SERVO_DUTY_CYCLE_PERCENTAGE / 100)
                                 * const.DUTY_CYCLE_FACTOR)
servo = PWM("P2", freq=const.SERVO_FREQ, duty_u16=servo_duty_cycle_factor)

# DC Motor on Pin 8 at 1600Hz, 50% duty cycle
motor_duty_cycle_factor = round((const.MOTOR_DUTY_CYCLE_PERCENTAGE / 100)
                                 * const.DUTY_CYCLE_FACTOR)
motor = PWM("P8", freq=const.MOTOR_FREQ, duty_u16=motor_duty_cycle_factor)


# === LED SETUP FOR VISUAL FEEDBACK === #
led_red   = LED("LED_RED")    # Right turn
led_green = LED("LED_GREEN")  # Straight
led_blue  = LED("LED_BLUE")   # Left turn
led_list  = [led_red, led_green, led_blue]   # List of LED objects (ordered).

# === CLOCK SETUP TO MEASURE FPS === #
clock = time.clock()

# === PID CONTROLLER SETUP === #
# TODO: Tune these values!
p_i_d = PID(p=2, i=0, d=1.5, i_max=25, i_resetting=False)


### === MAIN PROGRAM === ###
count = 0
next_steering_pw = const.STEERING_CENTER
next_motor_pw = const.MOTOR_VERY_SLOW

# === Initialize servo and motor === #
servo.duty_ns(const.STEERING_CENTER)
motor.duty_ns(const.MOTOR_BRAKE)
time.sleep(5)

# === Main loop === #
while True:
    clock.tick()
    img = sensor.snapshot().binary([const.THRESHOLD])  # Convert to B/W based on threshold

    # Try to fit a straight line to the detected white pixels
    line = img.get_regression([(255, 255)], robust=True)

    location = None
    next_motor_pw = const.MOTOR_VERY_SLOW   # Default = very slow speed
    next_steering_pw = const.STEERING_CENTER

    theta = 0
    phi = 0
    error = 0
    turning_angle = 0

    if line and line.magnitude() > 0:
        img.draw_line(line.line(), color=127)  # Draw the detected line

        print(line.theta())

        # Determine steering direction and servo pw needed #
        location = get_location(line)
        phi = calc_phi(line)

        # If the line is in the left or right 3rd of the screen,
        # steer the car towards the center point of the line (using 'phi' angle).
        #
        # Otherwise, steer the car in the direction of the deflection
        # angle of the line (using 'theta' angle).
        error = phi

        # Set motor speed.
        # motor_pw = F1_protocol(theta, location)

        # Use error to calculate turning angle via PID.
        turning_angle = p_i_d.get_pid(error)


        # Set floor on turning angle, to avoid "snake-like" behavior when running.
        if (abs(phi) < 10): turning_angle = 0


        # Set steering based on calculated turning angle.
        next_steering_pw = calc_servo_pw(turning_angle)


        # Set LED based on which third of the screen the regression line is
        # detected in.
        if line.x1() < const.CENTER_REGION[0] and line.x2() < const.CENTER_REGION[0]:
            set_led("blue", led_list)  # Turn Blue LED on
        elif line.x1() > const.CENTER_REGION[1] and line.x2() > const.CENTER_REGION[1]:
            set_led("red", led_list)  # Turn Red LED on
        else:
            set_led("green", led_list)  # Turn Green LED on

        print(f"Deflection Angle: {theta:.2f}°, Position: {location}")

    else:
        print("Deflection Angle: None, No line detected")
        set_led(None, led_list)  # No LED on

    motor_pw = next_motor_pw
    steering_pw = next_steering_pw

    # === SEND PWM TO HARDWARE ===
    motor.duty_ns(motor_pw)
    servo.duty_ns(steering_pw)


    # === DEBUG INFO FOR SERIAL MONITOR ===
    print(f"""
        Deflection Angle: {theta}°, Phi Angle: {phi}
        P_Term: {p_i_d.calc_P()}, D_Term: {p_i_d.calc_D()}, I_Term: {p_i_d.calc_I()},
        Turning Angle: {turning_angle}, Servo: {steering_pw}, Motor: {motor_pw},
        FPS: {clock.fps():.2f}
    """)


