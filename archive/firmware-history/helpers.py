import const
from math import atan2, pi

## === HELPER FUNCTIONS === ##
"""
Sets which LED on the microcontroller is currently ON, based on input arguments.
Only one LED is on at a time.

Parameters
----------
color:
    String.  Represents which color LED should be turned ON.
    Valid options are "red", "green", "blue".  None is also valid.
led_list:
    List of LED objects (should be 3 of them, ordered for "red",
    "green", and "blue").

Raises
------
NotImplementedError:
    If color parameter is invalid.

"""
def set_led(color, led_list):
    valid_options = ["red", "green", "blue", None]

    led_list[0].off()
    led_list[1].off()
    led_list[2].off()

    if color not in valid_options:
        raise NotImplementedError(f"{color} is an invalid argument for set_led().")

    if color == "red":
        led_list[0].on()
    elif color == "green":
        led_list[1].on()
    elif color == "blue":
        led_list[2].on()

"""
Determines which 3rd of the screen the detected line is in.

Parameters
----------
line:
    An image.line object that represents the track line that was detected by the
    OpenMV camera.

Returns
-------
location:
    String.  Represents which 3rd of the screen the detected line is in. Valid
    options are "left", "center", and "right."
"""
def get_location(line):
    if line.x1() < const.CENTER_REGION[0] and line.x2() < const.CENTER_REGION[0]:
        location = "Left"
    elif line.x1() > const.CENTER_REGION[1] and line.x2() > const.CENTER_REGION[1]:
        location = "Right"
    else:
        location = "Center"

    return location


"""
Calculates the angle between:
a) the line that is perpendicularly bisects the x-axis, and
b) the line from the center of the x-axis on the bottom of the screen,
   to the midpoint of the image.line object input argument.

This angle has a negative value when clockwise from the perpendicular bisector,
and a positive value when counterclockwise from the perpendicular bisector.

This angle effectively represents how much the car needs to turn in order to reach
the center of the detected line.

Parameters
----------
line:
    An image.line object that represents the track line that was detected by the
    OpenMV camera.

Returns
-------
phi:
    Float. The angle representing how much the car needs to turn to reach the center of
    the detected line.  This angle will be, at minimum, -45 degrees, and at
    maximum, 45 degrees.
"""
def calc_phi(line):
    midpoint = ((line.x2() + line.x1()) / 2, (line.y2() + line.y1()) / 2)
    x_dist = const.CENTER_POSITION - midpoint[0]
    y_dist = const.FRAME_HEIGHT - midpoint[1]
    phi = (atan2(x_dist, y_dist)) * (180 / pi)

    if   phi > 45  : phi = 45
    elif phi < -45 : phi = -45

    return phi


"""
Calculates the angle between:
a) the line that is perpendicularly bisects the x-axis, and
b) the line from the center of the x-axis on the bottom of the screen,
   to the calculated center of the lane.

This angle has a negative value when clockwise from the perpendicular bisector,
and a positive value when counterclockwise from the perpendicular bisector.

This angle effectively represents how much the car needs to turn in order to reach
the center of the detected line.

Parameters
----------
lane_center:
    An integer pair that represents the x and y coordinate of the calculated
    center of the lane detected by the OpenMV camera.

Returns
-------
error:
    Float. The angle representing how much the car needs to turn to reach the center of
    the detected line.  This angle will be, at minimum, -45 degrees, and at
    maximum, 45 degrees.
"""
def calc_error_lane(lane_center):
    x_dist = const.CENTER_POSITION - lane_center[0]
    y_dist = lane_center[1]
    error = (atan2(x_dist, y_dist)) * (180 / pi)

    if   error > 45  : error = 45
    elif error < -45 : error = -45

    return error


"""
Calculates the pulse width that the servo PWM needs to have in order to turn
a number of degrees based on the input argument.  (The servo PWM is assumed to
have a frequency of 100 Hz).

This pulse width is calculated by assuming that the range of motion of the servo
(in degrees) and the pulse width of its PWM are uniformly distributed and proportional
to one another.

Parameters
----------
turning_angle:
    Float. Angle representing how much the car wants to turn.  The angle is, at minimum
    -45 degrees, and at maximum, 45 degrees.

Returns
-------
width:
    Int. The pulse width that corresponds to the angle turn input.
"""
def calc_servo_pw(turning_angle):
    ratio = turning_angle / 90
    factor = (const.STEERING_RIGHT - const.STEERING_LEFT) * ratio

    servo_pw = const.STEERING_CENTER - factor
    return round(servo_pw)
