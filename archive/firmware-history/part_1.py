"""
Filename: part_1.py
Created for: EEC 195A Lab 6, part 1 ("Getting Started")
Team Number: 16
Team Members: Haaris Tahir-Kheli
              Maheshwari Tanwar
              Colin Tang
              Yanning Xu
Description: The script continuously takes a picture with the camera
             on the OpenMV Cam RT1062 Microcontroller.  It also flashs
             the red led on the microcontroller on whenever 50 frames
             of photos have been taken by the camera.  Finally, the
             current FPS of the microcontroller is printed to the serial
             terminal.
"""
import sensor, time  # Import necessary libraries
from machine import LED

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)  # Set frame size to QQQVGA (80x60)
sensor.skip_frames(time=2000)  # Wait 2 seconds to let camera adjust.

clock = time.clock()  # Create a clock object to measure FPS
led = LED("LED_RED")  # Initialize red LED

frame_count = 0  # Frame counter for blinking LED


while True:
    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.
    frame_count += 1  # Increment frame count

    # LED logic.  (Note that led.toggle() is avoided, since that function
    # forces the led to be ON for 1 sec before exiting function call.  This
    # causes an unnecessary delay in the LED signal.)
    led.on() if not (frame_count % 50) else led.off()

    print("FPS: ", clock.fps())  # Note: OpenMV Cam runs about half as fast when connected
    # to the IDE. The FPS should increase once disconnected.
