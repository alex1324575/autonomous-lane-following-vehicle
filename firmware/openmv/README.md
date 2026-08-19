# OpenMV firmware

This directory contains the cleaned, deployable reconstruction of the final lane-center workflow from the capstone archive. It targets OpenMV MicroPython and is **not** intended to run under CPython.

## Files

- `main.py` — camera, GPIO/PWM initialization, control loop, and fail-safe behavior.
- `config.py` — every vehicle-specific calibration value in one place.
- `control.py` — lane detection, steering conversion, and RPM utilities.
- `pid.py` — small PID controller that uses MicroPython tick counters.

## Deploy

1. In OpenMV IDE, create or upload all four files to the board filesystem.
2. Run `main.py` from the board filesystem.
3. Set `DEBUG = True` during calibration and watch the serial terminal.
4. When the lane center is stable, disable `DEBUG` for less serial overhead.

## Required calibration

Do not use the default values as a hardware guarantee. Confirm each item on your vehicle:

1. **Camera threshold:** tune `LANE_THRESHOLD` under track lighting; the provided value assumes a light line on a darker track.
2. **Steering endpoints:** verify `STEERING_LEFT_NS`, `STEERING_CENTER_NS`, and `STEERING_RIGHT_NS` with wheels off the ground.
3. **Motor command:** begin at `MOTOR_BRAKE_NS`, then raise `MOTOR_BASE_NS` slowly. Verify the motor direction before placing the vehicle on a track.
4. **IR sensor:** confirm `IR_PIN`, `GEAR_MARKS`, and `RPM_WINDOW_MS` against the encoder disk/gear.
5. **Speed PID:** leave `ENABLE_SPEED_PID = False` until a new calibration log supports enabling it.

## Safety

The firmware centers steering when it cannot see both lane markers and brakes after `MAX_MISSED_LANE_FRAMES`. Test that behavior with the drive wheels lifted before every new wiring or parameter change.

