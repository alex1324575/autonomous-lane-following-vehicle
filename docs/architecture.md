# Control architecture

## Per-frame lane estimation

The OpenMV Cam RT1062 captures an 80 × 60 grayscale frame. Automatic gain and white balance are disabled after startup so that a fixed threshold can segment a light lane marker from a darker track. The controller searches a 30-pixel-high near-field region for connected white blobs:

1. The nearest candidate on the left of the frame center is the left marker.
2. The nearest candidate on the right is the right marker.
3. When both markers are available, their midpoint is the lane center.
4. A signed angular error is calculated from that point to the camera center.

The archived work shows earlier regression-line and `find_lines()` experiments. The final preserved `main.py` used the two-blob midpoint approach, which is why it is the basis of the cleaned firmware.

## Steering loop

The PID output is limited to ±45° and mapped linearly around the servo's calibrated neutral pulse. A small deadband avoids repeated micro-corrections near the centerline. Unlike the historical script, the cleaned controller resets the steering PID on lane loss and applies a staged response:

| Condition | Steering | Motor |
| --- | --- | --- |
| Both markers detected | PID command | Base duty or optional speed-PID duty |
| One missed frame | Center | Base duty |
| `MAX_MISSED_LANE_FRAMES` misses | Center | Brake |

## Speed measurement and optional speed loop

The IR sensor increments an edge counter in an interrupt. A periodic timer ends the measurement window; RPM is estimated from gear marks and the configured gear-to-wheel ratio. The final report describes a motor PID that clamps its PWM adjustment and adds it to the base motor command.

The latest recovered `main.py` retains the RPM path and motor PID object but leaves its PWM adjustment commented out. This repository preserves that distinction: `ENABLE_SPEED_PID` is `False` by default until the physical vehicle is recalibrated. The report is evidence of the final system design; the archived file is evidence of the particular recovered firmware snapshot.

## Interfaces beyond the control loop

The final report also documents an HC-06 Bluetooth header for wireless debugging and start/stop control, along with a current-sense feedback path from the H-bridge to the OpenMV. These interfaces are represented in the Altium design files but are not exercised by the recovered lane-following firmware.
