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

The IR sensor increments an edge counter in an interrupt. A periodic timer ends the measurement window; RPM is estimated from gear marks and the configured gear-to-wheel ratio. The original code contained both the counter and a motor PID object, but left the duty adjustment commented out in the latest final loop. `ENABLE_SPEED_PID` is therefore `False` by default in the reconstruction. It is an implementation path, not a claim that it was freshly validated.

