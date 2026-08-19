# Design decisions and source reconciliation

This document connects the recovered source files to the team-authored final report and poster. It helps distinguish verified project history from configuration that must be revalidated on a future build.

## Vision strategy

The archive preserves multiple approaches: regression-line detection, Hough-style line detection, and the final left/right blob midpoint approach. The documented final flow uses grayscale QQQVGA frames, a binary threshold, a near-field bottom ROI, and `find_blobs()` to identify both lane markers. The maintainable firmware in `firmware/openmv/` follows that final approach.

## Dual PID design

The report describes independent steering and motor-speed PID controllers. Steering maps a bounded lane error to servo PWM. Motor control estimates RPM from a HiLetgo IR sensor and slotted gear, then clamps the PWM adjustment before applying it to the motor command.

The code snapshot preserved in `archive/firmware-history/main.py` measures RPM and creates a motor PID object, but its final motor-PWM adjustment remains disabled. This may reflect a test configuration rather than the final competition configuration. To avoid claiming a fresh validation, the cleaned firmware keeps speed PID opt-in until tested on the physical system.

## PCB architecture

The report explains why the PCB carries redundant brushed-motor control paths: a VNH5019ATR-E H-bridge supports bidirectional drive and current sense, while the IRF3205/MAX4427 path provides low-side PWM control. It also records separate logic, servo, and motor ground pours meeting at a battery-side star point. The native Altium project remains the authoritative engineering source.

## Camera-mount iteration

The first camera stand was rigid but fixed-angle, making field-of-view tuning expensive. The team then built an adjustable stand to change pitch and height during track testing. Initial video jitter was traced to loose chassis fasteners rather than the printed structure; replacing them with M3 machine screws eliminated the issue. The final mount was printed in PETG and used reinforced joints and truss cutouts to balance rigidity and weight.

## Evidence hierarchy

1. The physical vehicle, current calibration, and a new test log are the authority for a renewed run.
2. The native Altium project is the authority for the PCB design.
3. The final report and poster are the authority for historical results and design narrative.
4. `archive/firmware-history/` is an unmodified record of recovered implementation iterations.
5. `firmware/openmv/` is a cleaned reconstruction for review and future calibration.

