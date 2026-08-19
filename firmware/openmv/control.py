"""Vision and actuator helpers that are independent of board initialization."""

from math import atan2, pi

import config


def find_lane_center(image):
    """Return the midpoint of the nearest detected left/right lane markers.

    The camera image is expected to be binary, with white lane pixels. The
    search deliberately uses a near-field ROI, matching the final archived
    blob-detection implementation.
    """
    roi = (0, config.FRAME_HEIGHT - config.ROI_HEIGHT,
           config.FRAME_WIDTH, config.ROI_HEIGHT)
    blobs = image.find_blobs(
        [(255, 255)],
        roi=roi,
        pixels_threshold=config.BLOB_PIXELS_THRESHOLD,
        area_threshold=config.BLOB_AREA_THRESHOLD,
        merge=True,
    )

    frame_center_x = config.FRAME_WIDTH // 2
    left_marker = None
    right_marker = None

    for blob in blobs:
        x, y = blob.cx(), blob.cy()
        if x < frame_center_x:
            if left_marker is None or x > left_marker[0]:
                left_marker = (x, y)
        elif right_marker is None or x < right_marker[0]:
            right_marker = (x, y)

        if config.DEBUG:
            image.draw_rectangle(blob.rect(), color=127)
            image.draw_cross(x, y, color=127)

    if left_marker is None or right_marker is None:
        return None, left_marker, right_marker

    center = ((left_marker[0] + right_marker[0]) // 2,
              (left_marker[1] + right_marker[1]) // 2)
    return center, left_marker, right_marker


def steering_error_degrees(lane_center):
    """Convert lane-center displacement to a bounded angular error."""
    x_distance = (config.FRAME_WIDTH / 2) - lane_center[0]
    y_distance = max(1, config.FRAME_HEIGHT - lane_center[1])
    error = atan2(x_distance, y_distance) * (180 / pi)
    return clamp(error, -config.MAX_STEERING_DEGREES,
                 config.MAX_STEERING_DEGREES)


def steering_pulse_ns(turning_degrees):
    """Map a signed steering command to a calibrated servo pulse width."""
    turning_degrees = clamp(turning_degrees, -config.MAX_STEERING_DEGREES,
                            config.MAX_STEERING_DEGREES)
    if abs(turning_degrees) < config.STEERING_DEADBAND_DEGREES:
        turning_degrees = 0

    if turning_degrees >= 0:
        span = config.STEERING_RIGHT_NS - config.STEERING_CENTER_NS
    else:
        span = config.STEERING_CENTER_NS - config.STEERING_LEFT_NS

    return int(config.STEERING_CENTER_NS +
               (turning_degrees / config.MAX_STEERING_DEGREES) * span)


def rpm_from_edges(edge_count):
    """Estimate wheel RPM from IR edges accumulated over one configured window."""
    gear_rpm = (edge_count / config.GEAR_MARKS) * (60_000 / config.RPM_WINDOW_MS)
    return gear_rpm / config.GEAR_TO_WHEEL_RATIO


def clamp(value, lower, upper):
    return max(lower, min(upper, value))

