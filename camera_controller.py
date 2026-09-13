"""Right-stick orbit control for SOFA's ``InteractiveCamera``.

The geometry calculation is deliberately independent of SOFA so it can be
tested without launching the simulator.  The camera always orbits its current
look-at point, which means mouse camera adjustments and controller adjustments
can be mixed during the same run.
"""

import math

import numpy as np


RIGHT_STICK_DEADZONE = 0.16
CAMERA_YAW_SPEED = 1.35
CAMERA_PITCH_SPEED = 1.10
MIN_ELEVATION = math.radians(-80.0)
MAX_ELEVATION = math.radians(80.0)


def apply_stick_deadzone(value, deadzone=RIGHT_STICK_DEADZONE):
    """Remove a symmetric dead zone while preserving the full output range."""
    value = max(-1.0, min(1.0, float(value)))
    deadzone = float(deadzone)
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("deadzone must be in [0, 1)")
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(scaled, value)


def orbit_position(
    position,
    look_at,
    right_x,
    right_y,
    dt,
    yaw_speed=CAMERA_YAW_SPEED,
    pitch_speed=CAMERA_PITCH_SPEED,
):
    """Return a camera position orbited around ``look_at`` by one input step."""
    position = np.asarray(position, dtype=float)
    look_at = np.asarray(look_at, dtype=float)
    if position.shape != (3,) or look_at.shape != (3,):
        raise ValueError("camera position and look-at must each have 3 values")
    if not np.isfinite(position).all() or not np.isfinite(look_at).all():
        raise ValueError("camera position and look-at must be finite")

    dt = max(0.0, float(dt))
    horizontal_input = apply_stick_deadzone(right_x)
    vertical_input = apply_stick_deadzone(right_y)
    if dt == 0.0 or (horizontal_input == 0.0 and vertical_input == 0.0):
        return position.copy()

    offset = position - look_at
    radius = float(np.linalg.norm(offset))
    if radius <= 1.0e-9:
        raise ValueError("camera position cannot coincide with its look-at point")

    azimuth = math.atan2(offset[2], offset[0])
    elevation = math.asin(max(-1.0, min(1.0, offset[1] / radius)))
    azimuth += horizontal_input * float(yaw_speed) * dt
    # SDL reports stick-up as a negative value; pushing up raises the camera.
    elevation -= vertical_input * float(pitch_speed) * dt
    elevation = max(MIN_ELEVATION, min(MAX_ELEVATION, elevation))

    horizontal_radius = radius * math.cos(elevation)
    return look_at + np.asarray(
        [
            horizontal_radius * math.cos(azimuth),
            radius * math.sin(elevation),
            horizontal_radius * math.sin(azimuth),
        ]
    )


def orbit_camera(camera, right_x, right_y, dt):
    """Apply one right-stick orbit step to a SOFA ``InteractiveCamera``."""
    position_data = camera.getData("position")
    look_at_data = camera.getData("lookAt")
    current_position = np.asarray(position_data.value, dtype=float).reshape(3)
    look_at = np.asarray(look_at_data.value, dtype=float).reshape(3)
    new_position = orbit_position(
        current_position, look_at, right_x, right_y, dt
    )
    if np.allclose(new_position, current_position, atol=1.0e-12, rtol=0.0):
        return False

    orientation = camera.getOrientationFromLookAt(
        new_position.tolist(), look_at.tolist()
    )
    position_data.value = new_position.tolist()
    camera.getData("orientation").value = list(orientation)
    camera.getData("distance").value = float(
        np.linalg.norm(new_position - look_at)
    )
    return True
