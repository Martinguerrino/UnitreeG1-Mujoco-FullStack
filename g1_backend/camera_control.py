"""Simulation-only camera pitch; positive angles look down."""
import math

import numpy as np


def camera_pitch(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('Camera pitch must be finite')
    return max(-30.0, min(45.0, value))


def tilted_quaternion(original, degrees):
    """Compose MuJoCo wxyz quaternion with local right-axis rotation."""
    angle = math.radians(camera_pitch(degrees)) / 2
    w, x, y, z = original
    c, s = math.cos(angle), -math.sin(angle)
    return np.array([w*c-x*s, w*s+x*c, y*c+z*s, z*c-y*s])
