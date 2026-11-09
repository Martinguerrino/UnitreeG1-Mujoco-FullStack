"""D435 tilt calibration shared by the renderer and capture-time ROS TF."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

MIN_PITCH_DEG, MAX_PITCH_DEG = -45., 90.
OPTICAL_QUATERNION = (.5, -.5, .5, -.5)  # wxyz, d435_link <- optical


def mount():
    path = Path(__file__).resolve().parents[1] / 'src/g1_description/urdf/g1_29dof.urdf.xacro'
    joint = ET.parse(path).getroot().find("joint[@name='d435_joint']")
    origin = joint.find('origin')
    return ([float(v) for v in origin.get('xyz').split()],
            [float(v) for v in origin.get('rpy').split()])


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v])


def rpy_quaternion(rpy):
    r, p, y = [v / 2 for v in rpy]
    return multiply(multiply([math.cos(y), 0, 0, math.sin(y)],
                             [math.cos(p), 0, math.sin(p), 0]),
                    [math.cos(r), math.sin(r), 0, 0])


def validate_pitch(degrees):
    if not math.isfinite(degrees) or not MIN_PITCH_DEG <= degrees <= MAX_PITCH_DEG:
        raise ValueError(f'Camera pitch must be between {MIN_PITCH_DEG} and {MAX_PITCH_DEG} degrees')
    return float(degrees)


def optical_quaternion(pitch, original_rpy):
    """Nominal d435_link <- tilted optical frame; pitch in radians."""
    nominal = rpy_quaternion(original_rpy)
    changed = rpy_quaternion([original_rpy[0], pitch, original_rpy[2]])
    inverse = nominal * [1, -1, -1, -1]
    return multiply(multiply(inverse, changed), OPTICAL_QUATERNION)


def render_quaternion(pitch, original_rpy):
    changed = rpy_quaternion([original_rpy[0], pitch, original_rpy[2]])
    return multiply(multiply(changed, OPTICAL_QUATERNION), [0, 1, 0, 0])
