"""Geometry, pixel ordering and capture time of the optional RViz image plane."""
import importlib.util
import copy
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('rclpy')
from sensor_msgs.msg import CameraInfo, Image

spec = importlib.util.spec_from_file_location(
    'navigation_camera_plane', Path(__file__).resolve().parents[1]
    / 'scripts/navigation_camera_plane.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fixture():
    info = CameraInfo()
    info.width, info.height = 3, 3
    info.header.frame_id = 'camera_optical'
    info.k = [2., 0., 1., 0., 2., 1., 0., 0., 1.]
    image = Image()
    image.header = copy.deepcopy(info.header)
    image.header.stamp.sec = 12
    image.encoding = 'rgb8'
    image.width, image.height, image.step = 3, 3, 12
    # Include row padding so the projection cannot mistake it for RGB pixels.
    data = np.zeros((3, 12), np.uint8)
    data[0, :3] = [255, 0, 0]
    data[2, 6:9] = [0, 0, 255]
    data[:, 9:] = 123
    image.data = data.tobytes()
    return image, info


def test_optical_axes_colors_and_capture_time():
    image, info = fixture()
    marker = module.image_plane(image, info, distance=0.8, stride=1)
    points = np.array([[p.x, p.y, p.z] for p in marker.points]).reshape(3, 3, 6, 3)
    # Pixel cell edges extend half a pixel beyond the corner pixel centers.
    assert points[0, 0, 0, 0] == pytest.approx(-0.6)
    assert points[0, 0, 0, 1] == pytest.approx(-0.6)
    assert points[2, 2, 2, 0] == pytest.approx(0.6)
    assert points[2, 2, 2, 1] == pytest.approx(0.6)
    assert np.allclose(points[..., 2], 0.8)
    assert marker.colors[0].r == 1.0 and marker.colors[0].b == 0.0
    assert marker.colors[-1].b == 1.0 and marker.colors[-1].r == 0.0
    assert marker.header == image.header
    # Every triangle is planar with a normal along the optical axis.
    triangles = points.reshape(-1, 3, 3)
    normals = np.cross(triangles[:, 1] - triangles[:, 0],
                       triangles[:, 2] - triangles[:, 0])
    assert np.allclose(normals[:, :2], 0)
    assert (normals[:, 2] > 0).all()


def test_rejects_mismatched_frame_and_uncalibrated_camera():
    image, info = fixture()
    info.header.frame_id = 'other_camera'
    with pytest.raises(ValueError):
        module.image_plane(image, info)
    image, info = fixture()
    info.k[0] = 0
    with pytest.raises(ValueError):
        module.image_plane(image, info)


def test_partial_pixel_blocks_fill_the_whole_rectangle():
    image, info = fixture()
    marker = module.image_plane(image, info, stride=2)
    assert len(marker.points) == 4 * 6
    vertices = np.array([[p.x, p.y] for p in marker.points])
    assert np.allclose(vertices.min(axis=0), [-0.6, -0.6])
    assert np.allclose(vertices.max(axis=0), [0.6, 0.6])


def test_ros_optical_axes_match_mujoco_camera_view():
    import math
    import yaml
    from g1_backend.camera_angle import optical_quaternion, mount
    root = Path(__file__).resolve().parents[1]
    optical = optical_quaternion(mount()[1][1], mount()[1])

    def rotation(q):
        w, x, y, z = q
        return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                         [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                         [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

    import xml.etree.ElementTree as ET
    urdf = next((root / 'src/g1_description/urdf').glob('g1_29dof.urdf.xacro'))
    origin = ET.parse(urdf).getroot().find("joint[@name='d435_joint']/origin")
    pitch = float(origin.get('rpy').split()[1])
    parent = rotation([math.cos(pitch/2), 0, math.sin(pitch/2), 0])
    view = yaml.safe_load((root / 'configs/experiments/navigation.yaml').read_text())['camera']['views'][0]
    camera = rotation(view['quaternion'])
    assert np.allclose(parent @ rotation(optical), camera @ np.diag([1, -1, -1]))
