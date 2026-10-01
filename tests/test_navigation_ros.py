"""ROS bridge regression tests. Run with system Python after sourcing ROS."""

import importlib.util
import io
from pathlib import Path
import struct
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('rclpy')

spec = importlib.util.spec_from_file_location(
    'navigation_bridge', Path(__file__).resolve().parents[1] / 'scripts/navigation_bridge.py'
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def packets(sequence, name, count=2):
    stream = io.BytesIO()
    np.savez_compressed(stream, time=1.25, **{name: np.ones((2, 3))})
    data = stream.getvalue()
    cut = len(data) // count
    return [
        struct.pack('!IHH', sequence, i, count)
        + data[i * cut : (i + 1) * cut if i < count - 1 else len(data)]
        for i in range(count)
    ]


def bridge_for_packets(packets):
    bridge = module.Bridge.__new__(module.Bridge)
    bridge.sensor_socket = Mock()
    bridge.sensor_socket.recvfrom.side_effect = [(p, None) for p in packets] + [BlockingIOError()]
    bridge.sensor_chunks = {}
    bridge.publish_sensors = Mock()
    return bridge


def test_newer_modality_does_not_discard_partial_image():
    rgb = packets(1, 'd435i_rgb')
    cloud = packets(2, 'mid360_points', 1)
    bridge = bridge_for_packets([rgb[0], *cloud, rgb[1]])
    names = []
    bridge.publish_sensors.side_effect = lambda values: names.append(set(values.files) - {'time'})
    bridge.poll_sensors()
    assert names == [{'mid360_points'}, {'d435i_rgb'}]
    assert not bridge.sensor_chunks


def test_missing_chunks_and_malformed_headers_are_bounded():
    data = [struct.pack('!IHH', i, 0, 2) + b'x' for i in range(100)]
    bridge = bridge_for_packets(data)
    bridge.poll_sensors()
    assert len(bridge.sensor_chunks) <= 32
    bridge.publish_sensors.assert_not_called()


def test_old_partial_frames_expire():
    bridge = bridge_for_packets([packets(2, 'mid360_points')[0]])
    bridge.sensor_chunks[1] = (module.time.monotonic() - 1, [b'x', None])
    bridge.poll_sensors()
    assert 1 not in bridge.sensor_chunks


def test_rgb_depth_share_optical_frame_stamp_and_calibration():
    bridge = module.Bridge.__new__(module.Bridge)
    bridge.color, bridge.depth, bridge.camera_info = Mock(), Mock(), Mock()
    bridge.publish_sensors(
        {
            'time': np.array(1.25),
            'd435i_rgb': np.zeros((240, 424, 3), np.uint8),
            'd435i_depth': np.ones((240, 424), np.float32),
        }
    )
    rgb = bridge.color.publish.call_args.args[0]
    depth = bridge.depth.publish.call_args.args[0]
    info = bridge.camera_info.publish.call_args.args[0]
    assert rgb.header == depth.header == info.header
    assert (rgb.width, rgb.height) == (depth.width, depth.height) == (info.width, info.height)
    assert info.k[0] == info.k[4] == 240 / (2 * np.tan(np.radians(58) / 2))
    assert depth.encoding == '32FC1'


@pytest.mark.parametrize('position', [[1.0, -2.0, 0.85], [1, -2, 1]])
def test_bridge_preserves_pelvis_height_roll_pitch_and_joint_stamp(position):
    import json
    import math

    q = np.array([0.9, 0.1, -0.15, 0.3])
    q /= np.linalg.norm(q)
    row = {'time': 1.25, 'position': position, 'quaternion': q.tolist(),
           'linear_velocity': [0, 0, 0], 'joint_names': ['waist_yaw_joint'],
           'joint_positions': [0.3], 'scan': None, 'scan_time': None}
    bridge = module.Bridge.__new__(module.Bridge)
    bridge.socket = Mock()
    bridge.socket.recvfrom.side_effect = [(json.dumps(row).encode(), None), BlockingIOError()]
    bridge.clock, bridge.odom, bridge.joint_states, bridge.tf = Mock(), Mock(), Mock(), Mock()
    bridge.previous_pose = None
    bridge.poll_sensors = Mock()
    bridge.poll()
    base, pelvis = bridge.tf.sendTransform.call_args.args[0]
    assert pelvis.transform.translation.z == position[2]
    odom = bridge.odom.publish.call_args.args[0]
    assert type(odom.pose.pose.position.x) is float
    assert type(odom.pose.pose.position.y) is float
    assert type(pelvis.transform.translation.z) is float
    from rclpy.serialization import serialize_message
    assert serialize_message(odom)
    assert base.transform.translation.x == 1
    assert base.transform.translation.y == -2
    joints = bridge.joint_states.publish.call_args.args[0]
    assert base.header.stamp == pelvis.header.stamp == joints.header.stamp
    assert list(joints.position) == [0.3]
    # Compose yaw and residual pelvis rotation to recover the physical pose.
    rotation = pelvis.transform.rotation
    residual = np.array([rotation.w, rotation.x, rotation.y, rotation.z])
    yaw = 2 * math.atan2(base.transform.rotation.z, base.transform.rotation.w)
    c, s = math.cos(yaw/2), math.sin(yaw/2)
    w, x, y, z = residual
    np.testing.assert_allclose([c*w-s*z, c*x-s*y, c*y+s*x, c*z+s*w], q, atol=1e-12)
