"""ROS bridge regression tests. Run with system Python after sourcing ROS."""

import importlib.util
import io
from pathlib import Path
import struct
from unittest.mock import Mock
from collections import deque

import numpy as np
import pytest

pytest.importorskip('rclpy')

spec = importlib.util.spec_from_file_location(
    'navigation_bridge', Path(__file__).resolve().parents[1] / 'scripts/navigation_bridge.py'
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_slam_scan_waits_for_delayed_odometry_transform():
    adapter_spec = importlib.util.spec_from_file_location(
        'navigation_slam_adapter',
        Path(__file__).resolve().parents[1] / 'scripts/navigation_slam_adapter.py',
    )
    adapter_module = importlib.util.module_from_spec(adapter_spec)
    adapter_spec.loader.exec_module(adapter_module)
    adapter = adapter_module.SlamAdapter.__new__(adapter_module.SlamAdapter)
    adapter.get_clock = Mock()
    adapter.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    scan = Mock()
    scan.header.stamp = module.stamp(9.9)
    adapter.pending_scans = deque([scan], maxlen=10)
    adapter.scan_cloud = Mock(side_effect=[False, True])
    adapter.process_scans()
    assert list(adapter.pending_scans) == [scan]
    adapter.process_scans()
    assert not adapter.pending_scans
    assert adapter.scan_cloud.call_count == 2
    expired = Mock()
    expired.header.stamp = module.stamp(9.0)
    adapter.pending_scans.append(expired)
    adapter.process_scans()
    assert not adapter.pending_scans
    assert adapter.scan_cloud.call_count == 2


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
    row = {
        'time': 1.25,
        'position': position,
        'quaternion': q.tolist(),
        'linear_velocity': [0, 0, 0],
        'joint_names': ['waist_yaw_joint'],
        'joint_positions': [0.3],
        'scan': None,
        'scan_time': None,
    }
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
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    w, x, y, z = residual
    np.testing.assert_allclose(
        [c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w], q, atol=1e-12
    )


def test_real_mode_does_not_publish_truth_to_navigation_tf_or_scan():
    import json

    row = {
        'time': 1.25,
        'position': [123, -456, 9],
        'quaternion': [1, 0, 0, 0],
        'linear_velocity': [10, 20, 0],
        'joint_names': [],
        'joint_positions': [],
        'imu': {'quat': [1, 0, 0, 0], 'gyro': [0, 0, 0], 'acc': [0, 0, 9.81]},
        'scan': [1.0] * 360,
        'scan_time': 1.25,
    }
    bridge = module.Bridge.__new__(module.Bridge)
    bridge.real_odom = True
    bridge.socket = Mock()
    bridge.socket.recvfrom.side_effect = [(json.dumps(row).encode(), None), BlockingIOError()]
    for name in ('clock', 'odom', 'truth', 'imu', 'joint_states', 'tf', 'scan'):
        setattr(bridge, name, Mock())
    bridge.previous_pose = None
    bridge.poll_sensors = Mock()
    bridge.poll()
    bridge.odom.publish.assert_not_called()
    bridge.scan.publish.assert_not_called()
    bridge.tf.sendTransform.assert_not_called()
    assert bridge.truth.publish.call_args.args[0].pose.pose.position.x == 123
    assert bridge.imu.publish.call_args.args[0].header.frame_id == 'pelvis'
    # Neither the IMU nor ground truth supplies TF in real mode.
    row.pop('imu')
    bridge.socket.recvfrom.side_effect = [(json.dumps(row).encode(), None), BlockingIOError()]
    bridge.poll()
    bridge.tf.sendTransform.assert_not_called()


def test_real_mode_inhibits_commands_without_fresh_sensor_odometry():
    import json
    from geometry_msgs.msg import Twist

    bridge = module.Bridge.__new__(module.Bridge)
    bridge.real_odom = True
    bridge.last_received = module.time.monotonic()
    bridge.last_sim_time = 2.0
    bridge.last_estimated_time = 1.0
    bridge.socket = Mock()
    command = Twist()
    command.linear.x = 0.2
    bridge.command(command)
    assert json.loads(bridge.socket.sendto.call_args.args[0])['velocity'] == [0, 0, 0]
    bridge.last_estimated_time = 1.9
    bridge.command(command)
    assert json.loads(bridge.socket.sendto.call_args.args[0])['velocity'][0] == 0.2


def test_torso_imu_keeps_individual_stamps_and_does_not_use_truth_orientation():
    import json
    from rclpy.serialization import serialize_message

    row = {
        'time': 1.25, 'position': [123, 456, 9], 'quaternion': [0.8, 0, 0, 0.6],
        'linear_velocity': [0, 0, 0], 'joint_names': [], 'joint_positions': [],
        'scan': None, 'scan_time': None,
        'torso_imu': [
            {'time': 1.235, 'gyro': [1, 2, 3], 'acc': [0, 0, 9.81]},
            {'time': 1.24, 'gyro': [4, 5, 6], 'acc': [1, 0, 9.81]},
        ],
    }
    bridge = module.Bridge.__new__(module.Bridge)
    bridge.real_odom = True
    bridge.socket = Mock()
    bridge.socket.recvfrom.side_effect = [(json.dumps(row).encode(), None), BlockingIOError()]
    for name in ('clock', 'truth', 'torso_imu', 'joint_states', 'tf'):
        setattr(bridge, name, Mock())
    bridge.previous_pose = None
    bridge.poll_sensors = Mock()
    bridge.poll()
    first, second = [call.args[0] for call in bridge.torso_imu.publish.call_args_list]
    assert first.header.stamp.nanosec == 235000000
    assert second.header.stamp.nanosec == 240000000
    assert first.header.frame_id == 'torso_imu'
    assert first.orientation_covariance[0] == -1
    assert first.orientation.w == 1  # ROS default, marked unavailable above.
    assert first.orientation.z == 0
    assert first.angular_velocity.x == 1
    assert serialize_message(first)
    bridge.tf.sendTransform.assert_not_called()


def test_lidar_snapshot_has_xyzi_without_invented_scan_times():
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    bridge = module.Bridge.__new__(module.Bridge)
    bridge.points = Mock()
    points = np.array([[1., 2., 3.], [4., 5., 6.]], dtype=np.float32)
    bridge.publish_sensors({'time': np.array(1.25), 'mid360_points': points})
    message = bridge.points.publish.call_args.args[0]
    assert [field.name for field in message.fields] == ['x', 'y', 'z', 'intensity']
    np.testing.assert_array_equal(read_points_numpy(message, ['x', 'y', 'z']), points)


def test_tracker_compensates_articulated_mount_without_imu():
    odom_spec = importlib.util.spec_from_file_location(
        'navigation_odometry',
        Path(__file__).resolve().parents[1] / 'scripts/navigation_odometry.py',
    )
    odom_module = importlib.util.module_from_spec(odom_spec)
    odom_spec.loader.exec_module(odom_module)
    from g1_backend.sensor_odometry import Registration, pose_matrix

    tracker = odom_module.Tracker()
    anchor = pose_matrix([0.2, -0.1, 0.8, 0, 0.02, 0.4])
    old_mount = pose_matrix([0.05, 0, 0.4, 0, -0.1, 0])
    new_mount = pose_matrix([0.05, 0, 0.4, 0, -0.1, 0.1])
    movement = pose_matrix([0.1, 0.02, 0.03, 0.015, -0.02, 0.06])
    tracker.initialize(object(), old_mount, anchor, 1.0)
    camera_motion = np.linalg.inv(old_mount) @ movement @ new_mount
    result = Registration(camera_motion, np.eye(6) * 0.001, 0.01, 0.9)
    pose, covariance, _ = tracker.measurement(result, new_mount, 1.2)
    np.testing.assert_allclose(pose, anchor @ movement, atol=1e-10)
    assert np.linalg.eigvalsh(covariance)[0] > 0
