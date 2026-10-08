"""ROS image decoding and end-to-end RGB-D/TF publication."""

import importlib.util
import json
from pathlib import Path
import time

import numpy as np
import pytest

pytest.importorskip('rclpy')
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

spec = importlib.util.spec_from_file_location(
    'perception_rgbd', Path(__file__).resolve().parents[1] / 'scripts/perception_rgbd.py'
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('encoding,dtype,factor', [('32FC1', '>f4', 1), ('16UC1', '>u2', 1000)])
def test_depth_byte_order_padding_and_units(encoding, dtype, factor):
    values = np.array([[1, 2], [3, 4]])
    encoded = (values * factor).astype(dtype).view('u1').reshape(2, -1)
    padded = np.column_stack([encoded, np.zeros((2, 4), dtype='u1')])
    image = Image(
        width=2,
        height=2,
        encoding=encoding,
        is_bigendian=1,
        step=padded.shape[1],
        data=padded.tobytes(),
    )
    np.testing.assert_allclose(module.decode(image), values)
    image.step = 1
    with pytest.raises(ValueError):
        module.decode(image)


def test_ros_registered_images_tf_output_and_expiration():
    rclpy.init()
    detector = module.Perception()
    source = Node('perception_test_source')
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(detector)
    results = []
    marker_results = []
    source.create_subscription(
        String,
        '/perception/detections',
        lambda message: results.append(json.loads(message.data)),
        10,
    )
    source.create_subscription(MarkerArray, '/perception/markers', marker_results.append, 10)
    rgb_pub = source.create_publisher(Image, '/camera/color/image_raw', 10)
    depth_pub = source.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
    info_pub = source.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
    broadcaster = StaticTransformBroadcaster(source)
    camera = TransformStamped()
    camera.header.frame_id, camera.child_frame_id = 'base_link', 'test_camera_optical'
    camera.transform.translation.z = 1.6
    camera.transform.rotation.x = 1.0  # Optical Z down, optical Y opposite base Y.
    camera.transform.rotation.w = 0.0
    pelvis = TransformStamped()
    pelvis.header.frame_id, pelvis.child_frame_id = 'base_link', 'pelvis'
    pelvis.transform.translation.z = 0.8
    pelvis.transform.rotation.w = 1.0
    odom = TransformStamped()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
    broadcaster.sendTransform([odom, camera, pelvis])
    height, width, focal = 240, 320, 180.0
    v, u = np.mgrid[:height, :width]
    x, y = (u - (width - 1) / 2) * 0.75 / focal, -(v - (height - 1) / 2) * 0.75 / focal
    box_pixels = (abs(x - 0.1) <= 0.08) & (abs(y - 0.05) <= 0.04)
    depth = np.full((height, width), 0.85, dtype='<f4')
    depth[box_pixels] = 0.75
    rgb = np.full((height, width, 3), 100, dtype='u1')
    rgb[box_pixels] = [255, 0, 0]
    info = CameraInfo(width=width, height=height)
    info.header.frame_id = 'test_camera_optical'
    info.k = [focal, 0.0, (width - 1) / 2, 0.0, focal, (height - 1) / 2, 0.0, 0.0, 1.0]
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            stamp = source.get_clock().now().to_msg()
            info.header.stamp = stamp
            color = Image(
                header=info.header,
                width=width,
                height=height,
                encoding='rgb8',
                step=width * 3,
                data=rgb.tobytes(),
            )
            distance = Image(
                header=info.header,
                width=width,
                height=height,
                encoding='32FC1',
                step=width * 4,
                data=depth.tobytes(),
            )
            info_pub.publish(info)
            rgb_pub.publish(color)
            depth_pub.publish(distance)
            for _ in range(10):
                executor.spin_once(timeout_sec=0.02)
            valid = [r for r in results if r['status'] == 'ok' and r['tables'][0]['objects']]
            if valid and valid[-1]['tables'][0]['objects'][0]['confirmed']:
                break
        assert valid, results[-3:]
        result = valid[-1]
        assert result['frame_id'] == 'pelvis'
        box = result['tables'][0]['objects'][0]
        np.testing.assert_allclose(box['center'], [0.1, 0.05, 0], atol=0.015)
        np.testing.assert_allclose(box['dimensions'], [0.16, 0.08, 0.1], atol=0.015)
        assert result['stamp']['sec'] > 0
        assert result['tracking_frame'] == 'odom'
        assert box['confirmed'] and box['hits'] >= 2
        assert len({r['tables'][0]['objects'][0]['id'] for r in valid}) == 1
        assert 'raw_pose' in box and box['age_sec'] >= 0
        visual = [
            m for message in marker_results for m in message.markers if m.action == Marker.ADD
        ]
        labels = [m for m in visual if m.type == Marker.TEXT_VIEW_FACING]
        assert any('Mesa table_' in m.text and 'edad' in m.text for m in labels)
        assert any('Caja box_' in m.text and '0.150 x 0.075 x 0.100' in m.text for m in labels)
        volumes = [m for m in visual if m.type == Marker.CUBE]
        assert any(m.color.g == pytest.approx(0.8) and m.color.r == 0 for m in volumes)
        assert any(m.color.r == 1 and m.color.g == m.color.b == 0 for m in volumes)
        axes = [m for m in visual if m.type == Marker.ARROW]
        assert {tuple((p.x, p.y, p.z) for p in m.points)[1] for m in axes} == {
            (0.12, 0.0, 0.0),
            (0.0, 0.12, 0.0),
            (0.0, 0.0, 0.12),
        }
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert results[-1]['tables'] == []
        assert results[-1]['status'] == 'waiting_for_rgbd'
    finally:
        executor.shutdown()
        detector.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
