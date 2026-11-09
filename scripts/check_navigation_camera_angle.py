"""Live camera angle check: URDF default, ROS parameter, RViz drag and reset."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from visualization_msgs.msg import InteractiveMarkerFeedback
from tf2_ros import Buffer, TransformListener, TransformException

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.camera_angle import mount, optical_quaternion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-rviz', action='store_true')
    parser.add_argument('--require-panel', action='store_true')
    parser.add_argument('--numeric-pitch', type=float, default=0.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('camera_angle_check')
    frames = {}
    captured_angle = [None]
    node.create_subscription(Float64, '/camera/pitch_captured_deg',
        lambda message: captured_angle.__setitem__(0, message.data), 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    for name, topic in [('rgb', '/camera/color/image_raw'), ('depth', '/camera/depth/image_rect_raw')]:
        node.create_subscription(Image, topic, lambda msg, key=name: frames.update({key: msg}), qos_profile_sensor_data)
    client = node.create_client(SetParameters, '/g1_camera_angle/set_parameters')
    feedback = node.create_publisher(InteractiveMarkerFeedback, '/camera_angle/feedback', 10)
    original_rpy = mount()[1]
    original = math.degrees(original_rpy[1])
    results = []

    def wait(predicate, timeout=60):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
            if predicate():
                return
        raise TimeoutError('Camera control/image/TF did not reach requested state')

    def at_angle(degrees, after):
        if len(frames) != 2:
            return False
        if captured_angle[0] is None or abs(captured_angle[0] - degrees) > 1e-6:
            return False
        expected = optical_quaternion(math.radians(degrees), original_rpy)
        for image in frames.values():
            stamp = image.header.stamp
            if stamp.sec + stamp.nanosec * 1e-9 <= after:
                return False
            try:
                tf = buffer.lookup_transform('d435_link', image.header.frame_id, Time.from_msg(stamp))
            except TransformException:
                return False
            q = tf.transform.rotation
            if abs(np.dot(expected, [q.w, q.x, q.y, q.z])) < .999999:
                return False
        return True

    def record(label, degrees):
        image = frames['rgb']
        results.append({'case': label, 'pitch_deg': degrees,
                        'capture_time': image.header.stamp.sec + image.header.stamp.nanosec * 1e-9,
                        'rgb_hash': hashlib.sha256(bytes(image.data)).hexdigest()})

    try:
        wait(lambda: client.service_is_ready() and at_angle(original, 0)
             and (not args.require_rviz or bool(node.get_subscriptions_info_by_topic('/camera_angle/update')))
             and (not args.require_panel or any(p.node_name == 'rviz' for p in
                  node.get_subscriptions_info_by_topic('/camera/pitch_captured_deg'))))
        record('original_urdf', original)
        future = client.call_async(SetParameters.Request(parameters=[Parameter('pitch_deg', value=float(args.numeric_pitch)).to_parameter_msg()]))
        wait(future.done)
        assert future.result().results[0].successful
        wait(lambda: at_angle(args.numeric_pitch, results[-1]['capture_time']))
        record('parameter', args.numeric_pitch)
        event = InteractiveMarkerFeedback()
        event.header.frame_id = 'd435_link'
        event.marker_name, event.control_name, event.client_id = 'd435_pitch', 'pitch_ring', 'angle_check'
        event.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        delta = math.radians(30. - original)
        event.pose.orientation.w, event.pose.orientation.y = math.cos(delta / 2), math.sin(delta / 2)
        feedback.publish(event)
        wait(lambda: at_angle(30., results[-1]['capture_time']))
        record('rviz_drag_feedback', 30.)
        event.event_type = InteractiveMarkerFeedback.MENU_SELECT
        event.menu_entry_id = 1
        feedback.publish(event)
        wait(lambda: at_angle(original, results[-1]['capture_time']))
        record('restore_urdf_menu', original)
        assert results[0]['rgb_hash'] != results[1]['rgb_hash']
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({'passed': True, 'samples': results,
            'marker_clients': [p.node_name for p in node.get_subscriptions_info_by_topic('/camera_angle/update')],
            'captured_angle_clients': [p.node_name for p in node.get_subscriptions_info_by_topic('/camera/pitch_captured_deg')]}, indent=2))
        print(json.dumps({'passed': True, 'samples': results}))
    finally:
        # Restore calibration even if a stage fails.
        if client.service_is_ready():
            future = client.call_async(SetParameters.Request(parameters=[Parameter('pitch_deg', value=original).to_parameter_msg()]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
