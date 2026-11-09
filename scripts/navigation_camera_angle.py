"""RViz interactive D435 pitch control, initialized from the original URDF."""
import math
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
from std_msgs.msg import Float64
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.camera_angle import mount, validate_pitch, MIN_PITCH_DEG, MAX_PITCH_DEG


class CameraAngle(Node):
    def __init__(self):
        super().__init__('g1_camera_angle')
        self.original = math.degrees(mount()[1][1])
        self.declare_parameter('urdf_pitch_deg', self.original,
            ParameterDescriptor(description='Original D435 URDF mounting pitch', read_only=True))
        descriptor = ParameterDescriptor(description='D435 pitch in degrees; positive looks down. Original URDF at startup.',
            floating_point_range=[FloatingPointRange(from_value=MIN_PITCH_DEG, to_value=MAX_PITCH_DEG, step=0.)])
        self.pitch = self.declare_parameter('pitch_deg', self.original, descriptor).value
        validate_pitch(self.pitch)
        self.publisher = self.create_publisher(Float64, '/camera/pitch_deg', 10)
        self.server = InteractiveMarkerServer(self, 'camera_angle')
        marker = InteractiveMarker()
        marker.header.frame_id = 'd435_link'
        marker.name = 'd435_pitch'
        marker.description = 'D435 pitch: drag ring / reset URDF with right click'
        marker.scale = .35
        marker.pose.orientation.w = 1.
        control = InteractiveMarkerControl()
        control.name = 'pitch'
        control.orientation.w = control.orientation.z = math.sqrt(.5)
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.always_visible = True
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.
        sphere.pose.position.x = .18  # Keep the handle clear of the head mesh.
        sphere.scale.x = sphere.scale.y = sphere.scale.z = .10
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = .1, .8, 1., .8
        control.markers.append(sphere)
        marker.controls.append(control)
        ring = InteractiveMarkerControl()
        ring.name = 'pitch_ring'
        ring.orientation = control.orientation
        ring.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        ring.orientation_mode = InteractiveMarkerControl.FIXED
        marker.controls.append(ring)
        self.server.insert(marker, feedback_callback=self.feedback)
        self.menu = MenuHandler()
        self.menu.insert('Restaurar angulo original URDF', callback=self.reset)
        self.menu.apply(self.server, marker.name)
        self.add_on_set_parameters_callback(self.parameters)
        self.apply(self.pitch)
        self.create_timer(.2, self.publish)

    def publish(self):
        self.publisher.publish(Float64(data=float(self.pitch)))

    def apply(self, degrees):
        self.pitch = degrees
        delta = math.radians(degrees - self.original)
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.orientation.w = math.cos(delta / 2)
        pose.orientation.y = math.sin(delta / 2)
        self.server.setPose('d435_pitch', pose)
        self.server.applyChanges()
        self.publish()

    def parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'pitch_deg':
                try:
                    degrees = validate_pitch(float(parameter.value))
                except (ValueError, TypeError) as error:
                    return SetParametersResult(successful=False, reason=str(error))
                self.apply(degrees)
        return SetParametersResult(successful=True)

    def feedback(self, message):
        if message.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            q = message.pose.orientation
            delta = math.atan2(2 * (q.w*q.y + q.x*q.z), 1 - 2 * (q.y*q.y + q.z*q.z))
            degrees = max(MIN_PITCH_DEG, min(MAX_PITCH_DEG, self.original + math.degrees(delta)))
            self.set_parameters([Parameter('pitch_deg', value=float(degrees))])

    def reset(self, message):
        self.set_parameters([Parameter('pitch_deg', value=float(self.original))])


def main():
    rclpy.init()
    node = CameraAngle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
