"""Optional RGB image plane in the camera optical frame, for RViz only."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def image_plane(image, info, distance=0.8, stride=4):
    """Back-project RGB pixels onto a fixed plane, not measured scene depth."""
    if (image.encoding != 'rgb8' or image.width != info.width
            or image.height != info.height or image.header.frame_id != info.header.frame_id):
        raise ValueError('RGB image and calibration do not match')
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if not np.isfinite([fx, fy, cx, cy, distance]).all() or min(fx, fy, distance) <= 0:
        raise ValueError('Invalid camera calibration or plane distance')
    if image.step < image.width * 3 or len(image.data) != image.step * image.height:
        raise ValueError('Invalid RGB buffer')
    rows = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(image.height, image.step)
    rgb = rows[:, :image.width * 3].reshape(image.height, image.width, 3)[::stride, ::stride]
    # Real triangles stay in the optical XY plane; billboard points would
    # face the RViz observer instead of the camera's projection direction.
    v, u = np.mgrid[0:image.height:stride, 0:image.width:stride]
    x0 = (u - 0.5 - cx) * distance / fx
    x1 = (np.minimum(u + stride, image.width) - 0.5 - cx) * distance / fx
    y0 = (v - 0.5 - cy) * distance / fy
    y1 = (np.minimum(v + stride, image.height) - 0.5 - cy) * distance / fy
    xy = np.stack([np.stack([x0, y0], -1), np.stack([x1, y0], -1),
                   np.stack([x1, y1], -1), np.stack([x0, y0], -1),
                   np.stack([x1, y1], -1), np.stack([x0, y1], -1)], axis=-2)
    marker = Marker()
    marker.header = image.header
    marker.ns = 'camera_image_plane'
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.points = [Point(x=float(x), y=float(y), z=float(distance))
                     for x, y in xy.reshape(-1, 2)]
    colors = np.repeat(rgb.reshape(-1, 3), 6, axis=0) / 255.0
    marker.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
                     for r, g, b in colors]
    marker.lifetime.sec = 1
    return marker


class CameraPlane(Node):
    def __init__(self):
        super().__init__('g1_camera_image_plane')
        self.info = None
        self.pending = None
        self.publisher = self.create_publisher(
            Marker, '/camera/color/image_plane', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.calibration, 1)
        self.create_subscription(Image, '/camera/color/image_raw', self.image,
                                 qos_profile_sensor_data)

    def calibration(self, info):
        self.info = info
        if self.pending is not None:
            image, self.pending = self.pending, None
            self.image(image)

    def image(self, image):
        # Do no projection work while the optional RViz display is disabled.
        if self.publisher.get_subscription_count() == 0:
            return
        if self.info is None:
            self.pending = image
            return
        try:
            self.publisher.publish(image_plane(image, self.info, stride=8))
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = CameraPlane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
