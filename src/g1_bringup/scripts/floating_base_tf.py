#!/usr/bin/python3
# Copyright 2026 G1 Locomanipulation contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Bridge MuJoCo floating-base odometry into the ROS TF tree."""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FloatingBaseTf(Node):
    def __init__(self) -> None:
        super().__init__('floating_base_tf')
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            '/simulator/floating_base_state',
            self._publish_transform,
            10,
        )

    def _publish_transform(self, message: Odometry) -> None:
        transform = TransformStamped()
        # mujoco_ros2_control stamps odometry with MuJoCo's internal clock even
        # when the ROS stack uses wall time. Re-stamp the TF so the complete
        # tree lives in the clock domain selected by this node.
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'pelvis'
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = FloatingBaseTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
