"""Apply humanoid turn assistance upstream of Nav2's collision monitor."""
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import yaml
from g1_backend.navigation_control import TurnAssist

ROOT = Path(__file__).resolve().parents[1]


class CommandFilter(Node):
    def __init__(self):
        super().__init__('g1_navigation_command_filter')
        tuning = yaml.safe_load((ROOT / 'configs/navigation/nav2_tuning.yaml').read_text())
        self.assistance = TurnAssist(tuning['turn_assist'])
        self.command_value = np.zeros(3)
        self.command_received = -float('inf')
        self.scan_stamp = -float('inf')
        self.scan_values = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel_assisted', 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.command, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_timer(.02, self.tick)

    def command(self, message):
        values = np.array([message.linear.x, message.linear.y, message.angular.z])
        if np.isfinite(values).all():
            self.command_value = values
            self.command_received = time.monotonic()

    def scan(self, message):
        values = np.asarray(message.ranges)
        angles = message.angle_min + np.arange(len(values)) * message.angle_increment
        bins = np.round(((angles + np.pi) % (2 * np.pi)) * 360 / (2 * np.pi)).astype(int) % 360
        nearest = np.full(360, np.nan)
        valid = ((values >= message.range_min) & (values <= message.range_max)) | np.isposinf(values)
        # An infinite LaserScan return means no obstacle before range_max.
        for index, distance in zip(bins[valid], np.minimum(values[valid], message.range_max)):
            if np.isnan(nearest[index]) or distance < nearest[index]:
                nearest[index] = distance
        self.scan_values = ([None if np.isnan(value) else float(value) for value in nearest]
                            if valid.any() else None)
        self.scan_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = (time.monotonic() - self.command_received < .25
                 and 0 <= now - self.scan_stamp < .5 and self.scan_values is not None)
        requested = self.command_value if fresh else np.zeros(3)
        result = self.assistance._assist_turn(requested, {'time': now, 'scan': self.scan_values})
        output = Twist()
        output.linear.x, output.linear.y, output.angular.z = map(float, result)
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = CommandFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
