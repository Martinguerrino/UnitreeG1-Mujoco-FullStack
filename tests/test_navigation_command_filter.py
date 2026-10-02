"""Measured scan freshness and emergency-stop behavior of the ROS turn filter."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock
import time

import numpy as np
import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from g1_backend.navigation_control import TurnAssist

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('navigation_command_filter', ROOT / 'scripts/navigation_command_filter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def filter_node():
    node = module.CommandFilter.__new__(module.CommandFilter)
    node.assistance = TurnAssist({'enabled': True, 'speed': .3, 'acceleration': 10})
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 2_000_000_000
    node.publisher = Mock()
    node.command_received = time.monotonic()
    node.command_value = np.array([0., 0., .5])
    node.scan_stamp = 1.9
    node.scan_values = [20.] * 360
    return node


def test_missing_or_stale_sensor_data_stops_all_assistance():
    node = filter_node()
    node.assistance._assist_turn(np.array([0., 0., .5]), {'time': 1., 'scan': [20.] * 360})
    node.scan_stamp = 1.
    node.tick()
    message = node.publisher.publish.call_args.args[0]
    assert message.linear.x == message.linear.y == message.angular.z == 0
    assert node.assistance.turn_started is None
    node.scan_stamp = 1.9
    node.scan_values = None
    node.tick()
    assert node.publisher.publish.call_args.args[0].angular.z == 0


def test_stop_command_cancels_active_turn_instead_of_emitting_a_pulse():
    node = filter_node()
    node.assistance._assist_turn(np.array([0., 0., .5]), {'time': 1., 'scan': [20.] * 360})
    node.command(Twist())
    node.tick()
    message = node.publisher.publish.call_args.args[0]
    assert message.linear.x == message.angular.z == 0


def test_scan_rebinning_preserves_forward_obstacle_and_treats_no_return_as_free():
    node = filter_node()
    scan = LaserScan()
    scan.angle_min = 0.
    scan.angle_increment = 2 * np.pi / 360
    scan.range_min, scan.range_max = .05, 20.
    scan.ranges = [float('inf')] * 360
    scan.ranges[0] = .3
    node.scan(scan)
    assert node.scan_values[180] == pytest.approx(.3)
    assert node.scan_values[0] == 20
    assert not node.assistance._sector_clear(node.scan_values, True, .5)
    assert node.assistance._sector_clear(node.scan_values, False, .5)
    scan.ranges = [float('nan')] * 360
    node.scan(scan)
    assert node.scan_values is None
