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
"""Send one of the deliberately small, reviewed arm trajectories."""

import argparse
import sys

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
]

POSES = {
    'home': [0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0, 0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0],
    'reach_forward': [
        0.35,
        0.05,
        0.0,
        1.15,
        0.0,
        -0.25,
        0.0,
        0.35,
        -0.05,
        0.0,
        1.15,
        0.0,
        -0.25,
        0.0,
    ],
}


class ArmPoseClient(Node):
    def __init__(self) -> None:
        super().__init__('g1_arm_pose_client')
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/whole_body_controller/follow_joint_trajectory',
        )

    def execute(self, pose_name: str, duration: float) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('El controlador no está disponible tras 10 s')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = POSES[pose_name]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        goal.trajectory.points = [point]

        goal_handle = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_handle)
        accepted = goal_handle.result()
        if accepted is None or not accepted.accepted:
            self.get_logger().error('El controlador rechazó la trayectoria')
            return False
        result = accepted.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        wrapped = result.result()
        if wrapped is None or wrapped.result.error_code != 0:
            self.get_logger().error('La trayectoria terminó con error')
            return False
        self.get_logger().info(f"Pose '{pose_name}' completada")
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pose', choices=sorted(POSES))
    parser.add_argument('--duration', type=float, default=3.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error('--duration debe ser mayor que cero')

    rclpy.init()
    node = ArmPoseClient()
    try:
        return 0 if node.execute(args.pose, args.duration) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
