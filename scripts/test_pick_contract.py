"""Integration check: validation and missing-backend failure without arm motion.

Run with system Python after sourcing ROS and install/setup.bash, on an unused
ROS_DOMAIN_ID. Starts and terminates its own pick_server process.
"""

import signal
import subprocess

from action_msgs.msg import GoalStatus
from g1_interfaces.action import Pick
import rclpy
from rclpy.action import ActionClient


def main():
    rclpy.init()
    node = rclpy.create_node('test_pick_contract')
    client = ActionClient(node, Pick, '/g1/pick')
    process = subprocess.Popen(['ros2', 'run', 'g1_manipulation', 'pick_server'])

    def wait(future):
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        assert future.done(), 'Action timed out'
        return future.result()

    try:
        assert client.wait_for_server(timeout_sec=10)
        invalid = wait(client.send_goal_async(Pick.Goal()))
        assert not invalid.accepted
        goal = Pick.Goal()
        goal.object_id = 'cube'
        goal.arm = 'left_arm'
        goal.grasp_pose.header.frame_id = 'pelvis'
        goal.grasp_pose.pose.orientation.w = 1.0
        goal.approach_distance = 0.08
        goal.lift_distance = 0.1
        handle = wait(client.send_goal_async(goal))
        assert handle.accepted
        concurrent = wait(client.send_goal_async(goal))
        assert not concurrent.accepted
        result = wait(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED
        assert not result.result.success
        assert not result.result.object_attached
        assert 'No physical grasp backend' in result.result.message
        print('PASS: invalid/concurrent goals rejected; missing backend aborts before motion')
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
