"""Contracts that keep MoveIt, the robot model and ros2_control aligned."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
URDF = ROOT / 'src/g1_description/urdf/g1_29dof.urdf.xacro'
SRDF = ROOT / 'src/g1_moveit_config/config/g1.srdf'
MOVEIT = ROOT / 'src/g1_moveit_config/config'
CONTROLLERS = ROOT / 'src/g1_control/config/controllers.yaml'

LEFT_ARM = {
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
}
RIGHT_ARM = {joint.replace('left_', 'right_') for joint in LEFT_ARM}
ARMS = LEFT_ARM | RIGHT_ARM


def test_planning_groups_are_seven_dof_arm_chains():
    root = ET.parse(SRDF).getroot()
    groups = {group.attrib['name']: group for group in root.findall('group')}
    left = groups['left_arm'].find('chain')
    right = groups['right_arm'].find('chain')
    assert (left.attrib['base_link'], left.attrib['tip_link']) == (
        'torso_link',
        'left_rubber_hand',
    )
    assert (right.attrib['base_link'], right.attrib['tip_link']) == (
        'torso_link',
        'right_rubber_hand',
    )
    assert {child.attrib['name'] for child in groups['dual_arms'].findall('group')} == {
        'left_arm',
        'right_arm',
    }


def test_moveit_only_commands_arm_joints_known_by_ros2_control():
    moveit = yaml.safe_load((MOVEIT / 'moveit_controllers.yaml').read_text())
    moveit_joints = set(
        moveit['moveit_simple_controller_manager']['whole_body_controller']['joints']
    )
    control = yaml.safe_load(CONTROLLERS.read_text())
    controlled = set(control['whole_body_controller']['ros__parameters']['joints'])
    assert moveit_joints == ARMS
    assert moveit_joints < controlled
    assert control['whole_body_controller']['ros__parameters']['allow_partial_joints_goal']


def test_every_arm_has_ik_planner_and_conservative_limits():
    kinematics = yaml.safe_load((MOVEIT / 'kinematics.yaml').read_text())
    planners = yaml.safe_load((MOVEIT / 'ompl_planning.yaml').read_text())
    limits = yaml.safe_load((MOVEIT / 'joint_limits.yaml').read_text())['joint_limits']
    assert set(kinematics) == {'left_arm', 'right_arm'}
    assert {'left_arm', 'right_arm', 'dual_arms'} <= set(planners)
    assert set(limits) == ARMS
    for joint_limit in limits.values():
        assert 0.0 < joint_limit['max_velocity'] <= 2.5
        assert 0.0 < joint_limit['max_acceleration'] <= 5.0


def test_named_home_states_cover_each_arm_exactly():
    root = ET.parse(SRDF).getroot()
    states = {
        state.attrib['group']: {joint.attrib['name'] for joint in state.findall('joint')}
        for state in root.findall("group_state[@name='home']")
    }
    assert states == {'left_arm': LEFT_ARM, 'right_arm': RIGHT_ARM}


def test_adjacent_links_are_disabled_in_self_collision_matrix():
    urdf = ET.parse(URDF).getroot()
    adjacent = {
        frozenset((joint.find('parent').attrib['link'], joint.find('child').attrib['link']))
        for joint in urdf.findall('joint')
    }
    srdf = ET.parse(SRDF).getroot()
    disabled = {
        frozenset((pair.attrib['link1'], pair.attrib['link2']))
        for pair in srdf.findall('disable_collisions')
        if pair.attrib['reason'] == 'Adjacent'
    }
    assert adjacent <= disabled
