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
"""Launch the G1 simulation stack, optionally with MoveIt 2 and RViz."""

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
import yaml


def _materialize_mjcf(fixed_base: bool) -> str:
    """Create a self-contained temporary scene with an absolute mesh path."""
    description_share = Path(get_package_share_directory('g1_description'))
    mujoco_share = Path(get_package_share_directory('g1_mujoco'))
    generated = Path(tempfile.mkdtemp(prefix='g1_mujoco_'))

    model = (mujoco_share / 'mjcf' / 'g1_29dof.xml').read_text()
    mesh_dir = description_share / 'meshes'
    model = model.replace('meshdir="meshes"', f'meshdir="{mesh_dir}"', 1)
    (generated / 'g1_29dof.xml').write_text(model)
    scene = (mujoco_share / 'scenes' / 'scene_29dof.xml').read_text()
    if not fixed_base:
        start = scene.index('  <equality>')
        end = scene.index('  </equality>', start) + len('  </equality>\n')
        scene = scene[:start] + scene[end:]
    (generated / 'scene_29dof.xml').write_text(scene)
    return str(generated / 'scene_29dof.xml')


def _launch_setup(context):
    headless = LaunchConfiguration('headless').perform(context)
    fixed_base = LaunchConfiguration('fixed_base').perform(context).lower() == 'true'
    moveit_enabled = LaunchConfiguration('moveit').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    model_path = _materialize_mjcf(fixed_base)
    description_share = get_package_share_directory('g1_description')
    control_share = get_package_share_directory('g1_control')
    xacro_file = os.path.join(description_share, 'urdf', 'g1_29dof.urdf.xacro')
    controller_file = os.path.join(control_share, 'config', 'controllers.yaml')
    if moveit_enabled:
        moveit_share = get_package_share_directory('g1_moveit_config')
        rviz_file = os.path.join(moveit_share, 'config', 'moveit.rviz')
    else:
        rviz_file = os.path.join(description_share, 'config', 'g1.rviz')

    robot_description_xml = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' mujoco_model_path:=',
            model_path,
            ' headless:=',
            headless,
        ]
    ).perform(context)
    robot_description = {
        'robot_description': ParameterValue(robot_description_xml, value_type=str)
    }

    moveit_parameters = []
    if moveit_enabled:
        config_dir = Path(moveit_share) / 'config'
        semantic = (config_dir / 'g1.srdf').read_text()
        kinematics = yaml.safe_load((config_dir / 'kinematics.yaml').read_text())
        joint_limits = yaml.safe_load((config_dir / 'joint_limits.yaml').read_text())
        ompl = yaml.safe_load((config_dir / 'ompl_planning.yaml').read_text())
        controllers = yaml.safe_load((config_dir / 'moveit_controllers.yaml').read_text())
        execution = yaml.safe_load((config_dir / 'trajectory_execution.yaml').read_text())
        ompl.update(
            {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': [
                    'default_planning_request_adapters/ResolveConstraintFrames',
                    'default_planning_request_adapters/ValidateWorkspaceBounds',
                    'default_planning_request_adapters/CheckStartStateBounds',
                    'default_planning_request_adapters/CheckStartStateCollision',
                ],
                'response_adapters': [
                    'default_planning_response_adapters/AddTimeOptimalParameterization',
                    'default_planning_response_adapters/ValidateSolution',
                    'default_planning_response_adapters/DisplayMotionPath',
                ],
                'start_state_max_bounds_error': 0.1,
            }
        )
        moveit_parameters = [
            robot_description,
            {'robot_description_semantic': semantic},
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            {
                'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl',
                'ompl': ompl,
            },
            controllers,
            execution,
            {
                'allow_trajectory_execution': True,
                'publish_robot_description': True,
                'publish_robot_description_semantic': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
                'use_sim_time': use_sim_time,
            },
        ]

    if fixed_base:
        base_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x',
                '0',
                '--y',
                '0',
                '--z',
                '0.793',
                '--roll',
                '0',
                '--pitch',
                '0',
                '--yaw',
                '0',
                '--frame-id',
                'odom',
                '--child-frame-id',
                'pelvis',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='both',
        )
    else:
        base_tf_node = Node(
            package='g1_bringup',
            executable='floating_base_tf.py',
            parameters=[{'use_sim_time': use_sim_time}],
            output='both',
        )

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='both',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x',
                '0',
                '--y',
                '0',
                '--z',
                '0',
                '--roll',
                '0',
                '--pitch',
                '0',
                '--yaw',
                '0',
                '--frame-id',
                'world',
                '--child-frame-id',
                'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='both',
        ),
        base_tf_node,
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_file],
                    parameters=(
                        moveit_parameters if moveit_enabled else [{'use_sim_time': use_sim_time}]
                    ),
                    additional_env={
                        'GTK_PATH': '',
                        'GTK_EXE_PREFIX': '',
                        'GTK_MODULES': '',
                        'GDK_PIXBUF_MODULE_FILE': '',
                        'GDK_PIXBUF_MODULEDIR': '',
                        'GTK_IM_MODULE_FILE': '',
                    },
                    condition=IfCondition(LaunchConfiguration('rviz')),
                    output='both',
                )
            ],
        ),
        Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                ParameterFile(controller_file, allow_substs=True),
            ],
            output='both',
            on_exit=Shutdown(),
        ),
    ]
    if moveit_enabled:
        nodes.append(
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                parameters=moveit_parameters,
                output='both',
            )
        )
    for controller in ('joint_state_broadcaster', 'whole_body_controller'):
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller,
                    '--controller-manager',
                    '/controller_manager',
                    '--param-file',
                    controller_file,
                    '--controller-manager-timeout',
                    '30',
                ],
                output='both',
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='Run MuJoCo without its graphical viewer',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                description='Open RViz with the robot model and TF tree',
            ),
            DeclareLaunchArgument(
                'fixed_base',
                default_value='true',
                description='Weld the pelvis for arm development without a balance controller',
            ),
            DeclareLaunchArgument(
                'moveit',
                default_value='false',
                description='Start MoveIt move_group and use its RViz configuration',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description=(
                    'Use MuJoCo /clock. Disabled by default because the current '
                    'Jazzy plugin can publish non-monotonic time.'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
