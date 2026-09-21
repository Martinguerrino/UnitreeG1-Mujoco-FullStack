"""Standalone launch: MuJoCo/DDS, ROS bridge, SLAM or AMCL, Nav2 and RViz."""

from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


def prepare(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    map_path = LaunchConfiguration('map').perform(context)
    if mode == 'localization' and not Path(map_path).is_file():
        raise ValueError('localization requires map:=/absolute/path/map.yaml')
    nav = Path(get_package_share_directory('nav2_bringup'))
    params = yaml.safe_load((nav / 'params/nav2_params.yaml').read_text())
    # Start from defaults of the installed Jazzy release so plugin lists match.
    for node in params.values():
        if 'ros__parameters' in node:
            node['ros__parameters']['use_sim_time'] = True
            node['ros__parameters']['enable_stamped_cmd_vel'] = False
    controller = params['controller_server']['ros__parameters']
    controller['min_y_velocity_threshold'] = 0.001
    # A humanoid may need to rotate in place for a long time before a path
    # whose first segment is behind it becomes drivable.  SimpleProgressChecker
    # only counts translation, so it falsely reports "Failed to make progress"
    # during a valid turn.  PoseProgressChecker also counts angular progress.
    controller['progress_checker'].update(
        plugin='nav2_controller::PoseProgressChecker',
        required_movement_radius=0.08,
        required_movement_angle=0.12,
        movement_time_allowance=45.0,
    )
    controller['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': 0.45,
        'lookahead_dist': 0.6,
        'min_lookahead_dist': 0.4,
        'max_lookahead_dist': 0.9,
        'use_rotate_to_heading': True,
        'rotate_to_heading_angular_vel': 1.0,
        'rotate_to_heading_min_angle': 1.5707963267948966,  # 90 degrees (50% of 180)
        'max_angular_accel': 4.0,
        'allow_reversing': False,
        'use_collision_detection': True,
        'transform_tolerance': 0.5,
        'min_approach_linear_velocity': 0.1,
        # Once the goal position is reached, keep correcting its requested
        # heading. The resulting angular command goes through the exact same
        # backwards/forwards turn assist used for the initial alignment.
        'stateful': True,
    }
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        costmap.update(use_sim_time=True, robot_base_frame='base_link', robot_radius=0.4)
        costmap['inflation_layer']['inflation_radius'] = 0.7
    smoother = params['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[0.55, 0.0, 1.0], min_velocity=[-0.3, 0.0, -1.0],
                    max_accel=[0.8, 0.0, 4.0], max_decel=[-0.8, 0.0, -4.0])
    params['behavior_server']['ros__parameters'].update(
        max_rotational_vel=1.0, min_rotational_vel=0.1, rotational_acc_lim=4.0
    )
    params['amcl']['ros__parameters']['base_frame_id'] = 'base_link'
    if 'collision_monitor' in params:
        params['collision_monitor']['ros__parameters']['base_frame_id'] = 'base_link'
    # Keep the resolved launch configuration for reproducibility.
    directory = ROOT / 'results/navigation'
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='nav2_',
                                     dir=directory, delete=False) as handle:
        yaml.safe_dump(params, handle)
        params_path = handle.name
    simulator = ExecuteProcess(
        cmd=[LaunchConfiguration('sim_python'), str(ROOT / 'scripts/navigation_sim.py'),
             '--viewer', LaunchConfiguration('viewer')],
        output='screen',
    )
    bridge = ExecuteProcess(
        cmd=['/usr/bin/python3', str(ROOT / 'scripts/navigation_bridge.py')], output='screen'
    )
    actions = [bridge, simulator]
    # The simulator owns the DDS-side lifecycle.  Shutting down on both its
    # exit and the bridge's exit races two ROS shutdown requests and emits a
    # misleading "ROS adapter not running" error during otherwise normal
    # cleanup.
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=simulator,
        on_exit=[EmitEvent(event=Shutdown(reason='Navigation simulation stopped'))],
    )))
    if mode == 'mapping':
        slam = Path(get_package_share_directory('slam_toolbox'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam / 'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              'slam_params_file': str(ROOT / 'configs/navigation/slam.yaml')}.items(),
        ))
    else:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav / 'launch/localization_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'map': map_path,
                              'params_file': params_path, 'autostart': 'true',
                              # Nav2 Jazzy evaluates this through PythonExpression;
                              # use a Python bool spelling, not lowercase YAML text.
                              'use_composition': 'False'}.items(),
        ))
    # MuJoCo spends its warmup period creating the DDS bridge and the first
    # odom -> base_link transform. Nav2 controller_server aborts if activated
    # before that transform exists, so bring Nav2 up after the bridge has had
    # time to publish it.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav / 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'params_file': params_path,
                          'autostart': 'true', 'use_composition': 'False'}.items(),
    )
    actions.append(TimerAction(period=8.0, actions=[nav2]))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(TimerAction(period=8.0, actions=[
            Node(package='rviz2', executable='rviz2',
                 arguments=['-d', str(nav / 'rviz/nav2_default_view.rviz')],
                 parameters=[{'use_sim_time': True}])
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('sim_python', default_value=
                              '/home/martin/miniforge3/envs/mujoco-unitree/bin/python'),
        OpaqueFunction(function=prepare),
    ])
