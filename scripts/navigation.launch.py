"""Standalone launch: MuJoCo/DDS, ROS bridge, SLAM or AMCL, Nav2 and RViz."""

from pathlib import Path
import importlib.util
import tempfile
import sys
import sqlite3
from datetime import datetime, timezone

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

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
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROOT = Path(__file__).resolve().parents[1]


def prepare(context):
    ideal = LaunchConfiguration('ideal').perform(context) == 'true'
    slam_backend = LaunchConfiguration('slam').perform(context)
    sensor_slam = slam_backend != 'slam_toolbox'
    camera_fps = float(LaunchConfiguration('camera_fps').perform(context))
    if camera_fps == 0:
        camera_fps = 20.0 if slam_backend == 'rgb-d' else 10.0
    if camera_fps <= 0:
        raise ValueError('camera_fps must be positive (or 0 for backend default)')
    real_odom = LaunchConfiguration('real_odom').perform(context) == 'true' or sensor_slam
    if real_odom:
        for dependency in ('cv2', 'scipy'):
            if importlib.util.find_spec(dependency) is None:
                raise RuntimeError('--real-odom requires system Python OpenCV and SciPy: '
                                   'sudo apt install python3-opencv python3-scipy')
    if ideal and real_odom:
        raise ValueError('--real-odom and --slam require mounted sensors; remove --ideal')
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    if sensor_slam and mode != 'mapping':
        raise ValueError('3D backends currently support mapping; mode:=localization uses slam_toolbox/AMCL')
    map_path = LaunchConfiguration('map').perform(context)
    if mode == 'localization' and not Path(map_path).is_file():
        raise ValueError('localization requires map:=/absolute/path/map.yaml')
    nav = Path(get_package_share_directory('nav2_bringup'))
    description = Path(get_package_share_directory('g1_description'))
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
        if slam_backend == 'rgb-d':
            # Camera odometry describes past captures. Requiring odom at "now"
            # for scan motion compensation stalls the monitor on every command.
            params['collision_monitor']['ros__parameters']['base_shift_correction'] = False
    # Keep the resolved launch configuration for reproducibility.
    directory = ROOT / 'results/navigation'
    directory.mkdir(parents=True, exist_ok=True)
    slam_output = LaunchConfiguration('slam_output').perform(context)
    if sensor_slam:
        slam_output = Path(slam_output).expanduser().resolve() if slam_output else directory / (slam_backend + '_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f'))
        slam_output.mkdir(parents=True, exist_ok=True)
        database = slam_output / 'rtabmap.db'
        if slam_backend == 'rgb-d' and database.exists():
            try:
                with sqlite3.connect(database.as_uri() + '?mode=ro', uri=True) as connection:
                    tables = {row[0] for row in connection.execute(
                        "SELECT name FROM sqlite_master WHERE type='table'")}
                if not {'Node', 'Data', 'Statistics', 'Admin'}.issubset(tables):
                    raise ValueError('incomplete RTAB-Map schema')
            except (sqlite3.Error, ValueError) as error:
                raise ValueError(f'Cannot reuse {database}: {error}. '
                                 'Choose a new slam_output directory for this session.') from error
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='nav2_',
                                     dir=directory, delete=False) as handle:
        yaml.safe_dump(params, handle)
        params_path = handle.name
    rviz_config = yaml.safe_load((nav / 'rviz/nav2_default_view.rviz').read_text())
    displays = rviz_config['Visualization Manager']['Displays']
    for display in displays:
        if display.get('Name') == 'RobotModel':
            display['Enabled'] = display['Value'] = True
            display['Description Topic']['Durability Policy'] = 'Transient Local'
    displays.extend([
        {
            'Class': 'rviz_default_plugins/PointCloud2', 'Name': 'MID-360 3D',
            'Enabled': True, 'Value': True, 'Style': 'Points', 'Size (Pixels)': 2,
            'Color Transformer': 'AxisColor', 'Axis': 'Z', 'Decay Time': 0.3,
            'Topic': {'Value': '/mid360/points', 'Depth': 5,
                      'Reliability Policy': 'Best Effort',
                      'Durability Policy': 'Volatile', 'History Policy': 'Keep Last'},
        },
        {
            'Class': 'rviz_default_plugins/Image', 'Name': 'D435i RGB',
            'Enabled': True, 'Value': True, 'Normalize Range': True,
            'Topic': {'Value': '/camera/color/image_raw', 'Depth': 5,
                      'Reliability Policy': 'Best Effort',
                      'Durability Policy': 'Volatile', 'History Policy': 'Keep Last'},
        },
        {
            'Class': 'rviz_default_plugins/Image', 'Name': 'D435i Depth',
            'Enabled': True, 'Value': True, 'Normalize Range': True,
            'Min Value': 0.0, 'Max Value': 10.0,
            'Topic': {'Value': '/camera/depth/image_rect_raw', 'Depth': 5,
                      'Reliability Policy': 'Best Effort',
                      'Durability Policy': 'Volatile', 'History Policy': 'Keep Last'},
        },
        {
            'Class': 'rviz_common/Group', 'Name': 'D435i Projection 3D',
            'Enabled': False, 'Value': False,
            'Displays': [
                {
                    'Class': 'rviz_default_plugins/CameraInfo', 'Name': 'Camera field of view',
                    'Enabled': True, 'Value': True, 'Far clip': 0.8,
                    'Show edges': True, 'Show polygons': False,
                    'Edge color': '0; 220; 255',
                    'Topic': {'Value': '/camera/color/camera_info', 'Depth': 1,
                              'Reliability Policy': 'Reliable',
                              'Durability Policy': 'Volatile', 'History Policy': 'Keep Last'},
                },
                {
                    'Class': 'rviz_default_plugins/Marker', 'Name': 'RGB image plane',
                    'Enabled': True, 'Value': True,
                    'Topic': {'Value': '/camera/color/image_plane', 'Depth': 1,
                              'Reliability Policy': 'Best Effort',
                              'Durability Policy': 'Volatile', 'History Policy': 'Keep Last'},
                },
            ],
        },
    ])
    if sensor_slam:
        displays.append({
            'Class': 'rviz_default_plugins/PointCloud2', 'Name': 'SLAM 3D map',
            'Enabled': True, 'Value': True, 'Style': 'Points', 'Size (Pixels)': 2,
            'Color Transformer': 'RGB8' if slam_backend == 'rgb-d' else 'AxisColor',
            'Axis': 'Z', 'Decay Time': 0.0,
            'Topic': {'Value': '/slam/map_points', 'Depth': 1,
                      'Reliability Policy': 'Reliable',
                      'Durability Policy': 'Volatile', 'History Policy': 'Keep Last'},
        })
        if slam_backend == 'rgb-d':
            displays.append({
                'Class': 'rviz_default_plugins/Marker', 'Name': 'SLAM 3D mesh',
                'Enabled': False, 'Value': False, 'Alpha': 1.0,
                'Topic': {'Value': '/slam/map_mesh', 'Depth': 1,
                          'Reliability Policy': 'Reliable',
                          'Durability Policy': 'Transient Local', 'History Policy': 'Keep Last'},
            })
    rviz_config['Visualization Manager']['Views']['Current'] = {
        'Class': 'rviz_default_plugins/Orbit', 'Name': 'G1 3D', 'Distance': 7.0,
        'Pitch': 0.55, 'Yaw': 0.8, 'Target Frame': 'base_link',
        'Focal Point': {'X': 0.0, 'Y': 0.0, 'Z': 0.8},
    }
    # Nav2's saved Qt layout has no camera docks. Restore our layout so both
    # image panels are visible beside the 3D view instead of inheriting it.
    rviz_config['Window Geometry'] = yaml.safe_load(
        (ROOT / 'configs/navigation_rviz_layout.yaml').read_text()
    )
    rviz_path = directory / 'g1_navigation.rviz'
    rviz_path.write_text(yaml.safe_dump(rviz_config, sort_keys=False))
    description_xml = Command([
        FindExecutable(name='xacro'), ' ',
        str(description / 'urdf/g1_29dof.urdf.xacro'),
    ]).perform(context)
    if LaunchConfiguration('dexterity').perform(context) == 'claw':
        from g1_backend.dexterity import description_with_claws

        description_xml = description_with_claws(description_xml)
    robot_description = {
        'robot_description': ParameterValue(description_xml, value_type=str)
    }
    simulator = ExecuteProcess(
        cmd=[LaunchConfiguration('sim_python'), str(ROOT / 'scripts/navigation_sim.py'),
             '--viewer', LaunchConfiguration('viewer'),
             '--duration', LaunchConfiguration('duration'),
             '--camera-fps', str(camera_fps),
             '--ideal', LaunchConfiguration('ideal'),
             '--dexterity', LaunchConfiguration('dexterity')],
        output='screen',
    )
    bridge = ExecuteProcess(
        cmd=['/usr/bin/python3', str(ROOT / 'scripts/navigation_bridge.py'),
             '--ros-args', '-p', 'real_odom:=' + str(real_odom).lower()], output='screen'
    )
    actions = [
        bridge,
        simulator,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description, {'use_sim_time': True}], output='screen'),
    ]
    if real_odom and not sensor_slam:
        estimator = ExecuteProcess(
            cmd=['/usr/bin/python3', str(ROOT / 'scripts/navigation_odometry.py')],
            output='screen',
        )
        actions.append(estimator)
    if sensor_slam:
        estimator = ExecuteProcess(
            cmd=['/usr/bin/python3', str(ROOT / 'scripts/navigation_slam_adapter.py'),
                 '--ros-args', '-p', 'use_sim_time:=true', '-p', 'backend:=' + slam_backend,
                 '-p', 'output_dir:=' + str(slam_output)], output='screen',
        )
        actions.append(estimator)
        if slam_backend == 'lidar3d':
            engine = Node(
                package='fast_lio', executable='fastlio_mapping',
                parameters=[str(ROOT / 'configs/navigation/fast_lio.yaml')],
                remappings=[('/Odometry', '/slam/odometry'),
                            ('/cloud_registered', '/slam/registered_points')],
                output='screen',
            )
            actions.append(engine)
        else:
            actions.append(ExecuteProcess(
                cmd=['/usr/bin/python3', str(ROOT / 'scripts/navigation_mesh.py'),
                     '--ros-args', '-p', 'use_sim_time:=true',
                     '-p', 'output_dir:=' + str(slam_output)], output='screen',
            ))
            rgbd_topics = [
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_rect_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
            ]
            engine = Node(
                package='rtabmap_odom', executable='rgbd_odometry',
                parameters=[{'use_sim_time': True, 'frame_id': 'pelvis',
                             'odom_frame_id': 'odom', 'publish_tf': False,
                             'approx_sync': False, 'qos': 2, 'qos_camera_info': 1,
                             'wait_for_transform': 0.2, 'Odom/Strategy': '1',
                             'Vis/CorType': '1',
                             'Vis/EstimationType': '0', 'Vis/InlierDistance': '0.03',
                             'Odom/KeyFrameThr': '0.0', 'Odom/VisKeyFrameThr': '0',
                             'Odom/GuessMotion': 'false', 'Vis/MinDepth': '0.5',
                             'Vis/MaxFeatures': '600',
                             'Vis/FeatureType': '8', 'Vis/MinInliers': '12'}],
                remappings=rgbd_topics + [('odom', '/slam/odometry')], output='screen',
            )
            mapping = Node(
                package='rtabmap_slam', executable='rtabmap',
                parameters=[{'use_sim_time': True, 'frame_id': 'base_link',
                             'map_frame_id': 'map', 'odom_frame_id': '',
                             'subscribe_depth': True, 'subscribe_rgb': True,
                             'subscribe_scan': False, 'approx_sync': True,
                             'approx_sync_max_interval': 0.2,
                             'qos_image': 2, 'qos_camera_info': 1, 'qos_odom': 1,
                             'database_path': str(slam_output / 'rtabmap.db'),
                             'wait_for_transform': 0.3, 'publish_tf': True,
                             'Rtabmap/DetectionRate': '2.0',
                             'Grid/Sensor': '1', 'Grid/CellSize': '0.1',
                             'Grid/RangeMin': '0.5',
                             'Grid/FootprintLength': '0.8', 'Grid/FootprintWidth': '0.8',
                             'Grid/FootprintHeight': '1.5',
                             'GridGlobal/FootprintRadius': '0.4',
                             'GridGlobal/MinSize': '20.0',
                             'Grid/RangeMax': '10.0', 'Grid/MaxObstacleHeight': '1.5',
                             'Grid/MinGroundHeight': '-0.1', 'Grid/MaxGroundHeight': '0.15',
                             'Grid/NormalsSegmentation': 'false',
                             'Grid/3D': 'true', 'map_always_update': True,
                             'cloud_output_voxelized': True}],
                remappings=rgbd_topics + [('odom', '/odom'),
                                         ('cloud_map', '/slam/map_points'),
                                         ('grid_map', '/map')], output='screen',
            )
            actions.extend([engine, mapping])
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=mapping, on_exit=lambda event, context: [
                    EmitEvent(event=Shutdown(reason='RTAB-Map stopped'))] if not context.is_shutdown else [],
            )))
    if not ideal and LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(ExecuteProcess(
            cmd=['/usr/bin/python3', str(ROOT / 'scripts/navigation_camera_plane.py')],
            output='screen',
        ))
    # Either essential process exiting must stop the stack. Emit shutdown only
    # once so simultaneous exits do not race ROS cleanup.
    shutdown_requested = False

    def stop_navigation(event, context):
        nonlocal shutdown_requested
        if shutdown_requested or context.is_shutdown:
            return []
        shutdown_requested = True
        return [EmitEvent(event=Shutdown(reason='Navigation process stopped'))]

    for process in (simulator, bridge, *([estimator] if real_odom else []),
                    *([engine] if sensor_slam else [])):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=stop_navigation,
        )))
    if mode == 'mapping' and not sensor_slam:
        slam = Path(get_package_share_directory('slam_toolbox'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam / 'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              'slam_params_file': str(ROOT / 'configs/navigation/slam.yaml')}.items(),
        ))
    elif mode == 'localization':
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
    if LaunchConfiguration('nav2').perform(context) == 'true':
        if sensor_slam:
            readiness = ExecuteProcess(
                cmd=['/usr/bin/python3', str(ROOT / 'scripts/navigation_slam_ready.py'),
                     '--ros-args', '-p', 'use_sim_time:=true'], output='screen',
            )

            def start_navigation(event, context):
                if context.is_shutdown:
                    return []
                if event.returncode != 0:
                    return [EmitEvent(event=Shutdown(reason='Sensor SLAM did not become ready'))]
                return [nav2]

            actions.extend([readiness, RegisterEventHandler(OnProcessExit(
                target_action=readiness, on_exit=start_navigation,
            ))])
        else:
            actions.append(TimerAction(period=8.0, actions=[nav2]))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(TimerAction(period=8.0, actions=[
            Node(package='rviz2', executable='rviz2',
                 arguments=['-d', str(nav / 'rviz/nav2_default_view.rviz') if ideal
                            else str(rviz_path)],
                 parameters=[{'use_sim_time': True}])
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('slam', default_value='slam_toolbox',
                              choices=['slam_toolbox', 'lidar3d', 'rgb-d']),
        DeclareLaunchArgument('slam_output', default_value=''),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('duration', default_value='3600.0'),
        DeclareLaunchArgument('camera_fps', default_value='0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument('dexterity', default_value='hand', choices=['hand', 'claw']),
        DeclareLaunchArgument('real_odom', default_value='false'),
        DeclareLaunchArgument('ideal', default_value='false'),
        DeclareLaunchArgument('sim_python', default_value=
                              '/home/martin/miniforge3/envs/mujoco-unitree/bin/python'),
        OpaqueFunction(function=prepare),
    ])
