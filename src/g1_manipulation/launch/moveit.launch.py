"""Convenient entry point for MuJoCo, MoveIt and its RViz panel."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('g1_bringup')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{bringup}/launch/simulation.launch.py'),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'fixed_base': 'true',
            'moveit': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('headless', default_value='false'),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            simulation,
        ]
    )
