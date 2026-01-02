#!/usr/bin/env python3

"""
Simple Robot Launch File
This demonstrates how to launch a robot simulation with Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simple_robot = get_package_share_directory('simple_robot_description')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=os.path.join(
        pkg_simple_robot, 'worlds', 'simple_room.world'))

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(
                os.path.join(pkg_simple_robot, 'urdf', 'simple_robot.urdf')
            ).read()
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_simple_robot, 'worlds', 'simple_room.world'),
            description='SDF world file'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])