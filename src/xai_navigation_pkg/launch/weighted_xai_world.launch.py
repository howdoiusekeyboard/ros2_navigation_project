#!/usr/bin/env python3
"""
Launch Gazebo with weighted XAI demo world (person, cart, furniture models)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # World file path (in project root)
    world_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_navigation_project',
        'worlds',
        'weighted_xai_demo.world'
    )

    # URDF file path - WAFFLE_PI has camera!
    urdf_file = os.path.join(
        pkg_turtlebot3_gazebo,
        'urdf',
        'turtlebot3_waffle_pi.urdf'
    )

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Gazebo launch
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver',
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen',
    )

    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn TurtleBot3 Waffle Pi (has camera!) at spawn marker position
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle_pi',
            '-file', os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
            '-x', '-6.0',
            '-y', '-6.0',
            '-z', '0.01'
        ],
        output='screen',
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_turtlebot_cmd,
    ])
