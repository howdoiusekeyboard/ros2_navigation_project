#!/usr/bin/env python3
"""
Rosbridge Monitor Launch File

Uses rosbridge WebSocket to get robot data instead of DDS.
This works reliably across subnets where DDS discovery fails.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip', default='10.30.96.171')
    training_mode = LaunchConfiguration('training_mode', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='10.30.96.171'),
        DeclareLaunchArgument('training_mode', default_value='false'),

        # Rosbridge relay - gets data via WebSocket
        Node(
            package='digital_twin_pkg',
            executable='rosbridge_relay_node',
            name='rosbridge_relay',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'rosbridge_port': 9090,
            }],
        ),

        # Behavior monitor - detects anomalies based on behavior patterns
        Node(
            package='digital_twin_pkg',
            executable='behavior_monitor_node',
            name='behavior_monitor',
            output='screen',
            parameters=[{
                'window_size': 50,
                'velocity_threshold': 0.1,
                'scan_threshold': 0.5,
            }],
        ),
    ])
