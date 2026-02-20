#!/usr/bin/env python3
"""
XAI Bridge Launch File

Launches the XAI Bridge Node that connects to robot's rosbridge
and relays Nav2 topics to local ROS2 for XAI processing.

Usage:
    ros2 launch digital_twin_pkg xai_bridge.launch.py robot_ip:=10.30.96.171
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip', default='10.30.96.171')
    rosbridge_port = LaunchConfiguration('rosbridge_port', default='9090')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='10.30.96.171',
            description='Robot IP address'
        ),
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='rosbridge WebSocket port on robot'
        ),

        # XAI Bridge Node
        Node(
            package='digital_twin_pkg',
            executable='xai_bridge_node',
            name='xai_bridge',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'rosbridge_port': rosbridge_port,
                'reconnect_interval': 5.0,
            }],
        ),

        # Behavior Monitor (optional, for anomaly detection)
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
