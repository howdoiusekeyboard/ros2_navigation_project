#!/usr/bin/env python3
"""
Twin Monitor Only Launch File

This launch file ONLY starts the monitoring nodes, assuming:
1. Real robot is running (bringup + rosbridge)
2. Gazebo twin is launched separately via: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

This is more reliable than trying to spawn everything in one launch file.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    training_mode = LaunchConfiguration('training_mode', default='false')
    baseline_samples = LaunchConfiguration('baseline_samples', default='600')
    model_path = LaunchConfiguration('model_path', default='~/.ros/digital_twin/anomaly_model.pkl')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (set true if using Gazebo clock)'
        ),
        DeclareLaunchArgument(
            'training_mode',
            default_value='false',
            description='If true, collect baseline and train IsolationForest'
        ),
        DeclareLaunchArgument(
            'baseline_samples',
            default_value='600',
            description='Samples to collect before training'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='~/.ros/digital_twin/anomaly_model.pkl',
            description='Path to save/load anomaly model'
        ),

        # Real robot bridge - republishes /odom, /scan to /real/*
        Node(
            package='digital_twin_pkg',
            executable='real_robot_bridge_node',
            name='real_robot_bridge',
            output='screen',
            parameters=[{
                'odom_in': '/odom',
                'scan_in': '/scan',
                'cmd_vel_in': '/cmd_vel',
                'namespace_out': '/real',
                'republish_cmd_vel': True,
            }],
        ),

        # Digital twin monitor - compares /real/* vs /twin/*
        Node(
            package='digital_twin_pkg',
            executable='digital_twin_monitor_node',
            name='digital_twin_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'anomaly_threshold': -0.5,
                'update_rate': 10.0,
                'training_mode': training_mode,
                'baseline_samples': baseline_samples,
                'model_path': model_path,
            }],
        ),

        # Command synchronizer - forwards /cmd_vel to both robots
        Node(
            package='digital_twin_pkg',
            executable='command_synchronizer_node',
            name='command_synchronizer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'sync_delay': 0.0,
            }],
        ),
    ])
