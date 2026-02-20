#!/usr/bin/env python3
"""
Real Robot Monitor Launch File

Monitors real robot without requiring Gazebo twin.
Publishes sensor data to dashboard and detects anomalies based on 
robot behavior patterns (velocity, scan consistency, etc.)

Use this when:
- You want to monitor the real robot without running Gazebo
- Gazebo twin setup is having issues
- You just want basic anomaly detection on the physical robot
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    training_mode = LaunchConfiguration('training_mode', default='false')
    baseline_samples = LaunchConfiguration('baseline_samples', default='600')
    model_path = LaunchConfiguration('model_path', default='~/.ros/digital_twin/anomaly_model.pkl')

    return LaunchDescription([
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

        # Real robot anomaly monitor (simplified - monitors real robot patterns)
        Node(
            package='digital_twin_pkg',
            executable='digital_twin_monitor_node',
            name='real_robot_monitor',
            output='screen',
            parameters=[{
                'anomaly_threshold': -0.5,
                'update_rate': 10.0,
                'training_mode': training_mode,
                'baseline_samples': baseline_samples,
                'model_path': model_path,
            }],
            # Remap twin topics to real topics for self-comparison baseline
            remappings=[
                ('/twin/odom', '/real/odom'),
                ('/twin/scan', '/real/scan'),
            ],
        ),
    ])
