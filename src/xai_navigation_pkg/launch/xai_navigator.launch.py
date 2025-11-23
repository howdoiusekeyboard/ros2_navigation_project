#!/usr/bin/env python3
"""
XAI Navigator Launch File

Launches the XAI Navigator Node with configurable parameters.
Use with Nav2 navigation stack for full functionality.

Usage:
    ros2 launch xai_navigation_pkg xai_navigator.launch.py

    With parameters:
    ros2 launch xai_navigation_pkg xai_navigator.launch.py enable_logging:=true backend_url:=http://localhost:8000
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Generate launch description for XAI Navigator."""
    
    # Get Gemini API key from environment
    gemini_api_key = os.getenv('GEMINI_API_KEY', '')

    # Declare launch arguments
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable decision logging to database'
    )

    backend_url_arg = DeclareLaunchArgument(
        'backend_url',
        default_value='http://localhost:8000',
        description='Backend API URL for sync'
    )

    sync_interval_arg = DeclareLaunchArgument(
        'sync_interval',
        default_value='5.0',
        description='Backend sync interval in seconds'
    )

    explanation_level_arg = DeclareLaunchArgument(
        'explanation_level',
        default_value='detailed',
        description='Explanation detail level: simple, detailed, or debug'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )
    
    # NEW: Explanation arguments
    enable_explanations_arg = DeclareLaunchArgument(
        'enable_explanations',
        default_value='true',
        description='Enable explanation generation'
    )
    
    gemini_api_key_arg = DeclareLaunchArgument(
        'gemini_api_key',
        default_value=gemini_api_key,
        description='Gemini API key for explanations'
    )

    # XAI Navigator Node
    xai_navigator_node = Node(
        package='xai_navigation_pkg',
        executable='xai_navigator_node',
        name='xai_navigator_node',
        output='screen',
        parameters=[{
            'enable_logging': LaunchConfiguration('enable_logging'),
            'backend_url': LaunchConfiguration('backend_url'),
            'sync_interval': LaunchConfiguration('sync_interval'),
            'explanation_level': LaunchConfiguration('explanation_level'),
            'explanation_level': LaunchConfiguration('explanation_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_explanations': LaunchConfiguration('enable_explanations'),
            'gemini_api_key': LaunchConfiguration('gemini_api_key')
        }],
        remappings=[
            # Add any topic remappings here if needed
            # ('/plan', '/global_costmap/plan'),
        ]
    )

    return LaunchDescription([
        # Launch arguments
        enable_logging_arg,
        backend_url_arg,
        sync_interval_arg,
        explanation_level_arg,
        explanation_level_arg,
        use_sim_time_arg,
        enable_explanations_arg,
        gemini_api_key_arg,

        # Nodes
        xai_navigator_node
    ])
