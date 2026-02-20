#!/usr/bin/env python3
"""
Dual Robot Launch File for Digital Twin

Spawns two TurtleBot3 robots in Gazebo:
- /real/ namespace: Represents the physical robot
- /twin/ namespace: Represents the digital twin simulation

Week 1 Day 4: Basic spawning with namespace separation
Week 4-5: Will add full sensor synchronization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for dual TurtleBot3 setup."""

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    training_mode = LaunchConfiguration('training_mode', default='false')
    baseline_samples = LaunchConfiguration('baseline_samples', default='600')
    model_path = LaunchConfiguration('model_path', default='~/.ros/digital_twin/anomaly_model.pkl')
    use_physical_real = LaunchConfiguration('use_physical_real', default='false')

    # Robot positions (separated to avoid collision)
    real_robot_x = LaunchConfiguration('real_x', default='0.0')
    real_robot_y = LaunchConfiguration('real_y', default='0.0')
    twin_robot_x = LaunchConfiguration('twin_x', default='2.0')
    twin_robot_y = LaunchConfiguration('twin_y', default='0.0')

    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # URDF model path - use TURTLEBOT3_MODEL env var (default: waffle_pi)
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    urdf_file = os.path.join(
        pkg_turtlebot3_gazebo,
        'urdf',
        f'turtlebot3_{turtlebot3_model}.urdf'
    )

    # World file (using default TurtleBot3 world)
    world_file = os.path.join(
        pkg_turtlebot3_gazebo,
        'worlds',
        'empty_world.world'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'training_mode',
            default_value='false',
            description='If true, collect baseline and train IsolationForest model on startup'
        ),
        DeclareLaunchArgument(
            'baseline_samples',
            default_value='600',
            description='Number of baseline samples to collect before training (default: 600 ~ 60s @ 10Hz)'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='~/.ros/digital_twin/anomaly_model.pkl',
            description='Path to save/load anomaly model (pickle)'
        ),
        DeclareLaunchArgument(
            'use_physical_real',
            default_value='false',
            description='If true, do NOT spawn the /real Gazebo robot; instead bridge physical /odom,/scan into /real/*'
        ),
        DeclareLaunchArgument(
            'real_x',
            default_value='0.0',
            description='X position for real robot'
        ),
        DeclareLaunchArgument(
            'real_y',
            default_value='0.0',
            description='Y position for real robot'
        ),
        DeclareLaunchArgument(
            'twin_x',
            default_value='2.0',
            description='X position for twin robot'
        ),
        DeclareLaunchArgument(
            'twin_y',
            default_value='0.0',
            description='Y position for twin robot'
        ),

        # Launch Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Launch Gazebo client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Spawn REAL robot with /real namespace (simulation mode only)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_real_robot',
            namespace='real',
            output='screen',
            condition=UnlessCondition(use_physical_real),
            arguments=[
                '-file', urdf_file,
                '-entity', 'real_turtlebot3',
                '-robot_namespace', 'real',
                '-x', real_robot_x,
                '-y', real_robot_y,
                '-z', '0.01'
            ],
        ),

        # Spawn TWIN robot with /twin namespace
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_twin_robot',
            namespace='twin',
            output='screen',
            arguments=[
                '-file', urdf_file,
                '-entity', 'twin_turtlebot3',
                '-robot_namespace', 'twin',
                '-x', twin_robot_x,
                '-y', twin_robot_y,
                '-z', '0.01'
            ],
        ),

        # Robot state publisher for REAL robot (simulation mode only)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='real_robot_state_publisher',
            namespace='real',
            output='screen',
            condition=UnlessCondition(use_physical_real),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': open(urdf_file).read()
            }],
            remappings=[
                ('/tf', '/real/tf'),
                ('/tf_static', '/real/tf_static'),
            ]
        ),

        # Bridge physical robot topics into /real/* (physical mode only)
        Node(
            package='digital_twin_pkg',
            executable='real_robot_bridge_node',
            name='real_robot_bridge',
            output='screen',
            condition=IfCondition(use_physical_real),
            parameters=[{
                # Most TurtleBot3 stacks publish these un-namespaced
                'odom_in': '/odom',
                'scan_in': '/scan',
                'cmd_vel_in': '/cmd_vel',
                'namespace_out': '/real',
                # Set false if you don't want /real/cmd_vel mirroring
                'republish_cmd_vel': True,
            }],
        ),

        # Robot state publisher for TWIN robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='twin_robot_state_publisher',
            namespace='twin',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': open(urdf_file).read()
            }],
            remappings=[
                ('/tf', '/twin/tf'),
                ('/tf_static', '/twin/tf_static'),
            ]
        ),

        # Command synchronizer node
        Node(
            package='digital_twin_pkg',
            executable='command_synchronizer_node',
            name='command_synchronizer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'sync_delay': 0.0,  # No delay for perfect sync
            }],
        ),

        # Digital twin monitor node
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
    ])
