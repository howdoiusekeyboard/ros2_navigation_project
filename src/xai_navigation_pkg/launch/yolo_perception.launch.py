#!/usr/bin/env python3
"""
YOLO Perception Launch File

Launches the complete YOLO-World perception pipeline for Tesla-style
obstacle classification:

1. yolo_ros node (YOLO-World inference)
   - Subscribes to camera image
   - Publishes detections to /yolo/detections

2. yolo_class_setter node (our custom node)
   - Calls /yolo/set_classes service on startup
   - Configures Tesla-style priority classes

Usage:
    ros2 launch xai_navigation_pkg yolo_perception.launch.py

    # With custom parameters:
    ros2 launch xai_navigation_pkg yolo_perception.launch.py \
        device:=cuda:0 \
        threshold:=0.3

Topics Published:
    /yolo/detections (yolo_msgs/DetectionArray) - Object detections
    /yolo/tracking (yolo_msgs/DetectionArray) - Tracked objects
    /yolo/debug_image (sensor_msgs/Image) - Visualization image

Topics Subscribed:
    /camera/image_raw (sensor_msgs/Image) - TurtleBot3 camera input
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the launch description for YOLO perception."""

    # Get package directories
    yolo_bringup_dir = get_package_share_directory('yolo_bringup')

    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Device for YOLO inference (cpu, cuda:0, etc.)'
    )

    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.4',
        description='Detection confidence threshold (0.0-1.0)'
    )

    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/camera/image_raw',
        description='Input camera image topic (TurtleBot3 default)'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8s-worldv2.pt',
        description='YOLO-World model to use'
    )

    enable_arg = DeclareLaunchArgument(
        'enable',
        default_value='True',
        description='Enable YOLO inference on startup'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='yolo',
        description='Namespace for YOLO topics and services'
    )

    # Include yolo-world.launch.py from yolo_bringup
    yolo_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_bringup_dir, 'launch', 'yolo-world.launch.py')
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'device': LaunchConfiguration('device'),
            'enable': LaunchConfiguration('enable'),
            'threshold': LaunchConfiguration('threshold'),
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'namespace': LaunchConfiguration('namespace'),
            # Use reliable QoS for simulation
            'image_reliability': '2',
        }.items(),
    )

    # YOLO Class Setter node - configures Tesla-style classes
    yolo_class_setter_node = Node(
        package='xai_navigation_pkg',
        executable='yolo_class_setter',
        name='yolo_class_setter',
        output='screen',
        parameters=[{
            'publish_mapping': True,
        }],
    )

    return LaunchDescription([
        # Launch arguments
        device_arg,
        threshold_arg,
        input_image_topic_arg,
        model_arg,
        enable_arg,
        namespace_arg,

        # Nodes and includes
        yolo_world_launch,
        yolo_class_setter_node,
    ])
