import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to configuration files
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    # Add this line near the top of the function, after the other yaml path definitions
    behavior_tree_xml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'behavior.xml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')

    # Nodes to launch
    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server', # Name in the launch file can be different from executable
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            # Pass parameters including the resolved path to the BT XML
            parameters=[bt_navigator_yaml, # Load parameters from YAML first
                        {'default_nav_to_pose_bt_xml': behavior_tree_xml}]), # Then override/add specific params

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner', # Customized name
            output='screen',
            parameters=[{'autostart': True},
                        # List ALL nodes managed by this lifecycle manager
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server', # Use the 'name' given in the Node definition
                                        'bt_navigator']}])
    ])