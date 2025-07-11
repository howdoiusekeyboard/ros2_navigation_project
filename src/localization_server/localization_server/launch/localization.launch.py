import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the AMCL configuration file within this package
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')

    # Path to the map file we saved in the 'map_server' package (from Part 1)
    # *** Adjust 'map_server' if your map saving package had a different name ***
    # *** Adjust 'turtlebot_area.yaml' if your map file had a different name ***
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'turtlebot_area.yaml')

    # List of nodes to be managed by the lifecycle manager
    lifecycle_nodes = ['map_server', 'amcl']

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},         # Use simulation time
                        {'yaml_filename': map_file}]    # Path to map file
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]                     # Path to AMCL parameters
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',      # Node name can be customized
            output='screen',
            parameters=[{'use_sim_time': True},         # Use simulation time
                        {'autostart': True},            # Automatically start managed nodes
                        {'node_names': lifecycle_nodes}] # Nodes to manage
        )
    ])