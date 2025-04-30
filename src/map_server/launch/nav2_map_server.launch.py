import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
# If using Nav2 params file later, you might need:
# from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Correctly locate the map file within this package's installed share directory
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'turtlebot_area.yaml')

    # You can define parameters directly or use a Nav2 params file
    # Example direct parameters:
    map_server_params = { 'use_sim_time': True, # Use simulation time
                         'yaml_filename': map_file}
    lifecycle_nodes = ['map_server'] # List of nodes to be managed by lifecycle manager

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[map_server_params] # Pass parameters
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper', # Node name can be customized
            output='screen',
            parameters=[{'use_sim_time': True}, # Use simulation time
                        {'autostart': True},      # Automatically start managed nodes
                        {'node_names': lifecycle_nodes}] # Specify nodes to manage
        )
    ])