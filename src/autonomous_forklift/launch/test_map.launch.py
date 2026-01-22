import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Map Server
    pkg_path = get_package_share_directory('autonomous_forklift')
    map_file = os.path.join(pkg_path, 'maps', 'mundo_map.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager
    ])
