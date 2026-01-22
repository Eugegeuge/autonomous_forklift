import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_forklift')
    
    # Paths
    world_file_path = os.path.join(pkg_share, 'mvsim_models', 'forklift.world.xml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'forklift.rviz') # We will need to create this or let rviz create it

    # Arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=world_file_path,
        description='Path to the Mvsim world file'
    )

    # Nodes
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim_node',
        output='screen',
        parameters=[
            {'world_file': LaunchConfiguration('world_file')},
            {'headless': False}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_path] # Uncomment once we have a config
    )

    return LaunchDescription([
        world_file_arg,
        mvsim_node,
        # rviz_node
    ])
