import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')
    mvsim_world_file = os.path.join(pkg_autonomous_forklift, 'mvsim', 'warehouse.mvsim.xml')
    rviz_config_file = os.path.join(pkg_autonomous_forklift, 'rviz', 'visualization.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value=mvsim_world_file,
            description='Path to the MVSim world file'
        ),

        # MVSim Node
        Node(
            package='mvsim',
            executable='mvsim_node',
            name='mvsim',
            output='screen',
            parameters=[
                {'world_file': LaunchConfiguration('world_file')},
                {'headless': False} # Show MVSim GUI
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
