import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Paths
    simulation_launch = os.path.join(pkg_autonomous_forklift, 'launch', 'simulation.launch.py')
    slam_launch = os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
    rviz_config = os.path.join(pkg_autonomous_forklift, 'rviz', 'rviz_copiaDEFINITIVA.rviz')
    world_file = os.path.join(pkg_autonomous_forklift, 'worlds', 'mundo.xml')

    # 1. Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={'world_file': world_file}.items()
    )

    # 2. SLAM Toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch)
    )

    # 3. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        simulation,
        slam,
        rviz
    ])
