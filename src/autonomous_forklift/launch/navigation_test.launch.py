import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')

    # 1. Simulation (Gazebo + Robot + Bridge)
    world_file = os.path.join(pkg_autonomous_forklift, 'worlds', 'warehouse_aruco.sdf')
    
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autonomous_forklift, 'launch', 'robot_sim.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2. Navigation Node
    nav_node = Node(
        package='autonomous_forklift',
        executable='waypoint_follower.py',
        name='navigation_node',
        output='screen'
    )

    # 3. Graph Visualizer
    graph_vis_node = Node(
        package='autonomous_forklift',
        executable='graph_visualizer.py',
        name='graph_visualizer',
        output='screen'
    )

    # 4. RViz
    rviz_config = os.path.join(pkg_autonomous_forklift, 'rviz', 'rviz_copiaDEFINITIVA.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        simulation,
        nav_node,
        graph_vis_node,
        rviz_node
    ])
