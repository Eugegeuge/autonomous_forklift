import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')

    # Paths
    navigation_launch = os.path.join(pkg_autonomous_forklift, 'launch', 'navigation.launch.py')
    graph_file = os.path.join(pkg_autonomous_forklift, 'config', 'warehouse_graph.geojson')

    # 1. Navigation (Sim + Nav2 + RViz)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )

    waypoint_follower = Node(
        package='autonomous_forklift',
        executable='waypoint_follower.py',
        name='graph_navigator',
        output='screen',
        parameters=[{'graph_file': graph_file}]
    )

    # 3. Interface Node (GUI)
    interface = Node(
        package='autonomous_forklift',
        executable='interface_node.py',
        name='interface_node',
        output='screen'
    )

    return LaunchDescription([
        navigation,
        waypoint_follower,
        interface
    ])
