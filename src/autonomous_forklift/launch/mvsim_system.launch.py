import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')
    pkg_smart_warehouse = get_package_share_directory('smart_warehouse')

    # World file
    world_file = os.path.join(pkg_autonomous_forklift, 'mvsim', 'warehouse.mvsim.xml')
    rviz_config = os.path.join(pkg_autonomous_forklift, 'rviz', 'visualization.rviz')
    graph_file = os.path.join(pkg_autonomous_forklift, 'config', 'warehouse_graph.geojson')

    # 1. MVSim Node
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[{'world_file': world_file}]
    )

    # 2. Navigation Node (MVSim uses /odom, /cmd_vel, /laser1 for single vehicle)
    nav_node = Node(
        package='autonomous_forklift',
        executable='waypoint_follower.py',
        name='navigation_node',
        output='screen',
        parameters=[{'graph_file': graph_file}],
        remappings=[
            ('/scan', '/laser1')  # Only laser needs remapping
        ]
    )

    # 3. Interface Node
    interface_node = Node(
        package='autonomous_forklift',
        executable='interface_node.py',
        name='interface_node',
        output='screen'
    )

    # 4. Graph Visualizer
    graph_vis_node = Node(
        package='autonomous_forklift',
        executable='graph_visualizer.py',
        name='graph_visualizer',
        output='screen'
    )

    # 5. Docking Node (no remapping needed for single vehicle)
    docking_node = Node(
        package='smart_warehouse',
        executable='docking_aruco',
        name='aruco_docking',
        output='screen',
        parameters=[{
            'target_dist': 0.5,
            'max_v': 0.3,
            'max_w': 0.5
        }]
    )

    # 6. RViz (Delayed)
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        mvsim_node,
        nav_node,
        interface_node,
        graph_vis_node,
        docking_node,
        rviz
    ])
