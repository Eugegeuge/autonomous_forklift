import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')

    # World file with ArUco
    world_file = os.path.join(pkg_autonomous_forklift, 'worlds', 'warehouse_aruco.sdf')

    # 1. Simulation (Gazebo + Robot) - spawn robot closer to ArUco
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autonomous_forklift, 'launch', 'robot_sim.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2. ArUco Marker Publisher (Detection)
    aruco_node = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 1.0,
            'reference_frame': 'base_link',
            'camera_frame': 'camera_optical_frame',
            'marker_frame': 'aruco_detectado',
            'marker_id': 26,
        }],
        remappings=[
            ('/camera_info', '/camera_sensor/camera_info'),
            ('/image', '/camera_sensor/image_raw'),
        ],
        output='screen'
    )

    # 3. RViz
    rviz_config = os.path.join(pkg_autonomous_forklift, 'rviz', 'visualization.rviz')
    rviz = TimerAction(
        period=5.0,
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
        simulation,
        aruco_node,
        rviz
    ])
