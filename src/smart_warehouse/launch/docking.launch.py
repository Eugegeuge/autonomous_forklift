import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. ArUco Marker Publisher (Detection)
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

    # 2. Docking Logic Node
    docking_node = Node(
        package='smart_warehouse',
        executable='docking_aruco',
        name='aruco_docking',
        output='screen',
        parameters=[{
            'target_dist': 0.5,
            'max_v': 0.15,
            'max_w': 0.3
        }]
    )

    return LaunchDescription([
        aruco_node,
        docking_node
    ])
