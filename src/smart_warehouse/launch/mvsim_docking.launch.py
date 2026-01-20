import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_smart_warehouse = get_package_share_directory('smart_warehouse')
    pkg_mvsim = get_package_share_directory('mvsim')

    # World file
    world_file = os.path.join(pkg_smart_warehouse, 'worlds', 'docking_world.xml')

    # 1. Mvsim Node
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim_node',
        output='screen',
        parameters=[{'world_file': world_file}]
    )

    # 2. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', os.path.join(pkg_smart_warehouse, 'rviz', 'docking.rviz')] # Optional if we had one
    )

    # 3. ArUco Marker Publisher
    aruco_node = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.15,
            'reference_frame': 'base_link',
            'camera_frame': 'camera', # Mvsim usually publishes frames like 'camera' attached to robot
            'marker_frame': 'aruco_detectado',
            'marker_id': 0, # Using ID 0
        }],
        remappings=[
            ('/camera_info', '/robot1/camera/camera_info'),
            ('/image', '/robot1/camera/image'),
        ],
        output='screen'
    )

    # 4. Docking Logic
    docking_node = Node(
        package='smart_warehouse',
        executable='docking_aruco',
        name='aruco_docking',
        output='screen',
        parameters=[{
            'target_dist': 0.5,
            'max_v': 0.15,
            'max_w': 0.3
        }],
        remappings=[
            ('/cmd_vel', '/robot1/cmd_vel') # Remap to Mvsim robot topic
        ]
    )

    return LaunchDescription([
        mvsim_node,
        rviz_node,
        aruco_node,
        docking_node
    ])
