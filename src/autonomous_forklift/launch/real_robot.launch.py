import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')
    
    # --- HARDWARE CONFIGURATION (Based on Teacher's Instructions) ---
    
    # 1. LIDAR DRIVER (Hokuyo urg_node2)
    # Using direct Node to specify IP address found via tcpdump (192.168.0.11)
    lidar_node = Node(
        package='urg_node2',
        executable='urg_node2_node',
        output='screen',
        parameters=[{
            'ip_address': '192.168.0.11',
            'ip_port': 10940,
            'frame_id': 'laser',
            'calibrate_time': False,
            'publish_intensity': False,
            'publish_multiecho': False,
            'angle_min': -1.57,
            'angle_max': 1.57
        }]
    )

    # 2. STATIC TRANSFORM (Base -> Laser)
    # Teacher's args: 0 0 0 1 0 0 0 (x y z qx qy qz qw) -> 180 deg rotation around X
    # tf_lidar = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '1', '0', '0', '0', 'base_link', 'laser']
    # )
    
    # 3. ROBOT BASE DRIVER (Kobuki)
    # kobuki_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('kobuki'), 'launch', 'kobuki.launch.py')
    #     )
    # )
    
    # 4. ARUCO DETECTION (Real Camera)
    # aruco_node = Node(
    #     package='aruco_ros',
    #     executable='marker_publisher',
    #     parameters=[{
    #         'image_is_rectified': True,
    #         'marker_size': 0.1,
    #         'reference_frame': 'base_link',
    #         'camera_frame': 'camera_link'
    #     }],
    #     remappings=[
    #         ('/camera_info', '/camera/camera_info'),
    #         ('/image', '/camera/image_raw')
    #     ]
    # )

    # 5. LOCALIZATION (Required for Navigation)
    # Option A: SLAM Toolbox (Localization Mode) - Recommended
    # localization_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    #     ),
    #     launch_arguments={'slam_params_file': '/path/to/your/mapper_params_online_async.yaml'}.items()
    # )
    
    # Option B: AMCL (Map Server + AMCL)
    pkg_path = get_package_share_directory('autonomous_forklift')
    map_file = os.path.join(pkg_path, 'maps', 'mundo_map.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]
    )

    # --- CORE LOGIC NODES (The "Brain") ---
    
    # Navigation Node (Waypoint Follower)
    navigation_node = Node(
        package='autonomous_forklift',
        executable='waypoint_follower.py',
        name='navigation_node',
        output='screen',
        parameters=[{
            'linear_speed': 0.2,  # Slower for real robot safety
            'angular_speed': 0.5,
            'distance_tolerance': 0.15,
            'nav_enabled': True
        }]
    )

    # Interface Node (GUI)
    interface_node = Node(
        package='autonomous_forklift',
        executable='interface_node.py',
        name='interface_node',
        output='screen'
    )

    # Docking Node
    docking_node = Node(
        package='smart_warehouse',
        executable='docking_aruco',
        name='aruco_docking',
        output='screen'
    )

    # Graph Visualizer (for RViz)
    graph_viz_node = Node(
        package='autonomous_forklift',
        executable='graph_visualizer.py',
        name='graph_visualizer',
        output='screen'
    )

    # RViz
    rviz_config_file = os.path.join(pkg_autonomous_forklift, 'rviz', 'rviz_copiaDEFINITIVA.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        # --- RUN ON ROBOT (See start_robot.sh) ---
        # lidar_node,
        # tf_lidar,
        # kobuki_launch,
        # aruco_node,
        
        # --- RUN ON PC (Brain & UI) ---
        map_server_node,
        amcl_node,
        lifecycle_manager,
        navigation_node,
        interface_node,
        docking_node,
        graph_viz_node,
        rviz_node
    ])

