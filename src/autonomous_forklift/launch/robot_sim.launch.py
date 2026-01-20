#!/usr/bin/env python3
"""
Launch file for Forklift robot simulation in Gazebo Harmonic.

This launch file:
1. Starts Robot State Publisher with processed URDF
2. Launches Gazebo Harmonic with empty world
3. Spawns the forklift robot at origin
4. Starts the ROS-GZ bridge for topic communication
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to URDF/Xacro file
    xacro_file = os.path.join(pkg_autonomous_forklift, 'urdf', 'forklift.urdf.xacro')

    # Process Xacro to URDF
    robot_description = Command(['xacro ', xacro_file])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty.sdf')

    # ==================== 1. ROBOT STATE PUBLISHER ====================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # ==================== 2. GAZEBO HARMONIC ====================
    # Ensure GZ_SIM_RESOURCE_PATH includes our package share for models
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(pkg_autonomous_forklift, 'models')
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join(pkg_autonomous_forklift, 'models')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s ', world],  # -s = server only (headless)
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ==================== 3. SPAWN ROBOT ====================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'forklift',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
            '-Y', '0.0'
        ],
        output='screen'
    )



    # ==================== 4. ROS-GZ BRIDGE ====================
    # Bridge configuration for topic translation between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Velocity command: ROS -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry: Gazebo -> ROS
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # Lidar scan: Gazebo -> ROS
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # TF: Gazebo -> ROS
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # Clock: Gazebo -> ROS (for use_sim_time)
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            # Joint states: Gazebo -> ROS
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            # Camera image: Gazebo -> ROS
            '/camera_sensor/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            # Camera info: Gazebo -> ROS
            '/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Gazebo world file to load'
        ),

        # Nodes (in order)
        robot_state_publisher_node,
        gazebo,
        spawn_entity,

        bridge,
    ])
