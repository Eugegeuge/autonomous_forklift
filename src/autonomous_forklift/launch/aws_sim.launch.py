#!/usr/bin/env python3
"""
Launch file for AWS Warehouse simulation with Map Builder support.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_autonomous_forklift = get_package_share_directory('autonomous_forklift')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_aws_warehouse = get_package_share_directory('aws_robomaker_small_warehouse_world')

    # Set Gazebo Resource Path to include AWS models
    # We need to point to the models directory in the share folder
    # Usually: install/aws_robomaker_small_warehouse_world/share/aws_robomaker_small_warehouse_world/models
    aws_models_path = os.path.join(pkg_aws_warehouse, 'models')
    
    # Update GZ_SIM_RESOURCE_PATH
    # We append to the existing path if any
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            ':',
            aws_models_path
        ]
    )

    # Path to URDF/Xacro file
    xacro_file = os.path.join(pkg_autonomous_forklift, 'urdf', 'forklift.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # World file
    # In ros2 branch, it is nested: worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
    world_file = os.path.join(pkg_aws_warehouse, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')

    # ==================== NODES ====================
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'forklift',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '2.0',
            '-z', '0.3',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        gz_resource_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
    ])
