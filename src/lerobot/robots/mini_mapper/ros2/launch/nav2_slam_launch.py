#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory('mini_mapper_nav'), 
                                   'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the SLAM toolbox')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Start Mini Mapper lidar
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',  # Back to working port
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    )
    
    # Static transform from base_link to laser
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser']
    )
    
    # Robot state publisher with Mini Mapper URDF
    urdf_file = os.path.join(
        get_package_share_directory('mini_mapper_nav'),
        'urdf',
        'mini_mapper.urdf.xacro'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Mini Mapper bridge (provides odometry)
    mini_mapper_bridge = Node(
        package='mini_mapper_nav',
        executable='mini_mapper_bridge',
        name='mini_mapper_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Nav2 SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_slam_params_file_cmd,
        declare_use_sim_time_cmd,
        
        # Start immediately - basic transforms and hardware
        static_transform_publisher,
        robot_state_publisher,
        lidar_node,  # Start lidar immediately, no timer
        
        # Start bridge after lidar has time to connect (2 second delay)
        TimerAction(
            period=2.0,
            actions=[mini_mapper_bridge]
        ),
        
        # Start SLAM after bridge is ready (3 second delay)  
        TimerAction(
            period=3.0,
            actions=[slam_launch]
        ),
    ])