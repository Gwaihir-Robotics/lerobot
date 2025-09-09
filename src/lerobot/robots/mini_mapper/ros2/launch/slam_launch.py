from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os

def generate_launch_description():
    
    # Start Mini Mapper node
    mini_mapper_node = Node(
        package='mini_mapper_nav',
        executable='mini_mapper_node',
        name='mini_mapper_node',
        output='screen'
    )
    
    # Start C1 lidar (using same config as working sllidar launch)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0', 
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    )
    
    # Start SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.17,  # ~10 degrees
        }]
    )
    
    # Robot state publisher with xacro processing
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
            )
        }]
    )
    
    return LaunchDescription([
        # mini_mapper_node,  # Comment out until we fix the lerobot import issue
        lidar_node,
        robot_state_publisher,
        slam_node,
    ])