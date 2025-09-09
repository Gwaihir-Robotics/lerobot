from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Start Mini Mapper node
    mini_mapper_node = Node(
        package='mini_mapper_nav',
        executable='mini_mapper_node',
        name='mini_mapper_node',
        output='screen'
    )
    
    # Start C1 lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('sllidar_ros2'),
            '/launch/view_sllidar_c1_launch.py'
        ])
    )
    
    # Start SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'slam_toolbox.odom_frame': 'odom',
            'slam_toolbox.map_frame': 'map',
            'slam_toolbox.base_frame': 'base_link',
            'slam_toolbox.scan_topic': '/scan',
            'slam_toolbox.resolution': 0.05,
            'slam_toolbox.max_laser_range': 12.0,
            'slam_toolbox.minimum_travel_distance': 0.2,
            'slam_toolbox.minimum_travel_heading': 0.17,  # ~10 degrees
        }]
    )
    
    # Robot state publisher
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
        parameters=[{'robot_description': open(urdf_file).read()}]
    )
    
    return LaunchDescription([
        mini_mapper_node,
        lidar_launch,
        robot_state_publisher,
        slam_node,
    ])