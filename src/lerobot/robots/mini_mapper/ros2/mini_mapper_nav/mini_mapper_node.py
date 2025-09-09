#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from lerobot.robots.mini_mapper import MiniMapper, MiniMapperConfig

class MiniMapperNode(Node):
    def __init__(self):
        super().__init__('mini_mapper_node')
        
        # Initialize Mini Mapper
        config = MiniMapperConfig(id='mini_mapper_01')
        self.robot = MiniMapper(config)
        self.robot.connect()
        
        # ROS2 publishers/subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Timer for publishing odometry
        self.create_timer(0.05, self.publish_odometry)  # 20Hz
        
        self.get_logger().info('Mini Mapper node started')
    
    def cmd_vel_callback(self, msg):
        """Convert ROS2 Twist to Mini Mapper action"""
        action = {
            'x.vel': msg.linear.x,
            'y.vel': 0.0,  # Differential drive doesn't support lateral movement
            'theta.vel': math.degrees(msg.angular.z)  # Convert rad/s to deg/s
        }
        self.robot.send_action(action)
    
    def publish_odometry(self):
        """Publish odometry and TF"""
        current_time = self.get_clock().now()
        
        # Get robot observation
        obs = self.robot.get_observation()
        
        # Update odometry based on wheel velocities
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0:
            dx = obs['x.vel'] * dt
            dtheta = math.radians(obs['theta.vel']) * dt
            
            self.x += dx * math.cos(self.theta)
            self.y += dx * math.sin(self.theta)
            self.theta += dtheta
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # Convert to quaternion
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = obs['x.vel']
        odom.twist.twist.angular.z = math.radians(obs['theta.vel'])
        self.odom_pub.publish(odom)
        
        # Publish joint states (for visualization)
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [0.0, 0.0]  # Continuous joints
        joint_state.velocity = [obs['x.vel']/0.03175, obs['x.vel']/0.03175]
        self.joint_pub.publish(joint_state)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = MiniMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()