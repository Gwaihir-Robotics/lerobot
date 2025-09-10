#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import json
import zmq
import time

class MiniMapperBridge(Node):
    def __init__(self):
        super().__init__('mini_mapper_bridge')
        
        # ZMQ connection to Mini Mapper host
        self.zmq_context = zmq.Context()
        self.cmd_socket = self.zmq_context.socket(zmq.PUSH)
        self.cmd_socket.connect("tcp://localhost:5555")
        
        self.obs_socket = self.zmq_context.socket(zmq.PULL)
        self.obs_socket.connect("tcp://localhost:5556")
        self.obs_socket.setsockopt(zmq.RCVTIMEO, 10)  # 10ms timeout - don't block the timer
        
        # ROS2 publishers/subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry state - always start at origin for mapping
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Odometry calibration factors (adjust based on real-world testing)
        self.linear_scale = 0.66  # Calibrated: 1.0m actual / 1.53m reported = 0.66
        self.angular_scale = 1.0  # Test angular next
        
        self.get_logger().info('Mini Mapper bridge initialized - odometry starts at origin (0,0,0)')
        
        # Timer for publishing odometry
        self.create_timer(0.05, self.publish_odometry)  # 20Hz
        
        self.get_logger().info('Mini Mapper bridge started')
    
    def cmd_vel_callback(self, msg):
        """Convert ROS2 Twist to Mini Mapper ZMQ command"""
        action = {
            'x.vel': float(msg.linear.x),
            'y.vel': 0.0,
            'theta.vel': float(math.degrees(msg.angular.z))
        }
        try:
            self.cmd_socket.send_string(json.dumps(action), zmq.NOBLOCK)
        except zmq.Again:
            self.get_logger().warn('Failed to send command, Mini Mapper host may be busy')
    
    def publish_odometry(self):
        """Get observation from Mini Mapper host and publish odometry"""
        obs = None
        msg_count = 0
        
        # Drain the message queue to get the latest observation (non-blocking)
        try:
            while msg_count < 5:  # Limit drain attempts
                try:
                    msg = self.obs_socket.recv_string(zmq.NOBLOCK)
                    obs = json.loads(msg)
                    msg_count += 1
                except zmq.Again:
                    break  # No more messages
        except Exception as e:
            self.get_logger().warn(f'ZMQ receive error: {e}')
        
        if obs is None:
            return  # No data received
            
        current_time = self.get_clock().now()
        
        # Update odometry based on velocities
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0 and dt < 1.0:  # Sanity check - reject huge dt values
            dx = obs.get('x.vel', 0.0) * dt * self.linear_scale
            dtheta = math.radians(obs.get('theta.vel', 0.0)) * dt * self.angular_scale
            
            self.x += dx * math.cos(self.theta)
            self.y += dx * math.sin(self.theta)
            self.theta += dtheta
            
            # Debug logging (remove later)
            if msg_count > 1:
                self.get_logger().info(f'Drained {msg_count} messages, dt={dt:.3f}, dx={dx:.3f}')
        
        # Always publish transform and odometry (even if dt was bad)
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
        odom.twist.twist.linear.x = obs.get('x.vel', 0.0)
        odom.twist.twist.angular.z = math.radians(obs.get('theta.vel', 0.0))
        self.odom_pub.publish(odom)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = MiniMapperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.zmq_context.term()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()