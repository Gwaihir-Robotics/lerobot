#!/usr/bin/env python3

"""
Mini Mapper Auto-Calibration using Lidar
Automatically calibrates odometry by driving towards walls and measuring with lidar.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import time
import math

class AutoCalibrator(Node):
    def __init__(self):
        super().__init__('auto_calibrator')
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Data storage
        self.current_scan = None
        self.current_odom = None
        self.calibration_speed = 0.1  # m/s - slow for accuracy
        
        self.get_logger().info('Auto-calibrator initialized')
    
    def scan_callback(self, msg):
        self.current_scan = msg
    
    def odom_callback(self, msg):
        self.current_odom = msg
    
    def get_front_distance(self, angle_range=30):
        """Get distance to object directly in front (within angle_range degrees)"""
        if self.current_scan is None:
            return None
            
        scan = self.current_scan
        
        # Convert angle range to indices
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        
        # Calculate indices for front-facing region (±angle_range degrees)
        front_angle_rad = math.radians(angle_range)
        total_points = len(scan.ranges)
        
        # Find indices around 0° (front)
        center_idx = int((0.0 - angle_min) / angle_increment)
        range_indices = int(front_angle_rad / angle_increment)
        
        start_idx = max(0, center_idx - range_indices)
        end_idx = min(total_points, center_idx + range_indices)
        
        # Get valid ranges in front region
        front_ranges = []
        for i in range(start_idx, end_idx):
            if i < len(scan.ranges):
                r = scan.ranges[i]
                if scan.range_min <= r <= scan.range_max and not math.isinf(r):
                    front_ranges.append(r)
        
        if not front_ranges:
            return None
            
        # Return median distance (more robust than mean)
        return np.median(front_ranges)
    
    def get_current_position(self):
        """Get current odometry position"""
        if self.current_odom is None:
            return None
        return (self.current_odom.pose.pose.position.x, 
                self.current_odom.pose.pose.position.y)
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_pub.publish(twist)
    
    def drive_forward(self, duration):
        """Drive forward for specified duration"""
        twist = Twist()
        twist.linear.x = self.calibration_speed
        
        start_time = time.time()
        rate = self.create_rate(20)  # 20 Hz
        
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        self.stop_robot()
    
    def wait_for_data(self, timeout=5.0):
        """Wait for scan and odometry data"""
        start_time = time.time()
        rate = self.create_rate(10)
        
        while (time.time() - start_time) < timeout:
            if self.current_scan is not None and self.current_odom is not None:
                return True
            rate.sleep()
        
        return False
    
    def calibrate_linear(self, target_distance=1.0):
        """Calibrate linear movement by driving towards a wall"""
        self.get_logger().info(f"🎯 Starting linear calibration...")
        self.get_logger().info("Position robot facing a wall at least 2 meters away")
        
        # Wait for user confirmation
        input("Press Enter when robot is positioned facing a wall...")
        
        if not self.wait_for_data():
            self.get_logger().error("❌ No scan/odometry data received")
            return None
        
        # Get initial measurements
        initial_distance = self.get_front_distance()
        initial_position = self.get_current_position()
        
        if initial_distance is None:
            self.get_logger().error("❌ Cannot detect wall in front")
            return None
        
        self.get_logger().info(f"📏 Initial distance to wall: {initial_distance:.3f}m")
        self.get_logger().info(f"📍 Initial odometry position: ({initial_position[0]:.3f}, {initial_position[1]:.3f})")
        
        # Calculate drive time for target distance
        drive_time = target_distance / self.calibration_speed
        
        self.get_logger().info(f"🚗 Driving forward {target_distance}m (commanded)...")
        
        # Drive forward
        self.drive_forward(drive_time)
        
        # Wait a moment for data to stabilize
        time.sleep(0.5)
        
        # Get final measurements
        final_distance = self.get_front_distance()
        final_position = self.get_current_position()
        
        if final_distance is None:
            self.get_logger().error("❌ Cannot detect wall after movement")
            return None
        
        # Calculate actual movement using lidar
        lidar_movement = initial_distance - final_distance
        
        # Calculate odometry movement
        odom_movement = math.sqrt((final_position[0] - initial_position[0])**2 + 
                                 (final_position[1] - initial_position[1])**2)
        
        self.get_logger().info(f"📏 Final distance to wall: {final_distance:.3f}m")
        self.get_logger().info(f"📍 Final odometry position: ({final_position[0]:.3f}, {final_position[1]:.3f})")
        self.get_logger().info(f"📐 Lidar measured movement: {lidar_movement:.3f}m")
        self.get_logger().info(f"📐 Odometry measured movement: {odom_movement:.3f}m")
        self.get_logger().info(f"📐 Commanded movement: {target_distance:.3f}m")
        
        # Calculate calibration factors
        if odom_movement > 0:
            # Scale factor to correct odometry to match lidar
            odom_scale = lidar_movement / odom_movement
            self.get_logger().info(f"🔧 Recommended odometry scale factor: {odom_scale:.3f}")
            self.get_logger().info(f"🔧 Update bridge: self.linear_scale = {odom_scale:.3f}")
            
            # Accuracy metrics
            commanded_error = abs(target_distance - lidar_movement)
            odom_error = abs(target_distance - odom_movement)
            
            self.get_logger().info(f"📊 Commanded vs Actual error: {commanded_error:.3f}m ({commanded_error/target_distance*100:.1f}%)")
            self.get_logger().info(f"📊 Odometry error (before correction): {odom_error:.3f}m ({odom_error/target_distance*100:.1f}%)")
            
            return odom_scale
        else:
            self.get_logger().error("❌ No odometry movement detected")
            return None
    
    def calibrate_angular(self):
        """Calibrate angular movement using wall detection"""
        self.get_logger().info("🔄 Angular calibration not yet implemented")
        self.get_logger().info("📝 Use manual rotation calibration for now")
        return None

def main():
    rclpy.init()
    calibrator = AutoCalibrator()
    
    print("\n🎯 Mini Mapper Auto-Calibration")
    print("================================")
    print("This tool automatically calibrates odometry using lidar measurements")
    print("\n📋 Requirements:")
    print("  1. Robot should be positioned facing a flat wall")
    print("  2. At least 2 meters clearance to wall")
    print("  3. SLAM system should be running")
    
    try:
        result = calibrator.calibrate_linear(target_distance=1.0)
        
        if result:
            print(f"\n✅ Calibration complete!")
            print(f"🔧 Update your bridge with: linear_scale = {result:.3f}")
        else:
            print(f"\n❌ Calibration failed")
            
    except KeyboardInterrupt:
        print(f"\n🛑 Calibration cancelled")
    finally:
        calibrator.stop_robot()
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()