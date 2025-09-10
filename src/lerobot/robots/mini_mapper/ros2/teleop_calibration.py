#!/usr/bin/env python3

import sys
import tty
import select
import termios
import threading
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Mini Mapper Calibration Teleop
---------------------------
WASD controls:
   w/s : forward/backward
   a/d : left/right
   q/e : rotate left/right
   space : stop

Calibration hotkeys:
   1 : Drive 1 meter forward (calibration test)
   2 : Rotate 360° clockwise (calibration test)
   3 : Drive 0.5 meter forward (short test)
   4 : Rotate 180° clockwise (half turn test)

   x : quit

CTRL-C to quit
"""

moveBindings = {
    'w': (1.0, 0.0, 0.0),     # forward
    's': (-1.0, 0.0, 0.0),    # backward  
    'a': (0.0, 0.0, 1.0),     # rotate left
    'd': (0.0, 0.0, -1.0),    # rotate right
    'q': (0.0, 0.0, 1.0),     # rotate left (alt)
    'e': (0.0, 0.0, -1.0),    # rotate right (alt)
}

class CalibratedTeleop(Node):
    def __init__(self):
        super().__init__('calibrated_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Calibration speeds
        self.cal_linear_speed = 0.15   # Slower for accurate calibration
        self.cal_angular_speed = 0.3   # Slower rotation for accuracy
        
        self.get_logger().info('Calibrated teleop node started')
    
    def publish_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)
    
    def calibration_move_forward(self, distance):
        """Drive forward exactly 'distance' meters"""
        self.get_logger().info(f'Calibration: Driving {distance}m forward...')
        
        # Calculate time needed: time = distance / speed
        move_time = distance / self.cal_linear_speed
        
        # Start moving
        self.publish_twist(self.cal_linear_speed, 0.0)
        
        # Wait for calculated time
        time.sleep(move_time)
        
        # Stop
        self.publish_twist(0.0, 0.0)
        self.get_logger().info(f'Calibration complete: {distance}m forward movement finished')
    
    def calibration_rotate(self, degrees):
        """Rotate exactly 'degrees' clockwise (negative for counter-clockwise)"""
        self.get_logger().info(f'Calibration: Rotating {degrees}° clockwise...')
        
        # Convert degrees to radians
        target_radians = math.radians(abs(degrees))
        angular_vel = -self.cal_angular_speed if degrees > 0 else self.cal_angular_speed  # Clockwise is negative
        
        # Calculate time needed: time = angle / angular_speed
        move_time = target_radians / abs(angular_vel)
        
        # Start rotating
        self.publish_twist(0.0, angular_vel)
        
        # Wait for calculated time  
        time.sleep(move_time)
        
        # Stop
        self.publish_twist(0.0, 0.0)
        self.get_logger().info(f'Calibration complete: {degrees}° rotation finished')

def getKey(settings):
    tty.setcraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = CalibratedTeleop()
    
    print(msg)
    
    try:
        while True:
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x, y, th = moveBindings[key]
                node.publish_twist(x * node.linear_speed, th * node.angular_speed)
                
            elif key == ' ':  # Spacebar - stop
                node.publish_twist(0.0, 0.0)
                
            elif key == '1':  # Drive 1 meter forward
                node.calibration_move_forward(1.0)
                
            elif key == '2':  # Rotate 360° clockwise
                node.calibration_rotate(360)
                
            elif key == '3':  # Drive 0.5 meter forward
                node.calibration_move_forward(0.5)
                
            elif key == '4':  # Rotate 180° clockwise
                node.calibration_rotate(180)
                
            elif key == 'x':
                print("Exiting...")
                break
                
            elif key == '\x03':  # Ctrl-C
                break
                
            else:
                # Stop on any other key
                node.publish_twist(0.0, 0.0)
                if key != '':
                    print(f"Key '{key}' not recognized")
                    
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        node.publish_twist(0.0, 0.0)  # Stop the robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()