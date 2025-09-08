#!/usr/bin/env python3

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Mini Mapper Teleoperation Script

Simple keyboard teleoperation for the Mini Mapper differential drive robot.
Connect from laptop to control robot remotely over network.
"""

import argparse
import sys
import time
import termios
import tty
from lerobot.robots.mini_mapper import MiniMapperClient, MiniMapperClientConfig


class NonBlockingInput:
    """Non-blocking keyboard input handler for real-time control"""
    def __init__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.cbreak(sys.stdin.fileno())
        
    def __enter__(self):
        return self
        
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
    def get_char(self):
        """Get a single character without blocking"""
        import select
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None


def main():
    parser = argparse.ArgumentParser(description="Mini Mapper Teleoperation")
    parser.add_argument(
        "--robot-ip", 
        type=str, 
        required=True,
        help="IP address of the Raspberry Pi running the Mini Mapper host"
    )
    parser.add_argument(
        "--cmd-port", 
        type=int, 
        default=5555,
        help="Command port (default: 5555)"
    )
    parser.add_argument(
        "--obs-port", 
        type=int, 
        default=5556,
        help="Observation port (default: 5556)"
    )
    
    args = parser.parse_args()
    
    # Create client configuration
    config = MiniMapperClientConfig(
        remote_ip=args.robot_ip,
        port_zmq_cmd=args.cmd_port,
        port_zmq_observations=args.obs_port
    )
    
    robot = MiniMapperClient(config)
    
    print(f"Connecting to Mini Mapper at {args.robot_ip}...")
    try:
        robot.connect()
        print("✅ Connected successfully!")
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        print("Make sure the Mini Mapper host is running on the Pi")
        return
    
    print("\n🎮 Mini Mapper Real-Time Teleoperation:")
    print("  W/S - Forward/Backward (hold for continuous motion)")
    print("  A/D - Rotate Left/Right (hold for continuous motion)")
    print("  R/F - Increase/Decrease speed")
    print("  Q   - Quit")
    print("  SPACE - Emergency stop")
    print("\n⚠️  Hold keys for continuous motion - no need to press Enter!")
    print("Press any key to start...")
    
    # Speed levels
    speed_levels = [0.1, 0.2, 0.3, 0.5]  # m/s
    rotation_levels = [30, 45, 60, 90]   # deg/s
    current_speed_idx = 1  # Start at 0.2 m/s
    
    # State tracking
    pressed_keys = {}  # key -> timestamp when pressed
    last_action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
    last_status_print = 0
    key_timeout = 0.5  # Keys timeout after 0.5 seconds of no re-press
    
    try:
        with NonBlockingInput() as input_handler:
            print(f"\n[Speed: {speed_levels[current_speed_idx]:.1f}m/s] Ready! Use W/A/S/D keys...")
            
            while True:
                # Read any available key presses
                char = input_handler.get_char()
                if char:
                    char = char.lower()
                    if char == 'q':
                        print("\nQuitting...")
                        break
                    elif char == ' ':  # Space for emergency stop
                        pressed_keys.clear()
                        print("\n🛑 Emergency stop!")
                    elif char == 'r':
                        current_speed_idx = min(current_speed_idx + 1, len(speed_levels) - 1)
                        print(f"\n⚡ Speed: {speed_levels[current_speed_idx]:.1f}m/s")
                    elif char == 'f':
                        current_speed_idx = max(current_speed_idx - 1, 0)
                        print(f"\n🐌 Speed: {speed_levels[current_speed_idx]:.1f}m/s")
                    elif char in 'wasd':
                        pressed_keys[char] = time.time()  # Record when key was pressed
                
                # Remove expired keys (simulate key release)
                current_time = time.time()
                expired_keys = [k for k, t in pressed_keys.items() if current_time - t > key_timeout]
                for k in expired_keys:
                    del pressed_keys[k]
                
                # Calculate current action based on pressed keys
                action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
                
                if 'w' in pressed_keys:
                    action['x.vel'] += speed_levels[current_speed_idx]
                if 's' in pressed_keys:
                    action['x.vel'] -= speed_levels[current_speed_idx]
                if 'a' in pressed_keys:
                    action['theta.vel'] += rotation_levels[current_speed_idx]
                if 'd' in pressed_keys:
                    action['theta.vel'] -= rotation_levels[current_speed_idx]
                
                # Only send action if it changed
                if action != last_action:
                    try:
                        robot.send_action(action)
                        last_action = action.copy()
                        
                        # Show movement status
                        if any(abs(action[k]) > 0.001 for k in action):
                            status = []
                            if action['x.vel'] > 0: status.append("↑FWD")
                            elif action['x.vel'] < 0: status.append("↓REV")
                            if action['theta.vel'] > 0: status.append("↺LEFT")
                            elif action['theta.vel'] < 0: status.append("↻RIGHT")
                            print(f"\r🤖 {' '.join(status) if status else 'STOPPED'} ", end='', flush=True)
                        else:
                            print(f"\r🤖 STOPPED ", end='', flush=True)
                            
                    except Exception as e:
                        print(f"\n❌ Command failed: {e}")
                        break
                
                # Periodic status update
                if current_time - last_status_print > 2.0:  # Every 2 seconds
                    try:
                        obs = robot.get_observation()
                        if any(abs(obs[k]) > 0.001 for k in ['x.vel', 'y.vel', 'theta.vel']):
                            print(f"\n📊 Actual: x={obs['x.vel']:.2f}m/s, θ={obs['theta.vel']:.1f}°/s")
                        last_status_print = current_time
                    except:
                        pass
                
                time.sleep(0.05)  # 20Hz control loop
    
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    
    finally:
        # Send stop command
        print("🛑 Stopping robot...")
        try:
            robot.send_action({'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0})
            robot.disconnect()
            print("✅ Disconnected safely")
        except:
            print("⚠️ Disconnect failed")


if __name__ == "__main__":
    main()