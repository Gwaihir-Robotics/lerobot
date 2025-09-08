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
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.unix_mode = True
        except (ImportError, AttributeError, termios.error):
            self.unix_mode = False
            print("⚠️  Non-blocking input not available - using simplified mode")
        
    def __enter__(self):
        return self
        
    def __exit__(self, type, value, traceback):
        if self.unix_mode:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
    def get_char(self):
        """Get a single character without blocking"""
        if not self.unix_mode:
            return None
            
        try:
            import select
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
        except (ImportError, OSError):
            pass
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
    
    print("\n🎮 Mini Mapper Driving Mode Control:")
    print("  W - Enter FORWARD driving mode")
    print("  S - Enter REVERSE driving mode")
    print("  A - Enter LEFT TURN mode") 
    print("  D - Enter RIGHT TURN mode")
    print("  + - Increase speed in current mode")
    print("  - - Decrease speed in current mode")
    print("  SPACE - STOP (exit all modes)")
    print("  Q - Quit")
    print("\nOnce in a driving mode, use +/- to control speed!")
    
    # Speed levels
    speed_levels = [0.05, 0.1, 0.2, 0.3, 0.5]  # m/s
    rotation_levels = [15, 30, 45, 60, 90]      # deg/s
    current_speed_idx = 1  # Start at 0.1 m/s
    
    # Driving mode state
    current_mode = "STOP"  # STOP, FORWARD, REVERSE, LEFT_TURN, RIGHT_TURN
    last_action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
    last_status_print = 0
    
    try:
        with NonBlockingInput() as input_handler:
            if not input_handler.unix_mode:
                # Fallback to simple input mode
                print("\n🔄 Using simple command mode (press Enter after each command)")
                while True:
                    cmd = input(f"\n[Speed: {speed_levels[current_speed_idx]:.1f}m/s] Command (w/s/a/d/r/f/q): ").lower().strip()
                    
                    if cmd == 'q':
                        break
                    elif cmd == 'w':
                        action = {'x.vel': speed_levels[current_speed_idx], 'y.vel': 0.0, 'theta.vel': 0.0}
                    elif cmd == 's': 
                        action = {'x.vel': -speed_levels[current_speed_idx], 'y.vel': 0.0, 'theta.vel': 0.0}
                    elif cmd == 'a':
                        action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': rotation_levels[current_speed_idx]}
                    elif cmd == 'd':
                        action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': -rotation_levels[current_speed_idx]}
                    elif cmd == 'r':
                        current_speed_idx = min(current_speed_idx + 1, len(speed_levels) - 1)
                        print(f"⚡ Speed: {speed_levels[current_speed_idx]:.1f}m/s")
                        continue
                    elif cmd == 'f':
                        current_speed_idx = max(current_speed_idx - 1, 0)
                        print(f"🐌 Speed: {speed_levels[current_speed_idx]:.1f}m/s")
                        continue
                    else:
                        action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
                    
                    try:
                        robot.send_action(action)
                        # Send stop after brief movement
                        time.sleep(0.5)
                        robot.send_action({'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0})
                    except Exception as e:
                        print(f"❌ Command failed: {e}")
                        break
            else:
                # Driving mode system  
                print(f"\n[Mode: {current_mode}] [Speed: {speed_levels[current_speed_idx]:.2f}m/s] Ready!")
                
                while True:
                    # Read any available key presses
                    char = input_handler.get_char()
                    mode_changed = False
                    
                    if char:
                        char = char.lower()
                        if char == 'q':
                            print("\nQuitting...")
                            break
                        elif char == ' ':  # Space to stop
                            current_mode = "STOP"
                            mode_changed = True
                            print(f"\n🛑 STOPPED")
                        elif char == 'w':  # Forward mode
                            current_mode = "FORWARD"
                            mode_changed = True
                            print(f"\n🚗 FORWARD mode at {speed_levels[current_speed_idx]:.2f}m/s")
                        elif char == 's':  # Reverse mode  
                            current_mode = "REVERSE"
                            mode_changed = True
                            print(f"\n🔄 REVERSE mode at {speed_levels[current_speed_idx]:.2f}m/s")
                        elif char == 'a':  # Left turn mode
                            current_mode = "LEFT_TURN" 
                            mode_changed = True
                            print(f"\n↺ LEFT TURN mode at {rotation_levels[current_speed_idx]:.0f}°/s")
                        elif char == 'd':  # Right turn mode
                            current_mode = "RIGHT_TURN"
                            mode_changed = True
                            print(f"\n↻ RIGHT TURN mode at {rotation_levels[current_speed_idx]:.0f}°/s")
                        elif char in '+=':  # Increase speed
                            if current_mode != "STOP":
                                current_speed_idx = min(current_speed_idx + 1, len(speed_levels) - 1)
                                mode_changed = True
                                if current_mode in ["FORWARD", "REVERSE"]:
                                    print(f"\n⚡ {current_mode} speed: {speed_levels[current_speed_idx]:.2f}m/s")
                                else:
                                    print(f"\n⚡ {current_mode} speed: {rotation_levels[current_speed_idx]:.0f}°/s")
                        elif char in '-_':  # Decrease speed
                            if current_mode != "STOP":
                                current_speed_idx = max(current_speed_idx - 1, 0)
                                mode_changed = True
                                if current_mode in ["FORWARD", "REVERSE"]:
                                    print(f"\n🐌 {current_mode} speed: {speed_levels[current_speed_idx]:.2f}m/s")
                                else:
                                    print(f"\n🐌 {current_mode} speed: {rotation_levels[current_speed_idx]:.0f}°/s")
                    
                    # Calculate action based on current driving mode
                    action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
                    
                    if current_mode == "FORWARD":
                        action['x.vel'] = speed_levels[current_speed_idx]
                    elif current_mode == "REVERSE":
                        action['x.vel'] = -speed_levels[current_speed_idx]
                    elif current_mode == "LEFT_TURN":
                        action['theta.vel'] = rotation_levels[current_speed_idx]
                    elif current_mode == "RIGHT_TURN":
                        action['theta.vel'] = -rotation_levels[current_speed_idx]
                    # STOP mode keeps all velocities at 0.0
                    
                    # Send action if mode changed or periodically
                    current_time = time.time()
                    if mode_changed or action != last_action or (current_time - last_status_print > 0.1):
                        try:
                            robot.send_action(action)
                            last_action = action.copy()
                            last_status_print = current_time
                        except Exception as e:
                            print(f"\n❌ Command failed: {e}")
                            break
                    
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