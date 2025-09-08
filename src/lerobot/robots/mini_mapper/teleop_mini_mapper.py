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
import time
from lerobot.robots.mini_mapper import MiniMapperClient, MiniMapperClientConfig


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
    
    print("\n🎮 Mini Mapper Teleoperation Controls:")
    print("  W - Move forward")
    print("  S - Move backward") 
    print("  A - Rotate left (counter-clockwise)")
    print("  D - Rotate right (clockwise)")
    print("  R - Increase speed")
    print("  F - Decrease speed") 
    print("  Q - Quit")
    print("\nEnter commands and press ENTER...")
    
    # Speed levels
    speed_levels = [0.1, 0.2, 0.3, 0.5]  # m/s
    rotation_levels = [30, 45, 60, 90]   # deg/s
    current_speed_idx = 1  # Start at 0.2 m/s
    
    try:
        while True:
            cmd = input(f"\n[Speed: {speed_levels[current_speed_idx]:.1f}m/s] Command: ").lower().strip()
            
            if cmd == 'q':
                print("Quitting...")
                break
            elif cmd == 'w':
                action = {'x.vel': speed_levels[current_speed_idx], 'y.vel': 0.0, 'theta.vel': 0.0}
                print("↑ Moving forward")
            elif cmd == 's': 
                action = {'x.vel': -speed_levels[current_speed_idx], 'y.vel': 0.0, 'theta.vel': 0.0}
                print("↓ Moving backward")
            elif cmd == 'a':
                action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': rotation_levels[current_speed_idx]}
                print("↺ Rotating left")
            elif cmd == 'd':
                action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': -rotation_levels[current_speed_idx]}
                print("↻ Rotating right")
            elif cmd == 'r':
                current_speed_idx = min(current_speed_idx + 1, len(speed_levels) - 1)
                action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
                print(f"⚡ Speed increased to {speed_levels[current_speed_idx]:.1f}m/s")
            elif cmd == 'f':
                current_speed_idx = max(current_speed_idx - 1, 0)
                action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
                print(f"🐌 Speed decreased to {speed_levels[current_speed_idx]:.1f}m/s")
            else:
                action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
                if cmd:
                    print("❓ Unknown command. Use W/A/S/D/R/F/Q")
                
            # Send command to robot
            try:
                robot.send_action(action)
                
                # Get robot feedback
                obs = robot.get_observation()
                if any(abs(obs[k]) > 0.001 for k in ['x.vel', 'y.vel', 'theta.vel']):
                    print(f"🤖 Robot: x={obs['x.vel']:.2f}m/s, θ={obs['theta.vel']:.1f}°/s")
                    
            except Exception as e:
                print(f"❌ Command failed: {e}")
                break
                
            time.sleep(0.1)  # Small delay for command processing
    
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