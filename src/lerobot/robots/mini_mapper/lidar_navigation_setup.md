# Slamtech C1 Lidar + Autonomous Navigation Setup

Complete guide to set up the Slamtech C1 lidar for autonomous mapping and navigation with the Mini Mapper robot.

## Goal
Enable the Mini Mapper to:
1. **Create maps** of indoor spaces using SLAM
2. **Save maps** for future missions  
3. **Navigate autonomously** using saved maps
4. **Avoid obstacles** dynamically
5. **Return to start point** after mapping

## Hardware Requirements

- ✅ **Slamtech C1 2D Lidar** - 360° scanning, 12m range
- ✅ **Mini Mapper robot** - Differential drive with corrected wheel kinematics
- ✅ **Raspberry Pi 4B/5** - Running Ubuntu 22.04 + ROS2 Kilted
- ✅ **USB connection** - C1 connects via USB to Pi

## Quick Setup (Automated)

The Mini Mapper includes an automated setup script that handles everything:

```bash
# On Raspberry Pi - run from anywhere
~/lerobot/src/lerobot/robots/mini_mapper/ros2/ros2_env_setup.sh
```

This script automatically:
- ✅ Installs all ROS2 navigation packages (nav2, slam_toolbox, etc.)
- ✅ Creates ROS2 workspace at `~/nav_ws`
- ✅ Clones and builds Slamtech C1 driver
- ✅ Configures USB permissions for C1 lidar
- ✅ Deploys Mini Mapper navigation package with all configs
- ✅ Tests the complete installation
- ✅ Provides clear next steps

After the script completes, you'll have a fully configured system ready for mapping!

## Manual Setup (Advanced)

If you prefer manual control or need to customize the installation:

## Part 1: Hardware Setup

### 1.1 Connect Slamtech C1 to Raspberry Pi

1. **USB Connection**: Connect C1 to Pi via USB cable
2. **Power**: C1 powers from USB (5V, ~400mA)
3. **Mounting**: Mount C1 on top of Mini Mapper, centered
4. **Height**: Mount ~20cm above ground for best scanning

### 1.2 Test C1 Connection

```bash
# Check if C1 is detected
lsusb | grep -i "10c4:ea60\|Silicon Labs"

# Should show Silicon Labs CP210x device
```

Use the automated setup script above, or follow these manual steps:

```bash
# Install ROS2 navigation packages
sudo apt update
sudo apt install -y ros-kilted-navigation2 ros-kilted-nav2-bringup ros-kilted-slam-toolbox

# Create workspace and clone C1 driver
mkdir -p ~/nav_ws/src
cd ~/nav_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git

# Deploy Mini Mapper package
~/lerobot/src/lerobot/robots/mini_mapper/ros2/deploy_ros2_package.sh

# Build workspace
cd ~/nav_ws && colcon build
source install/setup.bash
```

## Usage Instructions

### Step 1: Start Mini Mapper Host

On the Raspberry Pi, start the Mini Mapper host:
```bash
# On Raspberry Pi
cd ~/lerobot
python -m lerobot.robots.mini_mapper.mini_mapper_host --duration 0
```

### Step 2: Launch SLAM Mapping

In a new terminal:
```bash
cd ~/nav_ws && source install/setup.bash
ros2 launch mini_mapper_nav slam_launch.py
```

### Step 3: Control Robot to Create Map

In another terminal, drive the robot around to create a map:
```bash
# Option 1: ROS2 teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Option 2: Mini Mapper teleop (from laptop)
python -m lerobot.robots.mini_mapper.teleop_mini_mapper --robot-ip <PI_IP>
```

### Step 4: Save Map

When satisfied with your map:
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Step 5: Autonomous Navigation

Use the saved map for autonomous navigation:
```bash
ros2 launch mini_mapper_nav nav_launch.py map:=~/maps/my_map.yaml

# In RViz2, set navigation goals by clicking "2D Goal Pose"
ros2 run rviz2 rviz2
```

## What You Get

- ✅ **Real-time SLAM** mapping while driving
- ✅ **Persistent maps** saved to disk
- ✅ **Autonomous navigation** with obstacle avoidance  
- ✅ **RViz2 visualization** of maps and robot pose
- ✅ **Integration** with your existing Mini Mapper teleop

The setup provides a complete autonomous navigation stack tuned specifically for the Mini Mapper's differential drive kinematics!