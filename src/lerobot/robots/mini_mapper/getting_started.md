# Mini Mapper Robot - Getting Started Guide

This guide walks you through setting up your Mini Mapper differential drive robot, from servo configuration to remote control from your laptop.

## Prerequisites

- Mini Mapper hardware assembled with:
  - 2x Feetech STS3215 servos (left wheel ID 7, right wheel ID 8)
  - Raspberry Pi 4B/5 with camera module
  - MotorBus adapter for servo communication
  - 12V power supply with USB-C PD trigger
- Laptop with Ubuntu/macOS for remote control
- Network connectivity (WiFi/Ethernet for both devices)

## Part 1: Raspberry Pi Setup

### 1.1 Install Ubuntu 22.04 LTS on Raspberry Pi

1. Flash Ubuntu 22.04 LTS Server to SD card using Raspberry Pi Imager
2. Enable SSH and configure WiFi during imaging process
3. Boot the Pi and SSH in: `ssh ubuntu@<pi-ip-address>`

### 1.2 Install ROS2 Kilted

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Kilted
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-kilted-desktop -y

# Setup ROS2 environment
echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.3 Install LeRobot

```bash
# Install Python dependencies and tools
sudo apt install python3-pip python3-venv python3-full python3-opencv libopencv-dev python-is-python3 -y

# Clone LeRobot repository
cd ~
git clone git@github.com:Gwaihir-Robotics/lerobot.git
cd lerobot

# Create and activate virtual environment
python3 -m venv ~/lerobot_venv
source ~/lerobot_venv/bin/activate

# Upgrade pip in virtual environment
pip install --upgrade pip

# Install any missing dependencies that ROS2 packages might need
pip install typeguard

# Install in development mode with Feetech servo support
pip install -e ".[feetech]"

# Install additional dependencies for Mini Mapper
pip install pyserial zmq opencv-python numpy

# Add virtual environment activation to bashrc for convenience
echo "source ~/lerobot_venv/bin/activate" >> ~/.bashrc
echo "cd ~/lerobot" >> ~/.bashrc
```

### 1.4 Configure Camera

```bash
# Install camera utilities
sudo apt install v4l-utils fswebcam -y

# Check if any USB cameras are connected
lsusb | grep -i camera

# Check for any video devices
ls /dev/video*

# If you have a Raspberry Pi camera module, check if it's detected
# (This works on Ubuntu unlike vcgencmd)
dmesg | grep -i camera

# Test camera with v4l2 (if any video devices found)
v4l2-ctl --list-devices

# Find the correct camera device (Pi camera system uses higher numbered devices)
# Look for a device that supports capture
for device in /dev/video*; do
    if v4l2-ctl -d $device --list-formats-ext 2>/dev/null | grep -q "YUYV\|MJPG\|RGB"; then
        echo "Found camera device: $device"
        CAMERA_DEVICE=$device
        break
    fi
done

# Test camera capture with OpenCV (more compatible with Pi camera system)
if [ ! -z "$CAMERA_DEVICE" ]; then
    echo "Found camera device: $CAMERA_DEVICE"
    echo "Testing with OpenCV..."
    
    python -c "
import cv2
import sys

# Try to open the camera
device_path = '$CAMERA_DEVICE'
print(f'Attempting to open camera: {device_path}')

# OpenCV can handle device paths directly
cap = cv2.VideoCapture(device_path)

if not cap.isOpened():
    print('❌ Failed to open camera')
    sys.exit(1)

# Try to read a frame
ret, frame = cap.read()
if ret:
    # Save test image
    cv2.imwrite('opencv_test.jpg', frame)
    print('✅ Camera test successful! Image saved as opencv_test.jpg')
    print(f'Frame dimensions: {frame.shape[1]}x{frame.shape[0]}')
else:
    print('❌ Failed to capture frame')
    
cap.release()
"
else
    echo "No suitable camera device found"
fi
```

### 1.5 Configure Serial Port for Servos

```bash
# Add user to dialout group for serial port access
sudo usermod -a -G dialout $USER

# If using GPIO serial pins (not USB), disable serial console
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyAMA0.service

# Check available serial devices
ls /dev/tty*

# For USB-connected MotorBus adapter, device will typically be:
# /dev/ttyACM0 or /dev/ttyUSB0

# Logout and login again for group changes to take effect
exit
# Then SSH back in
```

## Part 2: Servo Configuration

### 2.1 Connect Servos for Setup

1. Connect your MotorBus adapter to the Pi (typically via USB to `/dev/ttyACM0`)
2. Connect **only the left wheel servo** to the MotorBus adapter
3. Power on the system

### 2.2 Configure Servo IDs

```bash
cd ~/lerobot

# Ensure virtual environment is activated
source ~/lerobot_venv/bin/activate

# Run servo setup for Mini Mapper
python -m lerobot.setup_motors \
  --robot.type mini_mapper \
  --robot.id mini_mapper_01

# Follow the prompts:
# 1. Connect right wheel servo only (ID will be set to 8)
# 2. When prompted, disconnect right servo and connect left servo
# 3. Right wheel servo ID will be set to 8
```

**Important**: Only connect one servo at a time during ID setup to avoid conflicts.

### 2.3 Test Individual Servos

```bash
# Test left wheel servo (ID 7)
python -c "
from lerobot.robots.mini_mapper import MiniMapper, MiniMapperConfig
config = MiniMapperConfig()
robot = MiniMapper(config)
robot.connect()
robot.bus.write('Goal_Velocity', 'base_left_wheel', 100)  # Slow forward
import time; time.sleep(2)
robot.bus.write('Goal_Velocity', 'base_left_wheel', 0)    # Stop
robot.disconnect()
"

# Test right wheel servo (ID 8)  
python -c "
from lerobot.robots.mini_mapper import MiniMapper, MiniMapperConfig
config = MiniMapperConfig()
robot = MiniMapper(config)
robot.connect()
robot.bus.write('Goal_Velocity', 'base_right_wheel', 100)  # Slow forward
import time; time.sleep(2)
robot.bus.write('Goal_Velocity', 'base_right_wheel', 0)    # Stop
robot.disconnect()
"
```

### 2.4 Calibrate Robot

```bash
# Run calibration (with both servos connected)
python -c "
from lerobot.robots.mini_mapper import MiniMapper, MiniMapperConfig
config = MiniMapperConfig(robot_id='mini_mapper_01')
robot = MiniMapper(config)
robot.connect(calibrate=True)
robot.disconnect()
"

# Follow prompts to ensure wheels are in neutral position
# Calibration file will be saved automatically
```

## Part 3: Test Basic Movement

### 3.1 Test Differential Drive

```bash
# Test forward movement
python -c "
from lerobot.robots.mini_mapper import MiniMapper, MiniMapperConfig
config = MiniMapperConfig(robot_id='mini_mapper_01')
robot = MiniMapper(config)
robot.connect()

# Move forward slowly
action = {'x.vel': 0.1, 'y.vel': 0.0, 'theta.vel': 0.0}
robot.send_action(action)
import time; time.sleep(2)

# Stop
action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
robot.send_action(action)
robot.disconnect()
"

# Test rotation
python -c "
from lerobot.robots.mini_mapper import MiniMapper, MiniMapperConfig
config = MiniMapperConfig(robot_id='mini_mapper_01')
robot = MiniMapper(config)
robot.connect()

# Rotate left
action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 30.0}
robot.send_action(action)
import time; time.sleep(2)

# Stop
action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
robot.send_action(action)
robot.disconnect()
"
```

## Part 4: Remote Control Setup

### 4.1 Install LeRobot on Laptop

```bash
# On your laptop (Ubuntu/macOS)
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e .
pip install zmq opencv-python
```

### 4.2 Start Host on Raspberry Pi

```bash
# On Raspberry Pi
cd ~/lerobot

# Get Pi's IP address
hostname -I

# Start Mini Mapper host (runs for 30 seconds by default)
python -m lerobot.robots.mini_mapper.mini_mapper_host
```

You should see:
```
INFO:root:Configuring MiniMapper
INFO:root:Connecting MiniMapper  
INFO:root:Starting HostAgent
INFO:root:Waiting for commands...
```

### 4.3 Connect from Laptop

```bash
# On laptop, create a simple teleop script
cat > teleop_mini_mapper.py << 'EOF'
#!/usr/bin/env python3

import time
from lerobot.robots.mini_mapper import MiniMapperClient, MiniMapperClientConfig

# Replace with your Pi's IP address
PI_IP = "192.168.1.100"  # Change this!

config = MiniMapperClientConfig(remote_ip=PI_IP)
robot = MiniMapperClient(config)

print("Connecting to Mini Mapper...")
robot.connect()
print("Connected! Use WASD to control:")
print("W/S: Forward/Backward")  
print("A/D: Rotate Left/Right")
print("R/F: Speed Up/Down")
print("Q: Quit")

try:
    import keyboard
    
    while True:
        pressed_keys = []
        if keyboard.is_pressed('w'): pressed_keys.append('w')
        if keyboard.is_pressed('s'): pressed_keys.append('s') 
        if keyboard.is_pressed('a'): pressed_keys.append('a')
        if keyboard.is_pressed('d'): pressed_keys.append('d')
        if keyboard.is_pressed('r'): pressed_keys.append('r')
        if keyboard.is_pressed('f'): pressed_keys.append('f')
        if keyboard.is_pressed('q'): 
            print("Quitting...")
            break
            
        # Generate movement commands
        action = robot._from_keyboard_to_base_action(pressed_keys)
        robot.send_action(action)
        
        time.sleep(0.1)  # 10Hz control loop
        
except ImportError:
    print("Install keyboard library: pip install keyboard")
    print("Or use manual control:")
    
    while True:
        cmd = input("Enter command (w/s/a/d/q): ").lower()
        if cmd == 'q':
            break
        elif cmd == 'w':
            action = {'x.vel': 0.2, 'y.vel': 0.0, 'theta.vel': 0.0}
        elif cmd == 's': 
            action = {'x.vel': -0.2, 'y.vel': 0.0, 'theta.vel': 0.0}
        elif cmd == 'a':
            action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 30.0}
        elif cmd == 'd':
            action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': -30.0}
        else:
            action = {'x.vel': 0.0, 'y.vel': 0.0, 'theta.vel': 0.0}
            
        robot.send_action(action)
        time.sleep(0.1)

finally:
    robot.disconnect()
    print("Disconnected.")
EOF

# Make executable and install keyboard library
chmod +x teleop_mini_mapper.py
pip install keyboard

# Update the PI_IP address in the script
nano teleop_mini_mapper.py

# Run teleop
python teleop_mini_mapper.py
```

## Part 5: Verify Camera Stream

```bash
# On laptop, test camera stream
cat > test_camera.py << 'EOF'
#!/usr/bin/env python3

import cv2
import numpy as np
from lerobot.robots.mini_mapper import MiniMapperClient, MiniMapperClientConfig

PI_IP = "192.168.1.100"  # Change this!

config = MiniMapperClientConfig(remote_ip=PI_IP)
robot = MiniMapperClient(config)

print("Connecting to Mini Mapper...")
robot.connect()

try:
    while True:
        obs = robot.get_observation()
        
        # Display camera feed
        if 'front' in obs:
            cv2.imshow('Mini Mapper Camera', obs['front'])
            
        # Print velocity feedback
        print(f"Velocity - X: {obs['x.vel']:.2f} m/s, Theta: {obs['theta.vel']:.1f} deg/s")
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
except KeyboardInterrupt:
    print("Stopping...")
finally:
    robot.disconnect()
    cv2.destroyAllWindows()
EOF

python test_camera.py
```

## Troubleshooting

### Servo Issues
- **Servo not responding**: Check power supply (12V), connections, and motor IDs
- **Jittery movement**: Lower P_Coefficient in servo config, check for loose connections
- **Wrong direction**: Check motor wiring or modify direction in kinematics

### Connection Issues  
- **Host timeout**: Check Pi IP address, firewall settings, network connectivity
- **Permission denied on serial**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
- **Camera not found**: Enable camera in raspi-config, check /dev/video0 exists

### Network Issues
- **Can't connect from laptop**: Ensure both devices on same network, check Pi's IP with `hostname -I`
- **Slow response**: Check WiFi signal strength, consider wired ethernet connection

## Next Steps

Once basic movement is working:

1. **Tune Performance**: Adjust PID parameters for smoother motion
2. **Add Safety Features**: Implement emergency stops, collision avoidance  
3. **Install Lidar**: Mount and configure Slamtech C1 lidar
4. **SLAM Integration**: Set up nav2 and slam_toolbox for autonomous mapping
5. **Visualization**: Configure RViz2 for real-time map viewing

## Configuration Files

All robot-specific configurations are stored in:
- Robot config: `~/.cache/lerobot/calibration/mini_mapper_01.json` 
- Logs: Check `~/.cache/lerobot/logs/` for debugging

Your Mini Mapper is now ready for basic teleoperation! The differential drive system provides excellent maneuverability for indoor navigation and mapping tasks.