#!/bin/bash

# Mini Mapper ROS2 Environment Setup Script
# 
# This script automates the complete ROS2 navigation stack setup for Mini Mapper:
# - Installs ROS2 navigation packages (nav2, slam_toolbox, etc.)
# - Clones and builds Slamtech C1 lidar driver
# - Configures USB permissions for C1 lidar
# - Deploys Mini Mapper navigation package
# - Tests the complete system
#
# Can be run from anywhere:
#   ~/lerobot/src/lerobot/robots/mini_mapper/ros2/ros2_env_setup.sh
#   cd ~/lerobot/src/lerobot/robots/mini_mapper/ros2 && ./ros2_env_setup.sh

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Get script directory (works regardless of how script is called)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROOT="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"

# Configuration
NAV_WS="$HOME/nav_ws"
ROS_DISTRO="kilted"

log_info "🤖 Mini Mapper ROS2 Environment Setup"
log_info "====================================="
log_info "Script location: $SCRIPT_DIR"
log_info "LeRobot root: $LEROBOT_ROOT"
log_info "Navigation workspace: $NAV_WS"
log_info "ROS2 distribution: $ROS_DISTRO"
echo ""

# Check if running on Raspberry Pi
log_info "📋 Checking system requirements..."
if ! command -v lsb_release &> /dev/null; then
    log_error "lsb_release not found. Please ensure you're running Ubuntu 22.04"
    exit 1
fi

UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" != "22.04" ]; then
    log_warning "Expected Ubuntu 22.04, found $UBUNTU_VERSION. Proceeding anyway..."
fi

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    log_error "ROS2 not found. Please install ROS2 $ROS_DISTRO first:"
    log_error "  sudo apt install ros-$ROS_DISTRO-desktop"
    exit 1
fi

log_success "System requirements check passed"

# Function to check if package is installed
check_ros_package() {
    local package=$1
    if dpkg -l | grep -q "^ii  $package "; then
        return 0
    else
        return 1
    fi
}

# Install ROS2 Navigation Packages
log_info "📦 Installing ROS2 navigation packages..."
NAV_PACKAGES=(
    "ros-$ROS_DISTRO-navigation2"
    "ros-$ROS_DISTRO-nav2-bringup"
    "ros-$ROS_DISTRO-slam-toolbox"
    "ros-$ROS_DISTRO-robot-localization"
    "ros-$ROS_DISTRO-joint-state-publisher"
    "ros-$ROS_DISTRO-robot-state-publisher"
    "ros-$ROS_DISTRO-xacro"
    "ros-$ROS_DISTRO-tf2-ros"
    "ros-$ROS_DISTRO-tf2-geometry-msgs"
    "ros-$ROS_DISTRO-cartographer"
    "ros-$ROS_DISTRO-cartographer-ros"
    "ros-$ROS_DISTRO-teleop-twist-keyboard"
)

MISSING_PACKAGES=()
for package in "${NAV_PACKAGES[@]}"; do
    if ! check_ros_package "$package"; then
        MISSING_PACKAGES+=("$package")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    log_info "Installing missing packages: ${MISSING_PACKAGES[*]}"
    sudo apt update
    sudo apt install -y "${MISSING_PACKAGES[@]}"
    log_success "Navigation packages installed"
else
    log_success "All navigation packages already installed"
fi

# Install Python dependencies for ROS2 build system
log_info "🐍 Installing Python dependencies for ROS2 build..."
# Install system-wide to avoid virtual environment conflicts
sudo apt install -y python3-catkin-pkg python3-empy python3-lark
log_success "Python dependencies installed"

# Create ROS2 workspace
log_info "🏗️  Creating ROS2 navigation workspace..."
if [ ! -d "$NAV_WS" ]; then
    mkdir -p "$NAV_WS/src"
    log_success "Created navigation workspace: $NAV_WS"
else
    log_success "Navigation workspace already exists: $NAV_WS"
fi

# Clone Slamtech C1 driver
log_info "📡 Setting up Slamtech C1 lidar driver..."
SLLIDAR_PATH="$NAV_WS/src/sllidar_ros2"
if [ ! -d "$SLLIDAR_PATH" ]; then
    log_info "Cloning sllidar_ros2 repository..."
    cd "$NAV_WS/src"
    git clone https://github.com/Slamtec/sllidar_ros2.git
    log_success "Cloned sllidar_ros2 driver"
else
    log_info "Updating existing sllidar_ros2 repository..."
    cd "$SLLIDAR_PATH"
    git pull
    log_success "Updated sllidar_ros2 driver"
fi

# Build sllidar_ros2 (force system Python to avoid virtual environment conflicts)
log_info "🔨 Building sllidar_ros2 driver..."
cd "$NAV_WS"

# Handle virtual environment conflicts by completely isolating the build environment
if [ -n "$VIRTUAL_ENV" ]; then
    log_info "Virtual environment detected: $VIRTUAL_ENV"
    log_info "Running build in isolated shell to avoid Python conflicts..."
    
    # Create a clean build script that runs outside the virtual environment
    cat > /tmp/build_sllidar.sh << 'EOF'
#!/bin/bash
set -e

# Clean environment - remove all virtual env traces
unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH

# Reset PATH to system defaults
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# Force system Python
export PYTHON_EXECUTABLE=/usr/bin/python3

# Source ROS2 and build
source /opt/ros/kilted/setup.bash
cd "$1"
colcon build --packages-select sllidar_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
EOF
    
    chmod +x /tmp/build_sllidar.sh
    
    # Run the build script in a clean environment
    if /tmp/build_sllidar.sh "$NAV_WS"; then
        log_success "Built sllidar_ros2 driver successfully"
        rm -f /tmp/build_sllidar.sh
    else
        log_error "Failed to build sllidar_ros2 driver"
        rm -f /tmp/build_sllidar.sh
        log_error "Virtual environment conflicts are preventing the build."
        log_error ""
        log_error "Manual workaround:"
        log_error "1. Exit this script (Ctrl+C)"
        log_error "2. Deactivate virtual environment: deactivate"
        log_error "3. Run build manually:"
        log_error "   cd $NAV_WS"
        log_error "   source /opt/ros/kilted/setup.bash" 
        log_error "   colcon build --packages-select sllidar_ros2"
        log_error "4. Re-run this script to continue setup"
        exit 1
    fi
else
    # No virtual environment, build normally
    export PYTHON_EXECUTABLE=/usr/bin/python3
    source /opt/ros/$ROS_DISTRO/setup.bash
    if colcon build --packages-select sllidar_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3; then
        log_success "Built sllidar_ros2 driver successfully"
    else
        log_error "Failed to build sllidar_ros2 driver"
        exit 1
    fi
fi

# Configure USB permissions for C1
log_info "🔌 Configuring USB permissions for Slamtech C1..."
if ! groups | grep -q dialout; then
    log_info "Adding user to dialout group..."
    sudo usermod -a -G dialout $USER
    log_warning "You'll need to logout and login again for group changes to take effect"
fi

# Create udev rule for C1
UDEV_RULE="/etc/udev/rules.d/99-slamtech-c1.rules"
if [ ! -f "$UDEV_RULE" ]; then
    log_info "Creating udev rule for consistent C1 device naming..."
    sudo tee "$UDEV_RULE" > /dev/null << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="slamtech_c1"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    log_success "Created udev rule for Slamtech C1"
else
    log_success "Udev rule for Slamtech C1 already exists"
fi

# Deploy Mini Mapper navigation package
log_info "🚀 Deploying Mini Mapper navigation package..."
cd "$SCRIPT_DIR"
if [ -f "./deploy_ros2_package.sh" ]; then
    ./deploy_ros2_package.sh
    log_success "Mini Mapper navigation package deployed"
else
    log_error "deploy_ros2_package.sh not found in $SCRIPT_DIR"
    exit 1
fi

# Build complete workspace
log_info "🔨 Building complete navigation workspace..."
cd "$NAV_WS"

# Use the same isolated approach for the full workspace build
if [ -n "$VIRTUAL_ENV" ]; then
    log_info "Building complete workspace in isolated environment..."
    
    # Create a clean build script for the full workspace
    cat > /tmp/build_workspace.sh << 'EOF'
#!/bin/bash
set -e

# Clean environment - remove all virtual env traces
unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH

# Reset PATH to system defaults
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# Force system Python
export PYTHON_EXECUTABLE=/usr/bin/python3

# Source ROS2 and build
source /opt/ros/kilted/setup.bash
cd "$1"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
EOF
    
    chmod +x /tmp/build_workspace.sh
    
    # Run the build script in a clean environment
    if /tmp/build_workspace.sh "$NAV_WS"; then
        log_success "Built complete navigation workspace"
        rm -f /tmp/build_workspace.sh
    else
        log_error "Failed to build navigation workspace"
        rm -f /tmp/build_workspace.sh
        exit 1
    fi
else
    # No virtual environment, build normally
    export PYTHON_EXECUTABLE=/usr/bin/python3
    source /opt/ros/$ROS_DISTRO/setup.bash
    if colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3; then
        log_success "Built complete navigation workspace"
    else
        log_error "Failed to build navigation workspace"
        exit 1
    fi
fi

# Setup environment sourcing (avoid duplicates)
log_info "⚙️  Configuring environment setup..."
BASHRC_LINE="source $NAV_WS/install/setup.bash"
if ! grep -qF "$BASHRC_LINE" ~/.bashrc; then
    log_info "Adding workspace sourcing to ~/.bashrc..."
    echo "# Mini Mapper ROS2 Navigation Workspace" >> ~/.bashrc
    echo "$BASHRC_LINE" >> ~/.bashrc
    log_success "Added workspace sourcing to ~/.bashrc"
else
    log_success "Workspace sourcing already configured in ~/.bashrc"
fi

# Source the workspace for current session
source "$NAV_WS/install/setup.bash"

# Test the installation
log_info "🧪 Testing installation..."

# Check if mini_mapper_nav package is available
if ros2 pkg list | grep -q mini_mapper_nav; then
    log_success "mini_mapper_nav package found"
else
    log_error "mini_mapper_nav package not found"
    exit 1
fi

# Check if sllidar_ros2 package is available
if ros2 pkg list | grep -q sllidar_ros2; then
    log_success "sllidar_ros2 package found"
else
    log_error "sllidar_ros2 package not found"
    exit 1
fi

# Test C1 connection (if hardware is connected)
log_info "🔍 Checking for Slamtech C1 hardware..."
if lsusb | grep -i "10c4:ea60\|Silicon Labs\|CP210"; then
    log_success "Slamtech C1 lidar detected (or compatible USB-Serial device)"
    
    # Test if device is accessible
    for device in /dev/ttyUSB* /dev/ttyACM* /dev/slamtech_c1; do
        if [ -e "$device" ]; then
            if [ -r "$device" ] && [ -w "$device" ]; then
                log_success "C1 device accessible: $device"
            else
                log_warning "C1 device found but not accessible: $device"
                log_warning "You may need to logout/login for group changes to take effect"
            fi
            break
        fi
    done
else
    log_warning "Slamtech C1 not detected. Connect hardware to test lidar functionality."
fi

# Final success message
echo ""
log_success "🎉 Mini Mapper ROS2 environment setup complete!"
echo ""
echo "Next steps:"
echo "1. Connect your Slamtech C1 lidar via USB"
echo "2. Start Mini Mapper host on Raspberry Pi:"
echo "   python -m lerobot.robots.mini_mapper.mini_mapper_host --duration 0"
echo ""
echo "3. Launch SLAM mapping:"
echo "   cd $NAV_WS && source install/setup.bash"
echo "   ros2 launch mini_mapper_nav slam_launch.py"
echo ""
echo "4. In another terminal, control the robot:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "5. Save your map:"
echo "   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map"
echo ""
echo "📚 Full documentation: $LEROBOT_ROOT/src/lerobot/robots/mini_mapper/lidar_navigation_setup.md"
echo ""

if groups | grep -q dialout; then
    log_success "Ready to start mapping!"
else
    log_warning "Please logout and login again, then you'll be ready to start mapping!"
fi