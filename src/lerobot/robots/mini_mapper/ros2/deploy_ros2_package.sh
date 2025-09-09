#!/bin/bash

# Deploy Mini Mapper ROS2 Navigation Package
# This script copies the ROS2 template files to the user's workspace

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/nav_ws/src"
PACKAGE_NAME="mini_mapper_nav"

echo "🚀 Mini Mapper ROS2 Package Deployment"
echo "======================================"
echo "Source: $SCRIPT_DIR"
echo "Target: $WORKSPACE_DIR/$PACKAGE_NAME"
echo ""

# Check if nav_ws exists
if [ ! -d "$HOME/nav_ws" ]; then
    echo "❌ ROS2 workspace not found at $HOME/nav_ws"
    echo "Please create the workspace first:"
    echo "  mkdir -p $HOME/nav_ws/src"
    echo "  cd $HOME/nav_ws"
    echo "  colcon build"
    exit 1
fi

# Create workspace src directory if it doesn't exist
mkdir -p "$WORKSPACE_DIR"

# Remove existing package directory
if [ -d "$WORKSPACE_DIR/$PACKAGE_NAME" ]; then
    echo "⚠️  Removing existing package: $WORKSPACE_DIR/$PACKAGE_NAME"
    rm -rf "$WORKSPACE_DIR/$PACKAGE_NAME"
fi

# Copy the entire ROS2 package
echo "📦 Copying ROS2 package files..."
cp -r "$SCRIPT_DIR" "$WORKSPACE_DIR/$PACKAGE_NAME"

# Remove the deployment script from the copied package
rm -f "$WORKSPACE_DIR/$PACKAGE_NAME/deploy_ros2_package.sh"

# Make the node executable
chmod +x "$WORKSPACE_DIR/$PACKAGE_NAME/mini_mapper_nav/mini_mapper_node.py"

echo "✅ Package deployed successfully!"
echo ""
echo "Next steps:"
echo "1. Build the workspace:"
echo "   cd $HOME/nav_ws"
echo "   colcon build --packages-select $PACKAGE_NAME"
echo "   source install/setup.bash"
echo ""
echo "2. Test the package:"
echo "   ros2 launch $PACKAGE_NAME slam_launch.py"
echo ""
echo "Package contents:"
find "$WORKSPACE_DIR/$PACKAGE_NAME" -type f | sort