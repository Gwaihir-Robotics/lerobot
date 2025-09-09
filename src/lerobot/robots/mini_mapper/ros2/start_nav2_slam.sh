#!/bin/bash

# Mini Mapper Nav2 SLAM System Startup Script
# Uses Nav2's robust SLAM instead of standalone SLAM toolbox

set -e

SESSION_NAME="mini_mapper_nav2_slam"

log_info() { echo -e "\033[0;34m[INFO]\033[0m $1"; }
log_success() { echo -e "\033[0;32m[SUCCESS]\033[0m $1"; }
log_error() { echo -e "\033[0;31m[ERROR]\033[0m $1"; }

# Check if tmux session already exists
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    log_error "Session '$SESSION_NAME' already exists"
    echo "To stop: tmux kill-session -t $SESSION_NAME"
    echo "To attach: tmux attach-session -t $SESSION_NAME"
    exit 1
fi

log_info "Starting Mini Mapper Nav2 SLAM system in tmux session: $SESSION_NAME"

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Window 0: Mini Mapper Host (with venv)
tmux rename-window -t $SESSION_NAME:0 'robot_host'
tmux send-keys -t $SESSION_NAME:0 'cd ~/lerobot && source venv/bin/activate' Enter
tmux send-keys -t $SESSION_NAME:0 'python -m lerobot.robots.mini_mapper.mini_mapper_host --duration 0' Enter

# Window 1: Nav2 SLAM Launch (comprehensive launch)
tmux new-window -t $SESSION_NAME -n 'nav2_slam'
tmux send-keys -t $SESSION_NAME:1 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
sleep 3  # Wait for robot host to start
tmux send-keys -t $SESSION_NAME:1 'ros2 launch mini_mapper_nav nav2_slam_launch.py' Enter

# Window 2: Teleop (ready but not started)
tmux new-window -t $SESSION_NAME -n 'teleop'
tmux send-keys -t $SESSION_NAME:2 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
tmux send-keys -t $SESSION_NAME:2 '# Run: ros2 run teleop_twist_keyboard teleop_twist_keyboard' Enter

# Window 3: Debug/Testing
tmux new-window -t $SESSION_NAME -n 'debug'
tmux send-keys -t $SESSION_NAME:3 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
tmux send-keys -t $SESSION_NAME:3 'echo "Nav2 SLAM debugging terminal ready"' Enter

log_success "Mini Mapper Nav2 SLAM system started!"
echo ""
echo "Commands:"
echo "  tmux attach-session -t $SESSION_NAME    # View/control sessions"  
echo "  tmux kill-session -t $SESSION_NAME      # Stop all processes"
echo ""
echo "In tmux:"
echo "  Ctrl+B + [0-3]  # Switch between windows"
echo "  Ctrl+B + d      # Detach (keeps running)"
echo ""
echo "Windows:"
echo "  0: robot_host   1: nav2_slam    2: teleop (ready)"
echo "  3: debug (testing terminal)"
echo ""
echo "Nav2 SLAM includes:"
echo "  - Lidar node"
echo "  - Static transforms" 
echo "  - Robot state publisher"
echo "  - Mini Mapper bridge"
echo "  - SLAM toolbox with loop closure"

# Attach to session
tmux attach-session -t $SESSION_NAME