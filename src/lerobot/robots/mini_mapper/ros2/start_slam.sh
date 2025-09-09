#!/bin/bash

# Mini Mapper SLAM System Startup Script
# Alternative to overmind - runs all processes in background with tmux

set -e

SESSION_NAME="mini_mapper_slam"

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

log_info "Starting Mini Mapper SLAM system in tmux session: $SESSION_NAME"

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Window 0: Lidar
tmux rename-window -t $SESSION_NAME:0 'lidar'
tmux send-keys -t $SESSION_NAME:0 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
tmux send-keys -t $SESSION_NAME:0 'ros2 launch sllidar_ros2 view_sllidar_c1_launch.py' Enter

# Window 1: Robot Host
tmux new-window -t $SESSION_NAME -n 'robot_host'
tmux send-keys -t $SESSION_NAME:1 'cd ~/lerobot && source venv/bin/activate' Enter
tmux send-keys -t $SESSION_NAME:1 'python -m lerobot.robots.mini_mapper.mini_mapper_host --duration 0' Enter

# Window 2: Robot Bridge
tmux new-window -t $SESSION_NAME -n 'robot_bridge'
tmux send-keys -t $SESSION_NAME:2 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
sleep 2  # Wait for robot host to start
tmux send-keys -t $SESSION_NAME:2 'ros2 run mini_mapper_nav mini_mapper_bridge' Enter

# Window 3: Static Transform (base_link -> laser)
tmux new-window -t $SESSION_NAME -n 'static_tf'
tmux send-keys -t $SESSION_NAME:3 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
tmux send-keys -t $SESSION_NAME:3 'ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link laser' Enter

# Window 4: SLAM
tmux new-window -t $SESSION_NAME -n 'slam'
tmux send-keys -t $SESSION_NAME:4 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
sleep 3  # Wait for other components
tmux send-keys -t $SESSION_NAME:4 'ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=false -p odom_frame:=odom -p map_frame:=map -p base_frame:=base_link -p scan_topic:=/scan -p resolution:=0.05 -p max_laser_range:=12.0 -p minimum_travel_distance:=0.2 -p minimum_travel_heading:=0.17 --log-level debug' Enter

# Window 6: SLAM Lifecycle Activation (auto-activates SLAM then closes)
tmux new-window -t $SESSION_NAME -n 'slam_activator'
tmux send-keys -t $SESSION_NAME:6 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
tmux send-keys -t $SESSION_NAME:6 'sleep 5' Enter  # Wait for SLAM node to start
tmux send-keys -t $SESSION_NAME:6 'echo "Activating SLAM toolbox lifecycle..."' Enter
tmux send-keys -t $SESSION_NAME:6 'ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"' Enter
tmux send-keys -t $SESSION_NAME:6 'sleep 2' Enter
tmux send-keys -t $SESSION_NAME:6 'ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"' Enter
tmux send-keys -t $SESSION_NAME:6 'echo "SLAM toolbox activated! Map should appear in RViz."' Enter
tmux send-keys -t $SESSION_NAME:6 'echo "Window will close in 5 seconds..."' Enter
tmux send-keys -t $SESSION_NAME:6 'sleep 5 && exit' Enter

# Window 5: Control (ready but not started)
tmux new-window -t $SESSION_NAME -n 'teleop'
tmux send-keys -t $SESSION_NAME:5 'cd ~/nav_ws && export ROS_DOMAIN_ID=42 && source install/setup.bash' Enter
tmux send-keys -t $SESSION_NAME:5 '# Run: ros2 run teleop_twist_keyboard teleop_twist_keyboard' Enter

log_success "Mini Mapper SLAM system started!"
echo ""
echo "Commands:"
echo "  tmux attach-session -t $SESSION_NAME    # View/control sessions"  
echo "  tmux kill-session -t $SESSION_NAME      # Stop all processes"
echo ""
echo "In tmux:"
echo "  Ctrl+B + [0-5]  # Switch between windows"
echo "  Ctrl+B + d      # Detach (keeps running)"
echo ""
echo "Windows:"
echo "  0: lidar        1: robot_host    2: robot_bridge"
echo "  3: static_tf    4: slam          5: teleop (ready)"
echo "  6: slam_activator (auto-closes)"

# Attach to session
tmux attach-session -t $SESSION_NAME