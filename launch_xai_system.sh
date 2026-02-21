#!/bin/bash
# Tesla-Style Weighted XAI Navigation System Launcher
# Single command to start everything

set -e

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║  🤖 Tesla-Style Weighted XAI Navigation System              ║"
echo "║     Multi-Modal Fusion: Camera + LiDAR                       ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Check if already running
if tmux has-session -t weighted_xai_complete 2>/dev/null; then
    echo "⚠️  System already running!"
    echo ""
    read -p "Kill existing session and restart? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "🛑 Stopping existing session..."
        tmux kill-session -t weighted_xai_complete
        pkill -9 gzserver gzclient 2>/dev/null || true
        sleep 3
    else
        echo "📎 Attaching to existing session..."
        tmux attach -t weighted_xai_complete
        exit 0
    fi
fi

# Setup environment
cd ~/ros2_navigation_project
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.gazebo/models

SESSION="weighted_xai_complete"

echo "🚀 Starting system components..."
echo ""

# Create tmux session
tmux new-session -d -s $SESSION -n 'Gazebo'

# Window 0: Gazebo with custom world
echo "[1/5] Launching Gazebo with custom world..."
tmux send-keys -t $SESSION:0 "cd ~/ros2_navigation_project" C-m
tmux send-keys -t $SESSION:0 "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:0 "export TURTLEBOT3_MODEL=waffle_pi" C-m
tmux send-keys -t $SESSION:0 "ros2 launch xai_navigation_pkg weighted_xai_world.launch.py" C-m

# Window 1: Nav2 Localization
echo "[2/5] Setting up Nav2 localization..."
tmux new-window -t $SESSION -n 'Nav2'
tmux send-keys -t $SESSION:1 "sleep 20" C-m
tmux send-keys -t $SESSION:1 "cd ~/ros2_navigation_project" C-m
tmux send-keys -t $SESSION:1 "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:1 "export TURTLEBOT3_MODEL=waffle_pi" C-m
tmux send-keys -t $SESSION:1 "ros2 launch localization_server localization.launch.py" C-m

# Window 2: Path Planner
echo "[3/5] Setting up path planner..."
tmux new-window -t $SESSION -n 'Planner'
tmux send-keys -t $SESSION:2 "sleep 30" C-m
tmux send-keys -t $SESSION:2 "cd ~/ros2_navigation_project" C-m
tmux send-keys -t $SESSION:2 "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:2 "ros2 launch path_planner_server pathplanner.launch.py" C-m

# Window 3: XAI Navigator
echo "[4/5] Setting up XAI Navigator with multi-modal fusion..."
tmux new-window -t $SESSION -n 'XAI'
tmux send-keys -t $SESSION:3 "sleep 40" C-m
tmux send-keys -t $SESSION:3 "cd ~/ros2_navigation_project" C-m
tmux send-keys -t $SESSION:3 "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:3 "ros2 launch xai_navigation_pkg xai_navigator.launch.py enable_explanations:=true enable_obstacle_weighting:=true" C-m

# Window 4: YOLO Perception
echo "[5/5] Setting up YOLO-World perception..."
tmux new-window -t $SESSION -n 'YOLO'
tmux send-keys -t $SESSION:4 "sleep 50" C-m
tmux send-keys -t $SESSION:4 "cd ~/ros2_navigation_project" C-m
tmux send-keys -t $SESSION:4 "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:4 "ros2 launch xai_navigation_pkg yolo_perception.launch.py device:=cuda:0 threshold:=0.35 input_image_topic:=/camera/image_raw" C-m

# Window 5: Quick Commands
tmux new-window -t $SESSION -n 'Commands'
tmux send-keys -t $SESSION:5 "cd ~/ros2_navigation_project" C-m
tmux send-keys -t $SESSION:5 "source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:5 "sleep 60" C-m
tmux send-keys -t $SESSION:5 "clear" C-m
tmux send-keys -t $SESSION:5 "cat << 'EOF'
╔══════════════════════════════════════════════════════════════════╗
║  ✅ SYSTEM READY - All components initialized                    ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  🤖 Robot: TurtleBot3 Waffle Pi (WITH CAMERA!)                   ║
║  🌍 World: weighted_xai_demo.world                                ║
║                                                                   ║
║  📍 Obstacle Locations:                                           ║
║     👤 Person 1: (2.0, 0.0) - Priority 10.0                      ║
║     👤 Person 2: (-3.0, 4.0) - Priority 10.0                     ║
║     🛒 Cart: (-2.0, -2.0) - Priority 5.0                         ║
║     💺 Chairs: (4.0, 3.0), (4.5, 4.5), (-4.0, -4.0) - Priority 2.0 ║
║                                                                   ║
╠══════════════════════════════════════════════════════════════════╣
║  🎯 Quick Commands:                                               ║
╚══════════════════════════════════════════════════════════════════╝

# Set initial pose for localization
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\
  \"{header: {frame_id: 'map'}, \\
    pose: {pose: {position: {x: -6.0, y: -6.0, z: 0.0}, \\
                  orientation: {w: 1.0}}}}\"

# Navigate to Person 1 (test multi-modal fusion)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
  \"{header: {frame_id: 'map'}, \\
    pose: {position: {x: 2.0, y: 0.0, z: 0.0}, \\
          orientation: {w: 1.0}}}\"

# Monitor YOLO detections
ros2 topic echo /yolo/detections

# Monitor obstacle classifications
ros2 topic echo /navigation/obstacle_classification

# Check all nodes
ros2 node list

# Navigate between windows: Ctrl+B, then 0-5
# Detach from session: Ctrl+B, then D

EOF
" C-m

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ✅ SYSTEM LAUNCHED SUCCESSFULLY!                            ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  ⏱️  Initialization time: ~60 seconds                        ║"
echo "║                                                              ║"
echo "║  Windows:                                                    ║"
echo "║    0: Gazebo    - Simulation with custom world              ║"
echo "║    1: Nav2      - Localization (AMCL)                        ║"
echo "║    2: Planner   - Nav2 path planning                         ║"
echo "║    3: XAI       - Multi-modal fusion navigator               ║"
echo "║    4: YOLO      - Camera-based detection                     ║"
echo "║    5: Commands  - Quick reference commands                   ║"
echo "║                                                              ║"
echo "║  📎 To attach: tmux attach -t weighted_xai_complete          ║"
echo "║  🛑 To stop:   tmux kill-session -t weighted_xai_complete    ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "⏳ Waiting 60 seconds for all components to initialize..."
echo "   You can attach now or wait for automatic attachment..."
echo ""

sleep 55

echo ""
echo "🎬 Attaching to session in 5 seconds..."
echo "   (Press Ctrl+B, D to detach anytime)"
sleep 5

# Attach to the session
tmux attach -t $SESSION
