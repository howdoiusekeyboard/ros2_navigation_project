#!/bin/bash
# FIXED Tesla-Style Weighted XAI System - Complete Launch
# Fixes: Map mismatch, spawn position, adds RViz2

set -e

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  🤖 FIXED Tesla-Style Weighted XAI Navigation System         ║"
echo "║                                                              ║"
echo "║  ✅ Map matches custom world (-7.5 to +7.5)                  ║"
echo "║  ✅ Robot spawns within map bounds                           ║"
echo "║  ✅ RViz2 visualization included                             ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Cleanup
if tmux has-session -t xai_fixed 2>/dev/null; then
    tmux kill-session -t xai_fixed
fi
pkill -9 gzserver gzclient 2>/dev/null || true
sleep 2

cd ~/ros2_navigation_project
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.gazebo/models

SESSION="xai_fixed"

echo "🚀 Launching system..."

# Create session
tmux new-session -d -s $SESSION -n 'Gazebo'

# Window 0: Gazebo
echo "[1/6] Gazebo with custom world..."
tmux send-keys -t $SESSION:0 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi" C-m
tmux send-keys -t $SESSION:0 "ros2 launch xai_navigation_pkg weighted_xai_world.launch.py" C-m

# Window 1: RViz2
echo "[2/6] RViz2 visualization..."
tmux new-window -t $SESSION -n 'RViz'
tmux send-keys -t $SESSION:1 "sleep 15" C-m
tmux send-keys -t $SESSION:1 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi" C-m
tmux send-keys -t $SESSION:1 "rviz2 -d \$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz" C-m

# Window 2: Localization (FIXED MAP!)
echo "[3/6] Localization with matching map..."
tmux new-window -t $SESSION -n 'Localization'
tmux send-keys -t $SESSION:2 "sleep 20" C-m
tmux send-keys -t $SESSION:2 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi" C-m
tmux send-keys -t $SESSION:2 "ros2 launch localization_server localization.launch.py" C-m

# Window 3: Path Planner
echo "[4/6] Path planner..."
tmux new-window -t $SESSION -n 'Planner'
tmux send-keys -t $SESSION:3 "sleep 30" C-m
tmux send-keys -t $SESSION:3 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:3 "ros2 launch path_planner_server pathplanner.launch.py" C-m

# Window 4: XAI Navigator
echo "[5/6] XAI Navigator with fusion..."
tmux new-window -t $SESSION -n 'XAI'
tmux send-keys -t $SESSION:4 "sleep 40" C-m
tmux send-keys -t $SESSION:4 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:4 "ros2 launch xai_navigation_pkg xai_navigator.launch.py enable_explanations:=true enable_obstacle_weighting:=true" C-m

# Window 5: YOLO
echo "[6/6] YOLO perception..."
tmux new-window -t $SESSION -n 'YOLO'
tmux send-keys -t $SESSION:5 "sleep 50" C-m
tmux send-keys -t $SESSION:5 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:5 "ros2 launch xai_navigation_pkg yolo_perception.launch.py device:=cuda:0 threshold:=0.35 input_image_topic:=/camera/image_raw" C-m

# Window 6: Test Commands
tmux new-window -t $SESSION -n 'Commands'
tmux send-keys -t $SESSION:6 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:6 "sleep 60 && clear" C-m
tmux send-keys -t $SESSION:6 "cat << 'EOF'

╔══════════════════════════════════════════════════════════════════╗
║  ✅ SYSTEM READY - All Fixed and Operational!                    ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  📍 Map: weighted_xai_map (15m x 15m, -7.5 to +7.5)               ║
║  🤖 Robot: At (-6, -6) WITHIN map bounds                          ║
║  📊 RViz: Window 1 (Ctrl+B, 1)                                    ║
║                                                                   ║
║  🎯 WORKING Navigation Commands:                                  ║
╚══════════════════════════════════════════════════════════════════╝

# Navigate to center (0, 0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
  \"{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"

# Navigate to Person 1 (2, 0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
  \"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"

# Navigate to goal marker (6, 6)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
  \"{header: {frame_id: 'map'}, pose: {position: {x: 6.0, y: 6.0, z: 0.0}, orientation: {w: 1.0}}}\"

# Check robot is moving
ros2 topic echo /cmd_vel

# Monitor YOLO
ros2 topic echo /yolo/detections

# Monitor classifications
ros2 topic echo /navigation/obstacle_classification

# Check all nodes
ros2 node list | grep -E 'xai|yolo|amcl|planner'

EOF
" C-m

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ✅ FIXED SYSTEM LAUNCHED!                                   ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║                                                              ║"
echo "║  Windows:                                                    ║"
echo "║    0: Gazebo         1: RViz2         2: Localization        ║"
echo "║    3: Planner        4: XAI           5: YOLO                ║"
echo "║    6: Commands (test goals here!)                            ║"
echo "║                                                              ║"
echo "║  📎 Attach: tmux attach -t xai_fixed                         ║"
echo "║  🛑 Stop:   tmux kill-session -t xai_fixed                   ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "⏳ Waiting 60s for initialization, then auto-attaching..."
sleep 60
tmux attach -t xai_fixed
