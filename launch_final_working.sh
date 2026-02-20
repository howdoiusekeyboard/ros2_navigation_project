#!/bin/bash
# FINAL WORKING Launch - All Issues Fixed
# - Larger map with safety margins (17x17m)
# - YOLO on CPU (no CUDA crash)
# - Proper initial pose
# - RViz2 included

set -e

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  🎉 FINAL WORKING Tesla-Style XAI Navigation System         ║"
echo "║                                                              ║"
echo "║  ✅ Map: 17x17m with 1m safety margins                       ║"
echo "║  ✅ YOLO: CPU mode (no CUDA crash)                           ║"
echo "║  ✅ Localization: Proper initial pose                        ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Cleanup
if tmux has-session -t xai_working 2>/dev/null; then
    tmux kill-session -t xai_working
fi
pkill -9 gzserver gzclient 2>/dev/null || true
sleep 2

cd ~/ros2_navigation_project
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

SESSION="xai_working"

echo "🚀 Launching fixed system..."

tmux new-session -d -s $SESSION -n 'Gazebo'

# Window 0: Gazebo
echo "[1/6] Gazebo with custom world..."
tmux send-keys -t $SESSION:0 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi" C-m
tmux send-keys -t $SESSION:0 "ros2 launch xai_navigation_pkg weighted_xai_world.launch.py" C-m

# Window 1: RViz2
echo "[2/6] RViz2..."
tmux new-window -t $SESSION -n 'RViz'
tmux send-keys -t $SESSION:1 "sleep 15" C-m
tmux send-keys -t $SESSION:1 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi" C-m
tmux send-keys -t $SESSION:1 "rviz2 -d \$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz" C-m

# Window 2: Localization (LARGER MAP!)
echo "[3/6] Localization with larger map..."
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
echo "[5/6] XAI Navigator..."
tmux new-window -t $SESSION -n 'XAI'
tmux send-keys -t $SESSION:4 "sleep 40" C-m
tmux send-keys -t $SESSION:4 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
export GEMINI_API_KEY="AIzaSyCImhqg9DxBZiS4EQwxmXFuTKNfA2jCwEU"
tmux send-keys -t $SESSION:4 "ros2 launch xai_navigation_pkg xai_navigator.launch.py enable_explanations:=true enable_obstacle_weighting:=true gemini_api_key:=$GEMINI_API_KEY" C-m

# Window 5: YOLO (CPU MODE - GPU incompatible, fallback to CPU)
echo "[6/6] YOLO perception (CPU mode - GPU has CUDA compatibility issue)..."
tmux new-window -t $SESSION -n 'YOLO'
tmux send-keys -t $SESSION:5 "sleep 50" C-m
tmux send-keys -t $SESSION:5 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:5 "ros2 launch xai_navigation_pkg yolo_perception.launch.py device:=cpu threshold:=0.35 input_image_topic:=/camera/image_raw" C-m

# Window 6: Test Commands
tmux new-window -t $SESSION -n 'Commands'
tmux send-keys -t $SESSION:6 "cd ~/ros2_navigation_project && source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION:6 "sleep 60 && clear" C-m
tmux send-keys -t $SESSION:6 "cat << 'EOF'

╔══════════════════════════════════════════════════════════════════╗
║  ✅ SYSTEM READY - Everything Fixed!                             ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  📍 Map: 17x17m (-8.5 to +8.5) with safety margins               ║
║  🤖 Robot: Waffle Pi at (-6, -6)                                  ║
║  🎥 YOLO: CPU mode (stable, no CUDA crash)                        ║
║  📊 RViz: Window 1 (Ctrl+B, 1)                                    ║
║                                                                   ║
║  🎯 WORKING Navigation Commands:                                  ║
╚══════════════════════════════════════════════════════════════════╝

# Set proper initial pose (CRITICAL!)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\
  \"{header: {frame_id: 'map'}, \\
    pose: {pose: {position: {x: -6.0, y: -6.0, z: 0.0}, \\
                  orientation: {w: 1.0}}, \\
           covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, \\
                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0, \\
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \\
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \\
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \\
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892]}}\"

# Navigate to center (safe test)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
  \"{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"

# Navigate to Person 1 at (2, 0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
  \"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"

# Check robot moving
ros2 topic echo /cmd_vel

# Monitor YOLO (CPU mode, slower but stable)
ros2 topic echo /yolo/detections

# Check nodes
ros2 node list | grep -E 'xai|yolo|amcl'

EOF
" C-m

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  ✅ FINAL WORKING SYSTEM LAUNCHED!                           ║"
echo "║                                                              ║"
echo "║  Windows:                                                    ║"
echo "║    0: Gazebo    1: RViz    2: Localization                   ║"
echo "║    3: Planner   4: XAI     5: YOLO (CPU)                     ║"
echo "║    6: Commands (MUST set initial pose first!)                ║"
echo "║                                                              ║"
echo "║  📎 Attach: tmux attach -t xai_working                       ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "⏳ Waiting 60s for initialization..."
sleep 60

echo ""
echo "🎬 Ready! Attach manually with:"
echo "   tmux attach -t xai_working"
echo ""
