#!/bin/bash
# Week 4 Demo Automation Script
# Usage: ./demo_week4.sh

# Check for tmux
if ! command -v tmux &> /dev/null; then
    echo "Error: tmux is not installed. Please install it with: sudo apt install tmux"
    exit 1
fi

SESSION="week4_demo"
API_KEY="AIzaSyDbgs1D3YYB0PrMb7NMy2WaMuzvzbEf5Og"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Create new session with a large window size to avoid layout issues
tmux new-session -d -s $SESSION -x 200 -y 60

# ==========================================
# Window 0: Simulation & Navigation
# ==========================================
tmux rename-window -t $SESSION:0 'Sim_and_Nav'

# Pane 0 (Top): Gazebo Simulation
tmux send-keys -t $SESSION:0 'export TURTLEBOT3_MODEL=waffle' C-m
tmux send-keys -t $SESSION:0 'echo "Launching Gazebo..."' C-m
tmux send-keys -t $SESSION:0 'ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py' C-m

# Split Pane 1 (Bottom): Navigation Stack
tmux split-window -v -t $SESSION:0 -p 30
tmux send-keys -t $SESSION:0.1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:0.1 'echo "Waiting for Gazebo to start (10s)..."' C-m
tmux send-keys -t $SESSION:0.1 'sleep 10' C-m
tmux send-keys -t $SESSION:0.1 'ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml' C-m

# ==========================================
# Window 1: XAI System & Voice
# ==========================================
tmux new-window -t $SESSION -n 'XAI_Brain'

# Pane 0 (Left): XAI Navigator Node
tmux send-keys -t $SESSION:1 "export GEMINI_API_KEY='$API_KEY'" C-m
tmux send-keys -t $SESSION:1 'echo "Launching XAI Navigator..."' C-m
tmux send-keys -t $SESSION:1 'ros2 launch xai_navigation_pkg xai_navigator.launch.py enable_explanations:=true' C-m

# Split Pane 1 (Right): Voice Interface
tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1.1 'echo "Launching Voice Interface..."' C-m
tmux send-keys -t $SESSION:1.1 'ros2 run conversation_memory_pkg explanation_handler --ros-args -p auto_speak_explanations:=true' C-m

# Split Pane 2 (Bottom Right): User Input Helpe
tmux split-window -v -t $SESSION:1.1 -p 20
tmux send-keys -t $SESSION:1.2 'echo "--- DEMO CONTROLS ---"' C-m
tmux send-keys -t $SESSION:1.2 'echo "To ask a question, run:"' C-m
tmux send-keys -t $SESSION:1.2 'echo "ros2 topic pub --once /conversation/user_input std_msgs/msg/String \"data: 'Why did you stop?'\""' C-m
tmux send-keys -t $SESSION:1.2 'history -s "ros2 topic pub --once /conversation/user_input std_msgs/msg/String \"data: 'Why did you stop?'\""' C-m

# Select the XAI window by default
tmux select-window -t $SESSION:1

echo "Demo environment started in tmux session '$SESSION'."
echo "Attaching to session... (Press Ctrl+B then D to detach)"
sleep 1

# Attach to the session
tmux attach-session -t $SESSION
