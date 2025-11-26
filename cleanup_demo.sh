#!/bin/bash
# Cleanup Script for Week 4 Demo

SESSION="week4_demo"

echo "Stopping Week 4 Demo..."

# 1. Kill the tmux session
if tmux has-session -t $SESSION 2>/dev/null; then
    tmux kill-session -t $SESSION
    echo "✅ Tmux session '$SESSION' killed."
else
    echo "⚠️  Tmux session '$SESSION' not found."
fi

# 2. Kill lingering processes
echo "Cleaning up lingering processes..."
pkill -f "ros2 launch"
pkill -f "gzserver"
pkill -f "gzclient"
pkill -f "xai_navigator"
pkill -f "explanation_handler"
pkill -f "nav2"
pkill -f "robot_state_publisher"
pkill -f "rviz2"

echo "✅ Cleanup complete. All systems offline."
