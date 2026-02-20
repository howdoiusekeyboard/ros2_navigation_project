#!/bin/bash
# ============================================================================
# Weighted XAI Navigation Demo Script
# ============================================================================
#
# This script launches the full navigation stack with Tesla-style weighted
# obstacle classification. It demonstrates:
#
# 1. Weighted obstacle prioritization (human > vehicle > furniture > wall)
# 2. Real-time classification based on zones, velocity, and size
# 3. Priority-aware explanations from Gemini
# 4. Dashboard with color-coded obstacle types
#
# Usage: ./demo_weighted_xai.sh
#
# ============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${PURPLE}"
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║                                                                   ║"
echo "║   🤖 WEIGHTED XAI NAVIGATION DEMO                                ║"
echo "║   Tesla-Style Obstacle Prioritization                            ║"
echo "║                                                                   ║"
echo "║   Human (10x) > Vehicle (5x) > Furniture (2x) > Wall (1x)        ║"
echo "║                                                                   ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check for tmux
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}Error: tmux is not installed. Please install it with: sudo apt install tmux${NC}"
    exit 1
fi

SESSION="weighted_xai_demo"

# Check if GEMINI_API_KEY is set
if [ -z "$GEMINI_API_KEY" ]; then
    echo -e "${YELLOW}⚠ Warning: GEMINI_API_KEY not set. Explanations will use fallback templates.${NC}"
    echo ""
    echo "To enable Gemini-powered explanations:"
    echo "  export GEMINI_API_KEY='your_key_here'"
    echo ""
    echo "Get your API key from: https://aistudio.google.com/app/apikey"
    echo ""
    read -p "Continue without Gemini? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo -e "${GREEN}✓ GEMINI_API_KEY detected${NC}"
fi

# Set TurtleBot model
export TURTLEBOT3_MODEL=waffle

echo -e "${CYAN}[1/6] Setting up environment...${NC}"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null || true

# Create new session with a large window size
tmux new-session -d -s $SESSION -x 220 -y 70

# ============================================================================
# Window 0: Simulation & Navigation
# ============================================================================
tmux rename-window -t $SESSION:0 'Simulation'

echo -e "${CYAN}[2/6] Starting Gazebo simulation...${NC}"

# Pane 0 (Top): Gazebo Simulation
tmux send-keys -t $SESSION:0 'export TURTLEBOT3_MODEL=waffle' C-m
tmux send-keys -t $SESSION:0 'echo "🌍 Launching Gazebo world..."' C-m
tmux send-keys -t $SESSION:0 'ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py' C-m

# Split for Navigation Stack
tmux split-window -v -t $SESSION:0 -p 35

echo -e "${CYAN}[3/6] Waiting for Gazebo to initialize (15s)...${NC}"

tmux send-keys -t $SESSION:0.1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:0.1 'source install/setup.bash 2>/dev/null || true' C-m
tmux send-keys -t $SESSION:0.1 'echo "⏳ Waiting for Gazebo (15s)..."' C-m
tmux send-keys -t $SESSION:0.1 'sleep 15' C-m
tmux send-keys -t $SESSION:0.1 'echo "🗺️ Starting Navigation Stack..."' C-m
tmux send-keys -t $SESSION:0.1 'ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml' C-m

# ============================================================================
# Window 1: XAI System
# ============================================================================
tmux new-window -t $SESSION -n 'XAI_Brain'

echo -e "${CYAN}[4/6] Preparing XAI Navigator with weighted classification...${NC}"

# Pane 0 (Left): XAI Navigator Node
tmux send-keys -t $SESSION:1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:1 'source install/setup.bash 2>/dev/null || true' C-m
tmux send-keys -t $SESSION:1 'echo "⏳ Waiting for Nav2 to be ready (20s)..."' C-m
tmux send-keys -t $SESSION:1 'sleep 20' C-m
tmux send-keys -t $SESSION:1 'echo "🧠 Launching XAI Navigator with WEIGHTED CLASSIFICATION..."' C-m

if [ -n "$GEMINI_API_KEY" ]; then
    tmux send-keys -t $SESSION:1 "export GEMINI_API_KEY='$GEMINI_API_KEY'" C-m
    tmux send-keys -t $SESSION:1 'ros2 launch xai_navigation_pkg xai_navigator.launch.py enable_explanations:=true enable_obstacle_weighting:=true' C-m
else
    tmux send-keys -t $SESSION:1 'ros2 launch xai_navigation_pkg xai_navigator.launch.py enable_explanations:=false enable_obstacle_weighting:=true' C-m
fi

# Split for explanation handler
tmux split-window -h -t $SESSION:1

tmux send-keys -t $SESSION:1.1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:1.1 'source install/setup.bash 2>/dev/null || true' C-m
tmux send-keys -t $SESSION:1.1 'sleep 25' C-m
tmux send-keys -t $SESSION:1.1 'echo "🔊 Starting explanation handler..."' C-m
tmux send-keys -t $SESSION:1.1 'ros2 run conversation_memory_pkg explanation_handler --ros-args -p auto_speak_explanations:=true' C-m

# Split for demo controls
tmux split-window -v -t $SESSION:1.1 -p 30

tmux send-keys -t $SESSION:1.2 'echo ""' C-m
tmux send-keys -t $SESSION:1.2 'echo "╔════════════════════════════════════════════════════════════╗"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  🎮 DEMO CONTROLS                                           ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "╠════════════════════════════════════════════════════════════╣"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║                                                              ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  1. Use RViz2 to set initial pose (2D Pose Estimate)        ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  2. Send navigation goals (Nav2 Goal)                       ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  3. Spawn obstacles in Gazebo to trigger classification     ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║                                                              ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  MONITOR TOPICS:                                             ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  ros2 topic echo /navigation/explanation                     ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  ros2 topic echo /navigation/obstacle_classification         ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║                                                              ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  PRIORITY WEIGHTS:                                           ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  🔴 Human: 10.0    🟠 Vehicle: 5.0                          ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  🟡 Dynamic: 3.0   🔵 Furniture: 2.0                        ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║  ⚪ Wall: 1.0      ❓ Unknown: 1.5                          ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "║                                                              ║"' C-m
tmux send-keys -t $SESSION:1.2 'echo "╚════════════════════════════════════════════════════════════╝"' C-m

# ============================================================================
# Window 2: Monitoring
# ============================================================================
tmux new-window -t $SESSION -n 'Monitor'

echo -e "${CYAN}[5/6] Setting up monitoring...${NC}"

# Classification monitor
tmux send-keys -t $SESSION:2 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:2 'echo "📊 Obstacle Classification Monitor"' C-m
tmux send-keys -t $SESSION:2 'echo "Waiting for topics..."' C-m
tmux send-keys -t $SESSION:2 'sleep 30' C-m
tmux send-keys -t $SESSION:2 'ros2 topic echo /navigation/obstacle_classification' C-m

# Split for explanation monitor
tmux split-window -h -t $SESSION:2

tmux send-keys -t $SESSION:2.1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:2.1 'echo "💬 Explanation Monitor"' C-m
tmux send-keys -t $SESSION:2.1 'sleep 30' C-m
tmux send-keys -t $SESSION:2.1 'ros2 topic echo /navigation/explanation' C-m

# ============================================================================
# Window 3: Web Dashboard
# ============================================================================
tmux new-window -t $SESSION -n 'Dashboard'

echo -e "${CYAN}[6/6] Starting web dashboard...${NC}"

tmux send-keys -t $SESSION:3 'echo "🌐 Starting Web Dashboard..."' C-m
tmux send-keys -t $SESSION:3 'cd project' C-m
tmux send-keys -t $SESSION:3 'sleep 5' C-m
tmux send-keys -t $SESSION:3 'npm run dev -- --host 2>/dev/null || bun run dev --host' C-m

# Select the XAI window
tmux select-window -t $SESSION:1

# ============================================================================
# Completion
# ============================================================================

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  ✅ DEMO ENVIRONMENT READY                                       ║${NC}"
echo -e "${GREEN}╠══════════════════════════════════════════════════════════════════╣${NC}"
echo -e "${GREEN}║                                                                   ║${NC}"
echo -e "${GREEN}║  Windows:                                                         ║${NC}"
echo -e "${GREEN}║    0: Simulation  - Gazebo + Nav2                                ║${NC}"
echo -e "${GREEN}║    1: XAI_Brain   - XAI Navigator + Explanations                 ║${NC}"
echo -e "${GREEN}║    2: Monitor     - Classification + Explanation topics          ║${NC}"
echo -e "${GREEN}║    3: Dashboard   - Web interface                                ║${NC}"
echo -e "${GREEN}║                                                                   ║${NC}"
echo -e "${GREEN}║  Navigation Controls:                                             ║${NC}"
echo -e "${GREEN}║    Ctrl+B, 0-3  - Switch windows                                 ║${NC}"
echo -e "${GREEN}║    Ctrl+B, D    - Detach from session                            ║${NC}"
echo -e "${GREEN}║                                                                   ║${NC}"
echo -e "${GREEN}║  Web Dashboard: http://localhost:5173                            ║${NC}"
echo -e "${GREEN}║                                                                   ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}Attaching to tmux session in 3 seconds...${NC}"
sleep 3

# Attach to session
tmux attach-session -t $SESSION
