#!/bin/bash
# Real Robot Navigation Demo with XAI Explanations
# Replicates demo_week4.sh functionality on physical TurtleBot3
#
# Usage: ./demo_real_nav.sh
#
# Prerequisites (run on TurtleBot3 FIRST):
#   Terminal 1: ros2 launch turtlebot3_bringup robot.launch.py
#   Terminal 2: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
#   Terminal 3: ./scripts/robot_nav_launch.sh /path/to/map.yaml
#
# This script runs on WSL and provides:
#   - XAI Bridge (relays Nav2 topics from robot)
#   - XAI Navigator (decision logging + Gemini explanations)
#   - Behavior Monitor (anomaly detection)
#   - Backend Server (voice command processing)
#   - Web Dashboard (visualization)

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Configuration
ROBOT_IP="${ROBOT_IP:-10.30.96.171}"
SESSION="real_nav_xai"
ROSBRIDGE_PORT=9090
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗"
echo -e "║     Real Robot Navigation Demo with XAI Explanations        ║"
echo -e "╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check prerequisites
echo -e "${YELLOW}Checking prerequisites...${NC}"

# Check tmux
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}Error: tmux not installed${NC}"
    echo "Install with: sudo apt install tmux"
    exit 1
fi

# Check GEMINI_API_KEY
if [ -z "$GEMINI_API_KEY" ]; then
    echo -e "${RED}Error: GEMINI_API_KEY not set${NC}"
    echo ""
    echo "Set it with:"
    echo "  export GEMINI_API_KEY='your_key_here'"
    echo ""
    echo "Get your key from: https://aistudio.google.com/app/apikey"
    exit 1
fi
echo -e "${GREEN}✓ GEMINI_API_KEY: ${GEMINI_API_KEY:0:15}...${NC}"

# Check robot connectivity
echo -e "${YELLOW}Checking robot at $ROBOT_IP...${NC}"
if ! ping -c 1 -W 2 "$ROBOT_IP" &> /dev/null; then
    echo -e "${RED}Error: Cannot reach robot at $ROBOT_IP${NC}"
    echo ""
    echo "Make sure:"
    echo "1. TurtleBot3 is powered on"
    echo "2. Connected to same network"
    echo "3. Robot IP is correct (set with: ROBOT_IP=x.x.x.x ./demo_real_nav.sh)"
    exit 1
fi
echo -e "${GREEN}✓ Robot reachable at $ROBOT_IP${NC}"

# Check rosbridge on robot
echo -e "${YELLOW}Checking rosbridge on robot...${NC}"
if ! timeout 2 bash -c "echo >/dev/tcp/$ROBOT_IP/$ROSBRIDGE_PORT" 2>/dev/null; then
    echo -e "${RED}Error: rosbridge not running on robot${NC}"
    echo ""
    echo "On the TurtleBot3, run:"
    echo "  ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    exit 1
fi
echo -e "${GREEN}✓ rosbridge accessible at ws://$ROBOT_IP:$ROSBRIDGE_PORT${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || true

# Check if Nav2 is running on robot by checking for /plan topic via rosbridge
echo -e "${YELLOW}Checking Nav2 on robot...${NC}"
echo -e "${CYAN}(Nav2 should be running on robot with a map)${NC}"

# Kill existing session
tmux kill-session -t $SESSION 2>/dev/null || true

echo ""
echo -e "${GREEN}Starting demo environment...${NC}"
echo ""

# Create tmux session
tmux new-session -d -s $SESSION -x 200 -y 60

# ==========================================
# Window 0: XAI System (Bridge + Navigator)
# ==========================================
tmux rename-window -t $SESSION:0 'XAI_System'

# Pane 0: XAI Bridge (relays Nav2 topics from robot)
tmux send-keys -t $SESSION:0 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:0 "source $SCRIPT_DIR/install/setup.bash" C-m
tmux send-keys -t $SESSION:0 "echo -e '${CYAN}Starting XAI Bridge...${NC}'" C-m
tmux send-keys -t $SESSION:0 "echo 'Connecting to robot rosbridge and relaying Nav2 topics'" C-m
tmux send-keys -t $SESSION:0 "sleep 2" C-m
tmux send-keys -t $SESSION:0 "ros2 run digital_twin_pkg xai_bridge_node --ros-args -p robot_ip:=$ROBOT_IP" C-m

# Split for XAI Navigator
tmux split-window -h -t $SESSION:0 -p 50
tmux send-keys -t $SESSION:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "source $SCRIPT_DIR/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "export GEMINI_API_KEY='$GEMINI_API_KEY'" C-m
tmux send-keys -t $SESSION:0.1 "echo -e '${CYAN}Starting XAI Navigator...${NC}'" C-m
tmux send-keys -t $SESSION:0.1 "echo 'Waiting for XAI Bridge to connect (10s)...'" C-m
tmux send-keys -t $SESSION:0.1 "sleep 10" C-m
tmux send-keys -t $SESSION:0.1 "ros2 launch xai_navigation_pkg xai_navigator.launch.py use_sim_time:=false enable_explanations:=true" C-m

# Split for Behavior Monitor
tmux split-window -v -t $SESSION:0.1 -p 40
tmux send-keys -t $SESSION:0.2 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:0.2 "source $SCRIPT_DIR/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.2 "echo -e '${CYAN}Starting Behavior Monitor...${NC}'" C-m
tmux send-keys -t $SESSION:0.2 "sleep 8" C-m
tmux send-keys -t $SESSION:0.2 "ros2 run digital_twin_pkg behavior_monitor_node" C-m

# ==========================================
# Window 1: Web Services (Backend + Frontend)
# ==========================================
tmux new-window -t $SESSION -n 'Web_Services'

# Pane 0: Local rosbridge for dashboard
tmux send-keys -t $SESSION:1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:1 "echo -e '${CYAN}Starting local rosbridge for dashboard (port 9091)...${NC}'" C-m
tmux send-keys -t $SESSION:1 "ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091" C-m

# Split for Backend
tmux split-window -h -t $SESSION:1 -p 60
tmux send-keys -t $SESSION:1.1 "cd $SCRIPT_DIR/backend" C-m
tmux send-keys -t $SESSION:1.1 "source venv/bin/activate 2>/dev/null || python3 -m venv venv && source venv/bin/activate" C-m
tmux send-keys -t $SESSION:1.1 "echo -e '${CYAN}Starting Backend Server...${NC}'" C-m
tmux send-keys -t $SESSION:1.1 "sleep 3" C-m
tmux send-keys -t $SESSION:1.1 "./run.sh" C-m

# Split for Frontend
tmux split-window -v -t $SESSION:1.1 -p 50
tmux send-keys -t $SESSION:1.2 "cd $SCRIPT_DIR/project" C-m
tmux send-keys -t $SESSION:1.2 "echo -e '${CYAN}Starting Web Dashboard...${NC}'" C-m
tmux send-keys -t $SESSION:1.2 "sleep 5" C-m
tmux send-keys -t $SESSION:1.2 "bun run dev -- --host 2>/dev/null || npm run dev -- --host" C-m

# ==========================================
# Window 2: Control & Monitoring
# ==========================================
tmux new-window -t $SESSION -n 'Control'

# Control pane with instructions
tmux send-keys -t $SESSION:2 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:2 "source $SCRIPT_DIR/install/setup.bash" C-m
tmux send-keys -t $SESSION:2 "clear" C-m
tmux send-keys -t $SESSION:2 "echo ''" C-m
tmux send-keys -t $SESSION:2 "echo -e '${BLUE}╔════════════════════════════════════════════════════════════╗${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo -e '${BLUE}║              NAVIGATION CONTROL PANEL                       ║${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo -e '${BLUE}╚════════════════════════════════════════════════════════════╝${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo ''" C-m
tmux send-keys -t $SESSION:2 "echo -e '${GREEN}Send Navigation Goal:${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo '  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \"'" C-m
tmux send-keys -t $SESSION:2 "echo '    {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}}}\"'" C-m
tmux send-keys -t $SESSION:2 "echo ''" C-m
tmux send-keys -t $SESSION:2 "echo -e '${GREEN}Monitor XAI Explanations:${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo '  ros2 topic echo /navigation/explanation'" C-m
tmux send-keys -t $SESSION:2 "echo ''" C-m
tmux send-keys -t $SESSION:2 "echo -e '${GREEN}Monitor Decisions:${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo '  ros2 topic echo /navigation/decision'" C-m
tmux send-keys -t $SESSION:2 "echo ''" C-m
tmux send-keys -t $SESSION:2 "echo -e '${GREEN}Ask Why Questions:${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo '  ros2 topic pub --once /conversation/user_input std_msgs/String \"data: Why did you stop?\"'" C-m
tmux send-keys -t $SESSION:2 "echo ''" C-m
tmux send-keys -t $SESSION:2 "echo -e '${YELLOW}Dashboard: http://localhost:5173${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo -e '${YELLOW}Backend API: http://localhost:8000/docs${NC}'" C-m
tmux send-keys -t $SESSION:2 "echo ''" C-m
tmux send-keys -t $SESSION:2 "echo 'Press Enter, then type commands...'" C-m

# Split for topic monitoring
tmux split-window -v -t $SESSION:2 -p 50
tmux send-keys -t $SESSION:2.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:2.1 "source $SCRIPT_DIR/install/setup.bash" C-m
tmux send-keys -t $SESSION:2.1 "echo -e '${CYAN}Topic Monitor${NC}'" C-m
tmux send-keys -t $SESSION:2.1 "echo 'Waiting for topics...'" C-m
tmux send-keys -t $SESSION:2.1 "sleep 15" C-m
tmux send-keys -t $SESSION:2.1 "echo 'Available XAI topics:'" C-m
tmux send-keys -t $SESSION:2.1 "ros2 topic list | grep -E '(navigation|anomaly|twin)'" C-m

# Select control window
tmux select-window -t $SESSION:2

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗"
echo -e "║                    Demo Started!                             ║"
echo -e "╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}tmux Windows:${NC}"
echo "  Window 0: XAI_System  - XAI Bridge + Navigator + Behavior Monitor"
echo "  Window 1: Web_Services - rosbridge + Backend + Frontend"
echo "  Window 2: Control     - Command panel + Topic monitor"
echo ""
echo -e "${CYAN}tmux Navigation:${NC}"
echo "  Ctrl+B then 0/1/2 - Switch windows"
echo "  Ctrl+B then D     - Detach (demo keeps running)"
echo "  tmux attach -t $SESSION - Reattach"
echo ""
echo -e "${YELLOW}Dashboard: http://localhost:5173${NC}"
echo -e "${YELLOW}API Docs:  http://localhost:8000/docs${NC}"
echo ""
echo -e "${GREEN}On Robot: Send goals via RViz or voice commands from dashboard${NC}"
echo ""
echo "Attaching to session..."
sleep 2

tmux attach-session -t $SESSION
