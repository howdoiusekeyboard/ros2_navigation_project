#!/bin/bash
# Real Robot Demo Script
# Usage: ./demo_real_robot.sh
#
# Prerequisites (run on robot first):
#   Terminal 1: ros2 launch turtlebot3_bringup robot.launch.py
#   Terminal 2: ros2 launch rosbridge_server rosbridge_websocket_launch.xml

set -e

ROBOT_IP="${ROBOT_IP:-10.30.96.171}"
SESSION="real_robot_demo"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Real Robot Demo Launcher${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Robot IP: $ROBOT_IP${NC}"
echo ""

# Check for tmux
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}Error: tmux not installed. Run: sudo apt install tmux${NC}"
    exit 1
fi

# Check robot connectivity
echo -e "${YELLOW}Checking robot connectivity...${NC}"
if ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Robot reachable${NC}"
else
    echo -e "${RED}✗ Cannot reach robot at $ROBOT_IP${NC}"
    echo -e "${YELLOW}Make sure robot is running:${NC}"
    echo "  1. SSH to robot: ssh ubuntu@$ROBOT_IP"
    echo "  2. Run: ros2 launch turtlebot3_bringup robot.launch.py"
    echo "  3. Run: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    exit 1
fi

# Check rosbridge on robot
echo -e "${YELLOW}Checking rosbridge on robot...${NC}"
if curl -s "http://$ROBOT_IP:9090" 2>&1 | grep -q "WebSocket"; then
    echo -e "${GREEN}✓ Robot rosbridge running${NC}"
else
    echo -e "${RED}✗ Robot rosbridge not detected${NC}"
    echo "Run on robot: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    exit 1
fi

# Kill existing session
tmux kill-session -t $SESSION 2>/dev/null || true

# Create new session
tmux new-session -d -s $SESSION -x 200 -y 60

# ==========================================
# Window 0: Core Services
# ==========================================
tmux rename-window -t $SESSION:0 'Core'

# Pane 0: WSL Rosbridge
tmux send-keys -t $SESSION:0 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:0 'echo "Starting WSL Rosbridge on port 9091..."' C-m
tmux send-keys -t $SESSION:0 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091' C-m

# Pane 1: Rosbridge Monitor
tmux split-window -v -t $SESSION:0 -p 50
tmux send-keys -t $SESSION:0.1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:0.1 'source ~/ros2_navigation_project/install/setup.bash' C-m
tmux send-keys -t $SESSION:0.1 'echo "Waiting for rosbridge (5s)..."' C-m
tmux send-keys -t $SESSION:0.1 'sleep 5' C-m
tmux send-keys -t $SESSION:0.1 "ros2 launch digital_twin_pkg rosbridge_monitor.launch.py robot_ip:=$ROBOT_IP" C-m

# ==========================================
# Window 1: Backend & Frontend
# ==========================================
tmux new-window -t $SESSION -n 'Web'

# Pane 0: Backend
tmux send-keys -t $SESSION:1 'cd ~/ros2_navigation_project/backend' C-m
tmux send-keys -t $SESSION:1 'source venv/bin/activate' C-m
tmux send-keys -t $SESSION:1 'echo "Starting Backend..."' C-m
tmux send-keys -t $SESSION:1 './run.sh' C-m

# Pane 1: Frontend
tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1.1 'cd ~/ros2_navigation_project/project' C-m
tmux send-keys -t $SESSION:1.1 'echo "Starting Frontend..."' C-m
tmux send-keys -t $SESSION:1.1 'sleep 3' C-m
tmux send-keys -t $SESSION:1.1 'bun run dev -- --host' C-m

# ==========================================
# Window 2: Robot Control
# ==========================================
tmux new-window -t $SESSION -n 'Control'

# Pane 0: Teleop
tmux send-keys -t $SESSION:2 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:2 'echo "=== TELEOP KEYBOARD CONTROLS ==="' C-m
tmux send-keys -t $SESSION:2 'echo "   i - forward    k - stop"' C-m
tmux send-keys -t $SESSION:2 'echo "   j - turn left  l - turn right"' C-m
tmux send-keys -t $SESSION:2 'echo "   , - backward"' C-m
tmux send-keys -t $SESSION:2 'echo ""' C-m
tmux send-keys -t $SESSION:2 'echo "Press Enter to start teleop..."' C-m
tmux send-keys -t $SESSION:2 'read' C-m
tmux send-keys -t $SESSION:2 'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel' C-m

# Pane 1: Topic Monitor
tmux split-window -v -t $SESSION:2 -p 40
tmux send-keys -t $SESSION:2.1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:2.1 'echo "Topic Monitor - run commands here:"' C-m
tmux send-keys -t $SESSION:2.1 'echo "  ros2 topic echo /twin/sensor_diff"' C-m
tmux send-keys -t $SESSION:2.1 'echo "  ros2 topic echo /anomaly/score"' C-m
tmux send-keys -t $SESSION:2.1 'echo "  ros2 topic hz /real/odom"' C-m

# Select Control window
tmux select-window -t $SESSION:2

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Demo Started!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Dashboard:${NC} http://localhost:5173"
echo -e "${YELLOW}Backend:${NC}   http://localhost:8000/docs"
echo ""
echo -e "${YELLOW}Windows:${NC}"
echo "  0: Core     - WSL rosbridge + Monitor"
echo "  1: Web      - Backend + Frontend"
echo "  2: Control  - Teleop + Topic monitor"
echo ""
echo -e "${YELLOW}Tmux shortcuts:${NC}"
echo "  Ctrl+B then 0/1/2  - Switch window"
echo "  Ctrl+B then D      - Detach (keep running)"
echo "  Ctrl+B then [      - Scroll mode (q to exit)"
echo ""
echo "Attaching to session..."
sleep 2

tmux attach-session -t $SESSION
