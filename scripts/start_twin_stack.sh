#!/bin/bash
#
# Start Digital Twin Stack (run on WSL/development machine)
#
# This script:
# 1. Sets up FastDDS for cross-network discovery
# 2. Launches the digital twin Gazebo simulation
# 3. Bridges physical robot topics to /real/* namespace
#
# Prerequisites:
# - Robot running: ros2 launch turtlebot3_bringup robot.launch.py
# - Robot running: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# - Backend running: ./backend/run.sh
# - Dashboard running: cd project && npm run dev --host

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Digital Twin Stack Launcher${NC}"
echo -e "${GREEN}========================================${NC}"

# Configuration
ROBOT_IP="${ROBOT_IP:-10.30.96.171}"
WSL_IP="${WSL_IP:-$(hostname -I | awk '{print $1}')}"
USE_PHYSICAL="${USE_PHYSICAL:-true}"
TRAINING_MODE="${TRAINING_MODE:-false}"

echo -e "${YELLOW}Configuration:${NC}"
echo "  Robot IP: $ROBOT_IP"
echo "  WSL IP: $WSL_IP"
echo "  Physical robot mode: $USE_PHYSICAL"
echo "  Training mode: $TRAINING_MODE"
echo ""

# Update FastDDS config with current IPs
FASTDDS_CONFIG="$PROJECT_ROOT/config/fastdds_discovery.xml"
if [ -f "$FASTDDS_CONFIG" ]; then
    # Backup and update IPs (simple sed replacement)
    sed -i "s|<address>10\.30\.97\.[0-9]*</address>|<address>$WSL_IP</address>|g" "$FASTDDS_CONFIG"
    sed -i "s|<address>10\.30\.96\.[0-9]*</address>|<address>$ROBOT_IP</address>|g" "$FASTDDS_CONFIG"
    echo -e "${GREEN}Updated FastDDS config with current IPs${NC}"
fi

# Set ROS2 environment
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"

# Configure DDS for cross-network discovery
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_CONFIG"
export TURTLEBOT3_MODEL=waffle_pi

echo -e "${YELLOW}ROS2 Environment:${NC}"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "  FastDDS Config: $FASTRTPS_DEFAULT_PROFILES_FILE"
echo ""

# Check if we can ping the robot
echo -e "${YELLOW}Checking robot connectivity...${NC}"
if ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Robot reachable at $ROBOT_IP${NC}"
else
    echo -e "${RED}✗ Cannot reach robot at $ROBOT_IP${NC}"
    echo -e "${YELLOW}Make sure the robot is powered on and connected to the network.${NC}"
    echo -e "${YELLOW}You can set ROBOT_IP environment variable if the IP is different.${NC}"
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Wait for ROS2 topics from robot (with timeout)
echo -e "${YELLOW}Waiting for robot ROS2 topics (10s timeout)...${NC}"
TIMEOUT=10
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if ros2 topic list 2>/dev/null | grep -q "/odom\|/scan"; then
        echo -e "${GREEN}✓ Robot ROS2 topics detected${NC}"
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo -n "."
done
echo ""

if [ $ELAPSED -ge $TIMEOUT ]; then
    echo -e "${YELLOW}⚠ Robot ROS2 topics not detected yet.${NC}"
    echo -e "${YELLOW}This is normal if DDS discovery takes time or robot hasn't started.${NC}"
    echo -e "${YELLOW}The bridge node will keep trying to connect.${NC}"
    echo ""
fi

# Launch the digital twin stack
echo -e "${GREEN}Launching Digital Twin Stack...${NC}"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop.${NC}"
echo ""

ros2 launch digital_twin_pkg dual_robot.launch.py \
    use_physical_real:="$USE_PHYSICAL" \
    training_mode:="$TRAINING_MODE"
