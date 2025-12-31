#!/bin/bash
#
# Real Robot Monitor - No Gazebo Required
#
# Monitors the physical robot for anomalies without needing a digital twin.
# This is simpler and more reliable than the full twin setup.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Real Robot Monitor${NC}"
echo -e "${GREEN}========================================${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
TRAINING_MODE="${TRAINING_MODE:-false}"

echo -e "${YELLOW}Configuration:${NC}"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  Training mode: $TRAINING_MODE"
echo ""

# Check if real robot topics are available
echo -e "${YELLOW}Checking for real robot topics...${NC}"
FOUND_ODOM=false
FOUND_SCAN=false

for i in {1..5}; do
    if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
        FOUND_ODOM=true
    fi
    if ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
        FOUND_SCAN=true
    fi
    if $FOUND_ODOM && $FOUND_SCAN; then
        break
    fi
    sleep 1
done

if $FOUND_ODOM; then
    echo -e "${GREEN}✓ /odom topic found${NC}"
else
    echo -e "${RED}✗ /odom topic not found${NC}"
fi

if $FOUND_SCAN; then
    echo -e "${GREEN}✓ /scan topic found${NC}"
else
    echo -e "${YELLOW}⚠ /scan topic not found (may have QoS issues)${NC}"
fi

if ! $FOUND_ODOM; then
    echo ""
    echo -e "${RED}ERROR: Real robot not detected.${NC}"
    echo -e "${YELLOW}Make sure the robot is running:${NC}"
    echo "  ssh ubuntu@<robot_ip>"
    echo "  ros2 launch turtlebot3_bringup robot.launch.py"
    exit 1
fi

echo ""
echo -e "${GREEN}Starting Real Robot Monitor...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop.${NC}"
echo ""

ros2 launch digital_twin_pkg real_robot_monitor.launch.py \
    training_mode:="$TRAINING_MODE"
