#!/bin/bash
#
# Simple Twin Stack - Two-Step Launch
#
# This is more reliable than the all-in-one approach.
# Step 1: Launch Gazebo with TurtleBot3 (this script)
# Step 2: The monitor nodes connect to both real robot and Gazebo twin
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Simple Twin Stack (Two-Step)${NC}"
echo -e "${GREEN}========================================${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle_pi}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

TRAINING_MODE="${TRAINING_MODE:-false}"

echo -e "${YELLOW}Configuration:${NC}"
echo "  TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  Training mode: $TRAINING_MODE"
echo ""

# Check if real robot topics are available
echo -e "${YELLOW}Checking for real robot topics...${NC}"
if ros2 topic list 2>/dev/null | grep -q "/odom"; then
    echo -e "${GREEN}✓ Real robot /odom detected${NC}"
else
    echo -e "${YELLOW}⚠ Real robot /odom not found. Make sure robot bringup is running.${NC}"
fi

echo ""
echo -e "${GREEN}Step 1: Starting Gazebo with twin TurtleBot3...${NC}"
echo -e "${YELLOW}This will open Gazebo. Wait for it to fully load.${NC}"
echo ""

# Launch Gazebo in background
ros2 launch turtlebot3_gazebo empty_world.launch.py &
GAZEBO_PID=$!

echo "Gazebo PID: $GAZEBO_PID"
echo ""
echo -e "${YELLOW}Waiting 10 seconds for Gazebo to initialize...${NC}"
sleep 10

# Check if twin topics appeared
if ros2 topic list 2>/dev/null | grep -q "/odom"; then
    echo -e "${GREEN}✓ Twin robot /odom detected in Gazebo${NC}"
else
    echo -e "${YELLOW}⚠ Twin robot topics not yet available${NC}"
fi

echo ""
echo -e "${GREEN}Step 2: Starting monitor nodes...${NC}"
echo ""

# Remap Gazebo's default topics to /twin/* namespace
# TurtleBot3 Gazebo publishes to /odom, /scan, /cmd_vel by default
# We need to remap these to /twin/*

# Create remapping nodes
ros2 run topic_tools relay /odom /twin/odom &
RELAY_ODOM_PID=$!

ros2 run topic_tools relay /scan /twin/scan --qos-reliability best_effort &
RELAY_SCAN_PID=$!

echo "Topic relays started (PIDs: $RELAY_ODOM_PID, $RELAY_SCAN_PID)"

sleep 2

# Launch monitor nodes
ros2 launch digital_twin_pkg twin_monitor_only.launch.py \
    training_mode:="$TRAINING_MODE" &
MONITOR_PID=$!

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Twin Stack Running${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Gazebo PID: $GAZEBO_PID"
echo "Monitor PID: $MONITOR_PID"
echo "Relay PIDs: $RELAY_ODOM_PID, $RELAY_SCAN_PID"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all processes.${NC}"
echo ""

# Wait for any to exit
trap "kill $GAZEBO_PID $MONITOR_PID $RELAY_ODOM_PID $RELAY_SCAN_PID 2>/dev/null; exit" INT TERM

wait $GAZEBO_PID $MONITOR_PID
