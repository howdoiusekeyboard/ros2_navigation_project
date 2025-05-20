#!/bin/bash

# Exit on any error
set -e

# ASCII art banner
echo "
 ____   ___  ____    ____   ___ ____   ___ _____
|  _ \ / _ \/ ___|  |  _ \ / _ \___ \ / _ \_   _|
| |_) | | | \___ \  | | | | | | |__) | | | || |
|  _ <| |_| |___) | | |_| | |_| / __/| |_| || |
|_| \_\\___/|____/  |____/ \___/_____|\___/ |_|
"
echo "ROS2-Web Dashboard Launcher"
echo "==========================="
echo

# Define colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
  echo -e "${YELLOW}ROS2 environment not sourced. Sourcing it now...${NC}"
  source /opt/ros/humble/setup.bash
else
  echo -e "${GREEN}ROS2 environment already sourced (${ROS_DISTRO})${NC}"
fi

# Source the workspace
echo -e "${YELLOW}Sourcing the workspace...${NC}"
source install/setup.bash

# Kill any existing rosbridge_server processes
echo -e "${YELLOW}Checking for existing rosbridge processes...${NC}"
if pgrep -f "rosbridge_websocket" > /dev/null; then
  echo -e "${YELLOW}Found existing rosbridge processes. Terminating...${NC}"
  sudo fuser -k 9090/tcp || true
  pkill -f "rosbridge_websocket" || true
  sleep 2
fi

# Kill any existing turtlesim processes
if pgrep -f "turtlesim_node" > /dev/null; then
  echo -e "${YELLOW}Found existing turtlesim processes. Terminating...${NC}"
  pkill -f "turtlesim_node" || true
  sleep 2
fi

# Kill any existing bridge server processes
if pgrep -f "ros_bridge_server" > /dev/null; then
  echo -e "${YELLOW}Found existing bridge server processes. Terminating...${NC}"
  pkill -f "ros_bridge_server" || true
  sleep 2
fi

# Start rosbridge in background
echo -e "${YELLOW}Starting ROS Bridge server...${NC}"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# Wait for rosbridge to start
sleep 2
echo -e "${GREEN}ROS Bridge server started with PID: ${ROSBRIDGE_PID}${NC}"

# Start turtlesim in background
echo -e "${YELLOW}Starting Turtlesim...${NC}"
ros2 run turtlesim turtlesim_node &
TURTLESIM_PID=$!

# Wait for turtlesim to start
sleep 2
echo -e "${GREEN}Turtlesim started with PID: ${TURTLESIM_PID}${NC}"

# No custom bridge node needed; we publish Twist directly from the dashboard

# Start the web application
echo -e "${YELLOW}Starting web application...${NC}"
cd project

# Check if bun is installed, otherwise use npm
if command -v bun &> /dev/null; then
  echo -e "${GREEN}Using bun to run dev server${NC}"
  bun run dev &
  WEB_PID=$!
elif command -v npm &> /dev/null; then
  echo -e "${YELLOW}Bun not found, using npm instead${NC}"
  npm run dev &
  WEB_PID=$!
else
  echo -e "${RED}Neither bun nor npm found. Please install one of them to run the web application.${NC}"
  # Clean up other processes
  kill $TURTLESIM_PID $ROSBRIDGE_PID 2>/dev/null || true
  exit 1
fi

echo -e "${GREEN}Web application started with PID: ${WEB_PID}${NC}"
echo
echo -e "${GREEN}All components started successfully!${NC}"
echo -e "${YELLOW}The web dashboard should be available at: http://localhost:5173${NC}"
echo
echo -e "${YELLOW}Press Ctrl+C to stop all components${NC}"

# Function to cleanup on exit
cleanup() {
  echo -e "\n${YELLOW}Shutting down all components...${NC}"
  kill $WEB_PID $TURTLESIM_PID $ROSBRIDGE_PID 2>/dev/null || true
  echo -e "${GREEN}All components stopped${NC}"
  exit 0
}

# Set up the trap to catch SIGINT (Ctrl+C)
trap cleanup SIGINT

# Wait for user to press Ctrl+C
wait 