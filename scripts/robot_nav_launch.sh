#!/bin/bash
# Navigation Launch Script for Physical TurtleBot3
# Runs Nav2 stack with your saved map
#
# Usage (on TurtleBot3):
#   ./robot_nav_launch.sh /path/to/your/map.yaml
#
# Prerequisites:
#   1. Robot bringup running: ros2 launch turtlebot3_bringup robot.launch.py
#   2. rosbridge running: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
#   3. Map file created using SLAM

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Default map path (TurtleBot3's pre-built map)
DEFAULT_MAP="/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml"
MAP_FILE="${1:-$DEFAULT_MAP}"

echo -e "${BLUE}========================================"
echo "  TurtleBot3 Navigation Stack"
echo -e "========================================${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle_pi}
echo -e "${GREEN}Using model: $TURTLEBOT3_MODEL${NC}"

# Set ROS2 networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Check map file
if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}Error: Map file not found: $MAP_FILE${NC}"
    echo ""
    echo "Options:"
    echo "1. Create a map using SLAM:"
    echo "   ./slam_real_robot.sh"
    echo ""
    echo "2. Use the default TurtleBot3 world map:"
    echo "   ./robot_nav_launch.sh $DEFAULT_MAP"
    echo ""
    exit 1
fi

echo -e "${GREEN}Using map: $MAP_FILE${NC}"
echo ""

# Check if bringup is running
if ! ros2 topic list | grep -q "/odom"; then
    echo -e "${RED}Error: Robot bringup not running${NC}"
    echo "Please start in another terminal:"
    echo "  ros2 launch turtlebot3_bringup robot.launch.py"
    exit 1
fi

# Check if rosbridge is running
if ! ros2 node list 2>/dev/null | grep -q "rosbridge"; then
    echo -e "${YELLOW}Warning: rosbridge not detected${NC}"
    echo "For WSL dashboard, start rosbridge:"
    echo "  ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
fi

echo ""
echo -e "${YELLOW}Instructions:${NC}"
echo "1. Wait for Nav2 to initialize (takes ~30s)"
echo "2. Open RViz on your development machine for visualization"
echo "3. Set initial pose estimate in RViz (2D Pose Estimate)"
echo "4. Send navigation goals via:"
echo "   - RViz '2D Nav Goal' button"
echo "   - Dashboard voice commands"
echo "   - ros2 topic pub /goal_pose ..."
echo ""
echo -e "${GREEN}Starting Navigation2...${NC}"

# Launch Nav2 with the map
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=false \
    map:=$MAP_FILE
