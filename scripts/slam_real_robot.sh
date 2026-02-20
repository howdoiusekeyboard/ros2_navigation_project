#!/bin/bash
# SLAM Script for Physical TurtleBot3
# Creates a map of your physical environment using slam_toolbox
#
# Usage (on TurtleBot3):
#   ./slam_real_robot.sh
#
# After mapping, save the map:
#   ros2 run nav2_map_server map_saver_cli -f ~/map
#
# Then copy to WSL:
#   scp ubuntu@ROBOT_IP:~/map.* ~/ros2_navigation_project/maps/

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================"
echo "  TurtleBot3 SLAM Mapping (slam_toolbox)"
echo -e "========================================${NC}"

# Check if we're on the robot
if [ ! -f /etc/turtlebot3_model ]; then
    echo -e "${YELLOW}Note: This script is designed to run on TurtleBot3${NC}"
    echo -e "${YELLOW}Make sure you're SSHed into the robot${NC}"
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle_pi}
echo -e "${GREEN}Using model: $TURTLEBOT3_MODEL${NC}"

# Set ROS2 networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo ""
echo -e "${YELLOW}Prerequisites:${NC}"
echo "1. Robot bringup must be running in another terminal:"
echo "   ros2 launch turtlebot3_bringup robot.launch.py"
echo ""
echo "2. After this script starts, use teleop in another terminal:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo -e "${YELLOW}Press Enter to start SLAM...${NC}"
read

# Check if bringup is running
if ! ros2 topic list | grep -q "/odom"; then
    echo -e "${RED}Error: /odom topic not found${NC}"
    echo "Please start robot bringup first:"
    echo "  ros2 launch turtlebot3_bringup robot.launch.py"
    exit 1
fi

echo -e "${GREEN}Starting SLAM Toolbox...${NC}"
echo ""
echo -e "${BLUE}Instructions:${NC}"
echo "1. Use teleop to drive the robot around the room"
echo "2. Move slowly near walls and obstacles"
echo "3. Make sure to close loops (return to areas you've mapped)"
echo "4. Press Ctrl+C to stop SLAM when done"
echo ""
echo -e "${YELLOW}After mapping, save the map with:${NC}"
echo "  ros2 run nav2_map_server map_saver_cli -f ~/my_map"
echo ""

# Launch SLAM using slam_toolbox (turtlebot3_slam package)
ros2 launch turtlebot3_slam slam.launch.py use_sim_time:=false
