#!/bin/bash
#
# TurtleBot3 Robot Setup Script
# 
# Copy this file to your TurtleBot3 and run it to:
# 1. Configure ROS2 for cross-network discovery
# 2. Start robot bringup
# 3. Start rosbridge for web dashboard
#
# Usage:
#   scp scripts/robot_setup.sh ubuntu@<robot_ip>:~/
#   ssh ubuntu@<robot_ip>
#   chmod +x robot_setup.sh
#   ./robot_setup.sh
#

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  TurtleBot3 Robot Setup${NC}"
echo -e "${GREEN}========================================${NC}"

# Configuration - UPDATE THESE FOR YOUR NETWORK
WSL_IP="${WSL_IP:-10.30.97.16}"         # Your development machine IP
ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"

echo -e "${YELLOW}Configuration:${NC}"
echo "  Robot IP: $ROBOT_IP"
echo "  WSL/Dev IP: $WSL_IP"
echo ""

# Create FastDDS config for cross-network discovery (ROS2 Humble compatible)
FASTDDS_CONFIG=~/fastdds_discovery.xml
cat > "$FASTDDS_CONFIG" << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>$WSL_IP</address>
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                            <address>$ROBOT_IP</address>
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                            <address>127.0.0.1</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

echo -e "${GREEN}Created FastDDS config at $FASTDDS_CONFIG${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash

# Set environment
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_CONFIG"
export TURTLEBOT3_MODEL=waffle_pi

echo -e "${YELLOW}ROS2 Environment:${NC}"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo ""

# Check if rosbridge is installed
if ! ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
    echo -e "${RED}rosbridge_server not installed!${NC}"
    echo -e "${YELLOW}Install with: sudo apt install ros-humble-rosbridge-server${NC}"
    exit 1
fi

echo -e "${GREEN}Starting robot services...${NC}"
echo ""
echo -e "${YELLOW}This will start:${NC}"
echo "  1. TurtleBot3 bringup (sensors, motors)"
echo "  2. rosbridge WebSocket server (port 9090)"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop.${NC}"
echo ""

# Run both in background, wait for either to exit
ros2 launch turtlebot3_bringup robot.launch.py &
PID1=$!

sleep 3  # Give bringup time to start

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
PID2=$!

echo -e "${GREEN}Services started!${NC}"
echo "  TurtleBot3 bringup PID: $PID1"
echo "  rosbridge PID: $PID2"
echo ""
echo -e "${YELLOW}Web dashboard should connect to: ws://$ROBOT_IP:9090${NC}"
echo ""

# Wait for any process to exit
wait -n $PID1 $PID2
EXIT_CODE=$?

# Kill remaining processes
kill $PID1 $PID2 2>/dev/null || true

exit $EXIT_CODE
