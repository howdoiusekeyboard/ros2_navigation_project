#!/bin/bash
#
# Setup ROS2 Network for Cross-Subnet Communication
#
# This script configures CycloneDDS for unicast discovery between
# WSL and TurtleBot3 on different subnets.
#
# Usage: source this script before running any ROS2 commands
#   source scripts/setup_ros_network.sh
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi

# Use CycloneDDS (more reliable for cross-subnet)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set CycloneDDS config for unicast discovery
export CYCLONEDDS_URI="file://$PROJECT_ROOT/config/cyclonedds.xml"

# Common settings
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "ROS2 Network Configuration:"
echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "  CYCLONEDDS_URI: $CYCLONEDDS_URI"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
