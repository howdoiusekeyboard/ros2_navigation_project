#!/bin/bash
# Emergency obstacle avoidance test for demo

source /opt/ros/humble/setup.bash
source ~/ros2_navigation_project/install/setup.bash

echo "🧪 TESTING OBSTACLE AVOIDANCE"
echo "=============================="

# Wait for system to be ready
echo "Waiting 5 seconds for system to stabilize..."
sleep 5

# Set initial pose
echo "Setting initial pose at (-6, -6)..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: -6.0, y: -6.0, z: 0.0},
        orientation: {w: 1.0}
      }
    }
  }"

sleep 2

# Navigate to chair position - robot MUST avoid!
echo ""
echo "🎯 Sending navigation goal to chair position (4.0, 3.0)"
echo "Robot should plan path AROUND chair, not through it!"
echo ""

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'},
    pose: {
      position: {x: 4.0, y: 3.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }"

echo ""
echo "✅ Goal sent! Monitor RViz to see if robot avoids chair."
echo "   - Green path should curve AROUND obstacles"
echo "   - Robot should NOT crash into chair"
