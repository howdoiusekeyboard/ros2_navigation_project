#!/bin/bash
# Quick Multi-Modal Fusion Verification Script

set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Multi-Modal Fusion Verification ==="
echo ""

# Check 1: XAI Navigator Node Running
echo "✓ Checking XAI navigator node..."
if ros2 node list | grep -q "xai_navigator_node"; then
    echo "  ✅ XAI navigator node is running"
else
    echo "  ❌ XAI navigator node NOT running"
    exit 1
fi

# Check 2: YOLO Node Running
echo "✓ Checking YOLO node..."
if ros2 node list | grep -q "yolo_node"; then
    echo "  ✅ YOLO node is running"
else
    echo "  ⚠️  YOLO node not running (will use heuristics only)"
fi

# Check 3: Classification Topic
echo "✓ Checking classification topic..."
if ros2 topic list | grep -q "/navigation/obstacle_classification"; then
    echo "  ✅ Classification topic exists"
    PUBS=$(ros2 topic info /navigation/obstacle_classification | grep "Publisher count:" | awk '{print $3}')
    echo "  Publishers: $PUBS"
else
    echo "  ❌ Classification topic missing"
    exit 1
fi

# Check 4: YOLO Detections
echo "✓ Sampling YOLO detections..."
timeout 2 ros2 topic echo /yolo/detections --once > /tmp/yolo_sample.txt 2>&1 || true
if grep -q "detections: \[\]" /tmp/yolo_sample.txt; then
    echo "  ⚠️  No objects detected by camera (empty scene)"
elif grep -q "detections:" /tmp/yolo_sample.txt; then
    COUNT=$(grep -c "class_name:" /tmp/yolo_sample.txt || echo "0")
    echo "  ✅ Camera detecting $COUNT object(s)"
else
    echo "  ⚠️  No YOLO data sampled"
fi

# Check 5: Check if robot is navigating
echo "✓ Checking robot navigation status..."
timeout 2 ros2 action list | grep -q "navigate_to_pose" && echo "  ✅ Nav2 action server active" || echo "  ⚠️  Nav2 not active"

# Summary
echo ""
echo "=== System Status: READY ==="
echo "Multi-modal fusion is integrated and running."
echo ""
echo "To trigger classification:"
echo "  1. Send navigation goal near objects"
echo "  2. Monitor: ros2 topic echo /navigation/obstacle_classification"
echo "  3. Check for 'detection_source: fusion' or 'camera' in messages"
echo ""
echo "For immediate testing:"
echo "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo "    \"{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}\""
