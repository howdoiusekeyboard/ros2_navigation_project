# Quick Start: Real Robot Navigation with XAI

This guide explains how to run autonomous navigation with XAI explanations on your **physical TurtleBot3 Waffle Pi**, replicating the full `demo_week4.sh` experience.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        TurtleBot3 (Robot)                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │   Bringup   │  │    Nav2     │  │    rosbridge_server    │  │
│  │  (sensors)  │  │ (navigation)│  │      (port 9090)       │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │ WebSocket
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         WSL (Your PC)                            │
│  ┌───────────────────┐  ┌─────────────────┐  ┌───────────────┐  │
│  │ XAI Bridge Node   │  │  XAI Navigator  │  │   Backend     │  │
│  │ (rosbridge relay) │→ │  (explanations) │→ │   + Dashboard │  │
│  └───────────────────┘  └─────────────────┘  └───────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Prerequisites

### On Your Development PC (WSL)
```bash
# Set your Gemini API key
export GEMINI_API_KEY="your_key_here"

# Know your robot's IP address
export ROBOT_IP="10.30.96.171"  # Change to your robot's IP
```

### On TurtleBot3
```bash
# SSH into robot
ssh ubuntu@10.30.96.171

# Source ROS2
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

---

## Step 1: Create a Map (First Time Only)

If you don't have a map of your environment, create one using SLAM (slam_toolbox):

### On TurtleBot3 (Terminal 1):
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
```

### On TurtleBot3 (Terminal 2):
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_slam slam.launch.py use_sim_time:=false
```

### On TurtleBot3 (Terminal 3):
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive the robot around your room slowly to create the map. Make sure to:
- Move close to walls and obstacles
- Close loops (revisit areas) for better accuracy

### Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_room_map
```

This creates two files: `my_room_map.pgm` (image) and `my_room_map.yaml` (metadata).

### Copy map to WSL:
```bash
scp ubuntu@10.30.96.171:~/my_room_map.* ~/ros2_navigation_project/maps/
```

---

## Step 2: Start Robot Services

### On TurtleBot3 (3 Terminals)

**Terminal 1 - Robot Bringup:**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=0
ros2 launch turtlebot3_bringup robot.launch.py
```

**Terminal 2 - rosbridge Server:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 3 - Navigation Stack:**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# Use your custom map:
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=false \
    map:=/home/ubuntu/my_room_map.yaml

# OR use default TurtleBot3 world map (for testing):
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=false \
    map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml
```

---

## Step 3: Start XAI Demo on WSL

```bash
cd ~/ros2_navigation_project
export GEMINI_API_KEY="your_key_here"
export ROBOT_IP="10.30.96.171"

./demo_real_nav.sh
```

This starts:
- **XAI Bridge**: Relays Nav2 topics from robot
- **XAI Navigator**: Logs decisions and generates Gemini explanations
- **Behavior Monitor**: Anomaly detection
- **Backend Server**: Voice command processing
- **Web Dashboard**: Visualization

---

## Step 4: Use the System

### Open Dashboard
Navigate to: **http://localhost:5173**

### Set Initial Pose
In RViz on the robot (or via rosbridge):
1. Click "2D Pose Estimate"
2. Click on map where robot is located

### Send Navigation Goals

**Option 1: RViz (on robot)**
1. Click "2D Nav Goal"
2. Click destination on map

**Option 2: Dashboard Voice Command**
1. Click microphone
2. Say "Navigate to x 1.0 y 0.5"

**Option 3: Command Line (on WSL)**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

### Monitor XAI Explanations
```bash
ros2 topic echo /navigation/explanation
```

### Ask "Why" Questions
```bash
ros2 topic pub --once /conversation/user_input std_msgs/String "data: 'Why did you stop?'"
```

---

## tmux Navigation

The demo runs in tmux. Here's how to navigate:

| Key Combo | Action |
|-----------|--------|
| `Ctrl+B` then `0` | XAI System window |
| `Ctrl+B` then `1` | Web Services window |
| `Ctrl+B` then `2` | Control window |
| `Ctrl+B` then `D` | Detach (keeps running) |
| `tmux attach -t real_nav_xai` | Reattach |

---

## What Each Component Does

| Component | Purpose |
|-----------|---------|
| **XAI Bridge** | Connects to robot's rosbridge, relays Nav2 topics to WSL |
| **XAI Navigator** | Monitors `/plan`, `/goal_pose`, generates Gemini explanations |
| **Behavior Monitor** | Detects anomalies in velocity, obstacles |
| **Backend** | Processes voice commands, stores conversation memory |
| **Dashboard** | Shows robot position, decision timeline, explanations |

---

## Troubleshooting

### "Cannot reach robot"
```bash
ping 10.30.96.171  # Check connectivity
```

### "rosbridge not running"
On robot:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### "No explanations generating"
```bash
echo $GEMINI_API_KEY  # Verify key is set
ros2 topic echo /navigation/decision  # Check if decisions are being logged
```

### "Nav2 not responding"
On robot:
```bash
ros2 topic list | grep plan  # Should see /plan, /local_plan
ros2 node list | grep nav2  # Should see planner, controller, etc.
```

---

## Comparison: Simulation vs. Real Robot

| Feature | `demo_week4.sh` (Sim) | `demo_real_nav.sh` (Real) |
|---------|----------------------|---------------------------|
| Robot Source | Gazebo | Physical TurtleBot3 |
| Map | Pre-built world | Your custom SLAM map |
| Nav2 Location | WSL | Robot (Raspberry Pi) |
| XAI Location | WSL | WSL (via rosbridge relay) |
| Latency | ~10ms | ~50-100ms (network) |

---

## Files Created

| File | Purpose |
|------|---------|
| `demo_real_nav.sh` | Main demo launcher for WSL |
| `scripts/slam_real_robot.sh` | SLAM helper for map creation |
| `scripts/robot_nav_launch.sh` | Nav2 launcher for robot |
| `src/digital_twin_pkg/.../xai_bridge_node.py` | Nav2 topic relay via rosbridge |
| `maps/` | Directory to store your custom maps |

---

## Next Steps

1. **Create custom map** of your environment using SLAM
2. **Train anomaly model** by running in training mode
3. **Test voice commands** for navigation
4. **Review decision database** at `~/.ros/navigation_decisions.db`
