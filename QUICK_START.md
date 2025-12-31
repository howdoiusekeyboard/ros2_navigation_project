# Quick Start Guide: Intelligent Digital Twin with XAI

This guide gets you running the complete system with a physical TurtleBot3 Waffle Pi.

## Prerequisites

- **Robot**: TurtleBot3 Waffle Pi running ROS2 Humble
- **Dev Machine**: WSL2 or native Linux with ROS2 Humble
- **Both machines on the same network** (can ping each other)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        YOUR DEV MACHINE (WSL)                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────────────────────┐ │
│  │   Backend    │   │   Frontend   │   │   Digital Twin Stack         │ │
│  │  (FastAPI)   │   │   (React)    │   │                              │ │
│  │  port 8000   │   │  port 5173   │   │  - Gazebo (twin simulation)  │ │
│  │              │   │              │   │  - Real Robot Bridge         │ │
│  │ - OpenAI STT │◄──│◄─Audio Blob  │   │  - Twin Monitor (ML/SHAP)    │ │
│  │ - Gemini     │   │              │   │                              │ │
│  │ - ROS2 Node  │   │  WebSocket───┼───┼─► rosbridge @ robot          │ │
│  └──────────────┘   └──────────────┘   └──────────────────────────────┘ │
│         │                                        │                      │
│         └────────────── ROS2 DDS ────────────────┘                      │
│                           │                                             │
└───────────────────────────┼─────────────────────────────────────────────┘
                            │
                      ╔═════╧═════╗
                      ║  Network  ║
                      ╚═════╤═════╝
                            │
┌───────────────────────────┼─────────────────────────────────────────────┐
│                     TURTLEBOT3 WAFFLE PI                                │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐   ┌──────────────┐                                    │
│  │   Bringup    │   │  rosbridge   │                                    │
│  │              │   │  port 9090   │◄─── WebSocket from dashboard       │
│  │ /odom        │   │              │                                    │
│  │ /scan        │   │              │                                    │
│  │ /cmd_vel     │   │              │                                    │
│  └──────────────┘   └──────────────┘                                    │
└─────────────────────────────────────────────────────────────────────────┘
```

## Step 1: Robot Setup

SSH into your TurtleBot3:

```bash
ssh ubuntu@<ROBOT_IP>  # e.g., ssh ubuntu@10.30.96.171
```

Install rosbridge (if not already):

```bash
sudo apt update
sudo apt install -y ros-humble-rosbridge-server
```

Copy and run the setup script:

```bash
# From your dev machine
scp scripts/robot_setup.sh ubuntu@<ROBOT_IP>:~/

# On the robot
chmod +x robot_setup.sh
WSL_IP=<YOUR_WSL_IP> ./robot_setup.sh
```

Or manually start the services:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=0

# Terminal 1: Robot bringup
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Step 2: Dev Machine Setup

### 2.1 Configure Environment

```bash
cd ~/ros2_navigation_project

# Set your robot's IP
export ROBOT_IP=10.30.96.171

# Create backend .env
cp backend/.env.example backend/.env
# Edit backend/.env:
#   GEMINI_API_KEY=your_gemini_key
#   OPENAI_API_KEY=your_openai_key  # For STT
#   CORS_ORIGINS=["http://localhost:5173","http://YOUR_WSL_IP:5173"]

# Create frontend .env
echo "VITE_ROSBRIDGE_URL=ws://${ROBOT_IP}:9090" > project/.env
```

### 2.2 Install Dependencies

```bash
# Backend
cd backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Frontend
cd ../project
npm install  # or: bun install

# ML dependencies for digital twin
pip3 install scikit-learn shap numpy
```

### 2.3 Build ROS2 Packages

```bash
cd ~/ros2_navigation_project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Step 3: Run the System

Open 3 terminals on your dev machine:

### Terminal 1: Backend

```bash
cd ~/ros2_navigation_project/backend
source venv/bin/activate
./run.sh
# Backend available at http://localhost:8000
# API docs at http://localhost:8000/docs
```

### Terminal 2: Frontend

```bash
cd ~/ros2_navigation_project/project
npm run dev -- --host
# Dashboard available at http://YOUR_WSL_IP:5173
```

### Terminal 3: Digital Twin Stack

```bash
cd ~/ros2_navigation_project
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# Run the twin stack (connects to physical robot)
ROBOT_IP=10.30.96.171 ./scripts/start_twin_stack.sh
```

## Verification

1. **Dashboard**: Open `http://YOUR_WSL_IP:5173` in your browser
2. **ROS Connection**: Should show "Connected" status
3. **Robot Status**: Should display pose from robot
4. **Voice Command**: Click microphone, say "move forward"
5. **Twin Comparison**: Should show live sensor diffs

## Troubleshooting

### "ROS2 topics not visible"

ROS2 DDS multicast doesn't work across subnets. The `start_twin_stack.sh` script automatically configures FastDDS unicast discovery. Make sure:

1. Both machines can ping each other
2. `ROS_DOMAIN_ID=0` on both machines
3. Robot IPs in `config/fastdds_discovery.xml` match your network

### "Backend API shows Failed"

Check CORS configuration in `backend/.env`:

```env
CORS_ORIGINS=["http://localhost:5173","http://YOUR_WSL_IP:5173"]
```

Restart backend after changes.

### "Voice recording not supported"

- Use Chrome or Edge browser
- Grant microphone permissions
- Disable ad blockers

### "Digital twin training mode"

To train the anomaly detection model from scratch:

```bash
ROBOT_IP=10.30.96.171 TRAINING_MODE=true ./scripts/start_twin_stack.sh
```

Wait ~60 seconds for baseline collection, then model trains automatically.

## Key Commands Reference

| What | Command |
|------|---------|
| Check ROS2 topics | `ros2 topic list` |
| Echo robot odometry | `ros2 topic echo /odom` |
| Check backend health | `curl http://localhost:8000/health` |
| Check anomaly score | `ros2 topic echo /anomaly/score` |
| View twin sensor diff | `ros2 topic echo /twin/sensor_diff` |

## File Locations

| Component | Path |
|-----------|------|
| FastDDS config | `config/fastdds_discovery.xml` |
| Backend .env | `backend/.env` |
| Frontend .env | `project/.env` |
| Anomaly model | `~/.ros/digital_twin/anomaly_model.pkl` |
| Navigation DB | `~/.ros/navigation_decisions.db` |
| Backend DB | `~/.ros/backend_data/` |

## Next Steps

- **Training Anomaly Model**: Run with `TRAINING_MODE=true` for 60s
- **Testing Explanations**: Send navigation goals and check `/navigation/explanation`
- **Customize Thresholds**: Edit `src/xai_navigation_pkg/config/xai_params.yaml`
