# Complete Dependencies List

**Purpose:** Complete inventory of all system packages, libraries, and tools required to build and run this project.  
**Use Case:** Fresh system setup, disaster recovery, Docker image creation.  
**Last Updated:** November 12, 2025

---

## System Requirements

### Operating System

**Primary (Tested):**
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Windows 11 Pro + WSL2 (Ubuntu 22.04)

**Should Work (Untested):**
- Ubuntu 20.04 LTS (Focal) - may require ROS2 Humble manual install
- Debian 11 (Bullseye)
- Any Linux with ROS2 Humble support

**Not Supported:**
- Windows native (no native ROS2 Humble support)
- macOS (ROS2 support limited)

### Hardware Minimum Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | 4 cores | 8+ cores |
| **RAM** | 8 GB | 16+ GB |
| **Storage** | 20 GB free | 50+ GB free |
| **GPU** | Integrated | NVIDIA (for Gazebo) |
| **Network** | 1 Mbps | 10+ Mbps |

---

## ROS2 Dependencies

### Core ROS2 Installation

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble (Desktop Full)
sudo apt update
sudo apt install -y ros-humble-desktop
```

**Version:** ROS2 Humble Hawksbill  
**Release:** May 2022  
**Support Until:** May 2027 (LTS)

### ROS2 Packages (from apt)

```bash
# Navigation Stack (Nav2)
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-common \
  ros-humble-nav2-controller \
  ros-humble-nav2-core \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-map-server \
  ros-humble-nav2-msgs \
  ros-humble-nav2-planner \
  ros-humble-nav2-recoveries \
  ros-humble-nav2-rviz-plugins \
  ros-humble-nav2-util \
  ros-humble-nav2-waypoint-follower

# SLAM (Cartographer)
sudo apt install -y \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-cartographer-rviz

# TurtleBot3
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-simulations \
  ros-humble-turtlebot3-teleop \
  ros-humble-dynamixel-sdk

# Web Integration
sudo apt install -y \
  ros-humble-rosbridge-server \
  ros-humble-rosbridge-suite

# Common Utilities
sudo apt install -y \
  ros-humble-teleop-twist-keyboard \
  ros-humble-teleop-twist-joy \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro

# Message Packages
sudo apt install -y \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-msgs \
  ros-humble-std-srvs

# Visualization
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins

# Transforms & Utilities
sudo apt install -y \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs
```

### ROS2 Build Tools

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-vcstool \
  python3-rosinstall-generator
```

**Initialize rosdep (one-time):**
```bash
sudo rosdep init
rosdep update
```

---

## Python Dependencies

### System Python Packages

```bash
# Python 3.10+ required
sudo apt install -y \
  python3 \
  python3-pip \
  python3-dev \
  python3-venv \
  python3-setuptools \
  python3-wheel
```

**Version:** Python 3.10.x (comes with Ubuntu 22.04)

### Backend Python Packages (via pip)

**File:** `backend/requirements.txt`

```
# Web Framework
fastapi==0.109.0
uvicorn[standard]==0.27.0
python-multipart==0.0.6
websockets==12.0

# OpenAI (Whisper API)
openai==1.12.0

# Google AI (Gemini)
google-genai==1.52.0

# Database
sqlalchemy==2.0.25
psycopg2-binary==2.9.9
alembic==1.13.1

# Vector Embeddings (for context/memory)
sentence-transformers==2.3.1
numpy==1.26.3

# Utilities
python-dotenv==1.0.1
pydantic==2.5.3
pydantic-settings==2.1.0

# Async support
aiofiles==23.2.1
httpx==0.26.0

# Logging
loguru==0.7.2

# Testing (development)
pytest==7.4.4
pytest-asyncio==0.23.3
```

**Install:**
```bash
cd backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### ROS2 Python Packages (from apt, not pip!)

**Critical:** These MUST be installed via apt, not pip:

```bash
sudo apt install -y \
  python3-rclpy \
  python3-geometry-msgs \
  python3-nav-msgs \
  python3-sensor-msgs \
  python3-tf2-ros
```

**Why not pip:** ROS2 packages are tightly coupled to system ROS installation. Using pip versions will cause runtime errors.

---

## Node.js / Bun Dependencies

### Bun Runtime (Recommended)

```bash
# Install Bun (fast JavaScript/TypeScript runtime)
curl -fsSL https://bun.sh/install | bash

# Add to PATH
export PATH="$HOME/.bun/bin:$PATH"
echo 'export PATH="$HOME/.bun/bin:$PATH"' >> ~/.bashrc

# Verify installation
bun --version  # Should show 1.x.x
```

**Version:** Bun 1.3.1+  
**Why Bun:** 10x faster than npm, native TypeScript support, drop-in replacement for Node.js.

### Frontend Dependencies (via Bun)

**File:** `project/package.json`

```json
{
  "dependencies": {
    "@google/generative-ai": "^0.24.1",
    "lucide-react": "^0.344.0",
    "react": "^18.3.1",
    "react-dom": "^18.3.1",
    "roslib": "^1.4.1"
  },
  "devDependencies": {
    "@eslint/js": "^9.9.1",
    "@types/react": "^18.3.5",
    "@types/react-dom": "^18.3.0",
    "@vitejs/plugin-react": "^4.3.1",
    "autoprefixer": "^10.4.18",
    "eslint": "^9.9.1",
    "eslint-plugin-react-hooks": "^5.1.0-rc.0",
    "eslint-plugin-react-refresh": "^0.4.11",
    "globals": "^15.9.0",
    "postcss": "^8.4.35",
    "tailwindcss": "^3.4.1",
    "typescript": "^5.5.3",
    "typescript-eslint": "^8.3.0",
    "vite": "^5.4.2"
  }
}
```

**Install:**
```bash
cd project
bun install
```

**Alternative (if using npm):**
```bash
npm install
```

---

## System Tools & Utilities

### Build Tools

```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  wget \
  curl \
  ca-certificates \
  gnupg \
  lsb-release
```

### Version Control & Backup Tools

```bash
sudo apt install -y \
  git \
  git-lfs \
  git-crypt
```

**Post-install:**
```bash
# Initialize Git LFS
git lfs install

# Verify installations
git --version       # Should be 2.34+
git lfs version     # Should be 3.x.x
git-crypt --version # Should be 0.7.0+
```

### Compression Tools

```bash
sudo apt install -y \
  tar \
  gzip \
  bzip2 \
  xz-utils \
  zip \
  unzip
```

### Text Editors (optional)

```bash
sudo apt install -y \
  nano \
  vim \
  gedit
```

---

## Simulation & Visualization

### Gazebo Classic 11

**Included with ROS2 Humble Desktop:**
```bash
# Should already be installed with ros-humble-desktop
gazebo --version  # Should show 11.10.x
```

**If needed (standalone install):**
```bash
sudo apt install -y gazebo gazebo-common gazebo-plugin-base libgazebo-dev
```

### X11 / Display Server (for WSL2)

**On Windows (WSL2):**
```bash
# Install X11 apps support
sudo apt install -y x11-apps

# Test X11 forwarding
xclock  # Should open a clock window
```

**Required on Windows:**
- VcXsrv or X410 (X11 server for Windows)
- Or: WSLg (built-in to Windows 11 22H2+)

### Graphics Libraries

```bash
sudo apt install -y \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  mesa-utils \
  libglu1-mesa
```

---

## Optional Dependencies

### Database (PostgreSQL - for production)

```bash
# Install PostgreSQL
sudo apt install -y postgresql postgresql-contrib

# Start service
sudo systemctl start postgresql
sudo systemctl enable postgresql

# Create database
sudo -u postgres createdb robot_voice_control
```

**Default:** Project uses SQLite (no setup needed)  
**Production:** Switch to PostgreSQL for better performance

### Cloud Sync Tools

**rclone (multi-cloud):**
```bash
sudo apt install -y rclone
rclone config  # Interactive setup
```

**Google Drive CLI:**
```bash
pip install gdown
```

**Dropbox:**
```bash
# Install Dropbox uploader script
wget https://raw.githubusercontent.com/andreafabrizi/Dropbox-Uploader/master/dropbox_uploader.sh
chmod +x dropbox_uploader.sh
./dropbox_uploader.sh  # Follow setup
```

### Development Tools

```bash
# Code quality tools
pip install black flake8 mypy isort

# Node/Bun development tools
bun add -d prettier eslint

# ROS2 development tools
sudo apt install -y \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins \
  ros-humble-rqt-robot-plugins
```

---

## API Keys Required

### OpenAI API

**Purpose:** Whisper speech-to-text  
**Get Key:** https://platform.openai.com/api-keys  
**Cost:** ~$0.006 per minute of audio  
**Limit:** 3 requests/minute (free tier), 3500/minute (paid)

### Google Gemini API

**Purpose:** Natural language command parsing  
**Get Key:** https://aistudio.google.com/app/apikey  
**Cost:** Free tier available, ~$0.000375 per command  
**Limit:** 60 requests/minute (free tier)

**Storage:** Both keys stored in encrypted `.env` files via git-crypt

---

## Dependency Size Breakdown

| Category | Size | Notes |
|----------|------|-------|
| **ROS2 Humble (full)** | ~3.5 GB | Desktop + Nav2 + Cartographer |
| **Gazebo 11** | ~500 MB | Included in ROS2 desktop |
| **Python packages** | ~800 MB | Including ML libraries |
| **Node modules** | ~300 MB | React + build tools |
| **Build artifacts** | ~200 MB | colcon build output |
| **Git LFS objects** | ~6 MB | PDFs and maps |
| **Total (fresh install)** | **~5.3 GB** | Full development environment |

---

## Version Compatibility Matrix

| Dependency | Version | Compatible With | Notes |
|-----------|---------|-----------------|-------|
| Ubuntu | 22.04 LTS | ROS2 Humble | Primary target |
| ROS2 | Humble | Ubuntu 22.04 | LTS until 2027 |
| Python | 3.10.x | Ubuntu 22.04 | System default |
| Gazebo | 11.10.x | ROS2 Humble | Bundled version |
| Bun | 1.3.1+ | Linux x64/arm64 | Fast JS runtime |
| Node.js | 18.x+ | Alternative to Bun | If Bun unavailable |
| Git | 2.34+ | Ubuntu 22.04 | For LFS support |
| Git LFS | 3.x.x | Git 2.x | Large file storage |

---

## Docker Alternative (Future)

**Status:** Not yet implemented (Week 5-8)

**Planned Docker image would include:**
- Ubuntu 22.04 base
- ROS2 Humble pre-installed
- All system dependencies
- Pre-built ROS2 workspace
- Backend venv with packages
- Frontend node_modules

**Benefits:**
- One-command setup: `docker-compose up`
- Identical environment everywhere
- No manual dependency installation
- Easy CI/CD integration

**Dockerfile location (future):** `docker/Dockerfile`

---

## Installation Quick Reference

### Complete Fresh Install (Ubuntu 22.04)

```bash
#!/bin/bash
# Complete dependency installation script
# Run on fresh Ubuntu 22.04 system

# 1. Update system
sudo apt update && sudo apt upgrade -y

# 2. Install ROS2 Humble
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop

# 3. Install ROS2 packages
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations \
  ros-humble-rosbridge-server \
  ros-humble-teleop-twist-keyboard

# 4. Install build tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-venv \
  build-essential \
  git \
  git-lfs \
  git-crypt

# 5. Initialize rosdep
sudo rosdep init
rosdep update

# 6. Install Bun
curl -fsSL https://bun.sh/install | bash

# 7. Source ROS2
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc

echo "✅ All dependencies installed! Ready to clone repository."
```

**Estimated time:** 15-20 minutes (depending on internet speed)

---

## Verify Installation

```bash
# Check ROS2
ros2 --version

# Check Python
python3 --version
pip3 --version

# Check Bun
bun --version

# Check Git tools
git --version
git lfs version
git-crypt --version

# Check Gazebo
gazebo --version

# Check ROS2 packages
ros2 pkg list | wc -l  # Should show 300+ packages

# Check build tools
colcon --help
```

**All commands should return version numbers without errors.**

---

## Troubleshooting

### Issue: "ros2: command not found"

**Solution:**
```bash
source /opt/ros/humble/setup.bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

### Issue: "git-lfs: command not found"

**Solution:**
```bash
sudo apt install git-lfs
git lfs install
```

### Issue: Python packages fail to install

**Solution:**
```bash
# Upgrade pip
python3 -m pip install --upgrade pip setuptools wheel

# Retry installation
pip install -r backend/requirements.txt
```

### Issue: Bun not found after install

**Solution:**
```bash
export PATH="$HOME/.bun/bin:$PATH"
echo 'export PATH="$HOME/.bun/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

---

**Dependencies Verified:** ⬜ Yes | ⬜ No | ⬜ Partial  
**Installation Date:** __________________  
**System:** __________________  
**Notes:** __________________

---

**For recovery procedures using these dependencies, see [RECOVERY.md](RECOVERY.md).**

