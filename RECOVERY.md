# 🚨 Disaster Recovery Guide

**Recovery Time Objective (RTO):** < 10 minutes  
**Recovery Point Objective (RPO):** Last git push  
**Last Updated:** November 12, 2025

---

## 🎯 Quick Recovery (10 Minutes)

This guide gets you from **complete data loss** to **fully functional system** in under 10 minutes.

### Prerequisites

Before starting recovery, ensure you have access to:

- ✅ **GitHub Access:** https://github.com/howdoiusekeyboard/ros2_navigation_project.git
- ✅ **GPG Key Backup:** `~/ros2_gpg_backup.asc` (or from USB/cloud/password manager)
- ✅ **Ubuntu 22.04 / WSL2:** Fresh or existing installation
- ✅ **Internet Connection:** For cloning repository and installing dependencies

### Step-by-Step Recovery

#### 1. Install Core System Dependencies (3 minutes)

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble (if not already installed)
sudo apt install -y ros-humble-desktop ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations ros-humble-cartographer \
  ros-humble-cartographer-ros ros-humble-rosbridge-server

# Install build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  python3-pip python3-venv git-lfs git-crypt

# Install Bun (fast package manager)
curl -fsSL https://bun.sh/install | bash
export PATH="$HOME/.bun/bin:$PATH"

# Verify installations
ros2 --version  # Should show: ros2 cli version: 0.18.x
bun --version   # Should show: 1.x.x
git lfs version # Should show: git-lfs/3.x.x
```

#### 2. Clone Repository (1 minute)

```bash
# Navigate to your workspace directory
cd ~
mkdir -p ~/workspace
cd ~/workspace

# Clone the repository
git clone https://github.com/howdoiusekeyboard/ros2_navigation_project.git
cd ros2_navigation_project

# Verify repository structure
ls -la  # Should see: src/, project/, backend/, RECOVERY.md, etc.
```

#### 3. Unlock Encrypted Files with Git-Crypt (1 minute)

```bash
# Option A: Using GPG key backup file
git-crypt unlock ~/ros2_gpg_backup.asc

# Option B: Using GPG keyring (if key already imported)
git-crypt unlock

# Verify decryption worked
cat backend/.env  # Should show actual API keys, not encrypted gibberish

# If unlock fails, see "Emergency Recovery Without Git-Crypt" section below
```

#### 4. Initialize Git LFS (1 minute)

```bash
# Install Git LFS hooks
git lfs install

# Pull LFS files (PDFs, maps)
git lfs pull

# Verify LFS files downloaded
ls -lh "Final Report"*.pdf  # Should show ~1MB file
ls -lh "Final Presentation"*.pdf  # Should show ~5MB file
```

#### 5. Build ROS2 Workspace (2 minutes)

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Verify build succeeded
ros2 pkg list | grep -E "cartographer_slam|map_server|localization_server|path_planner"
# Should show all 4 custom packages
```

#### 6. Setup Backend Server (1 minute)

```bash
cd backend

# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Verify installation
python -c "import fastapi, openai, google.generativeai, rclpy; print('✅ All imports successful')"

cd ..
```

#### 7. Setup Frontend Dashboard (1 minute)

```bash
cd project

# Install dependencies with Bun (fast!)
bun install

# Verify installation
bun run build  # Should complete in ~2 seconds

cd ..
```

#### 8. Verification Test (1 minute)

```bash
# Quick smoke test - start one ROS2 node
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run turtlesim turtlesim_node &
TURTLE_PID=$!

# Wait 2 seconds
sleep 2

# Test publish to topic
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.5}}"

# Kill test node
kill $TURTLE_PID

echo "✅ Recovery complete! System is operational."
```

---

## 🔄 Full System Startup

After recovery, start the complete system:

### Terminal 1: Simulation

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2: Full Stack (Backend + Frontend + Bridge)

```bash
cd ~/workspace/ros2_navigation_project
./start_robot_dashboard.sh
```

### Access

- **Web Dashboard:** http://localhost:5173
- **Backend API:** http://localhost:8000
- **API Docs:** http://localhost:8000/docs
- **ROS Bridge:** ws://localhost:9090

---

## 🆘 Emergency Recovery Without Git-Crypt

If you've lost your GPG key or can't decrypt .env files:

### 1. Create Backend .env Manually

```bash
cd backend
cp .env.example .env
nano .env
```

Fill in these required values:

```bash
OPENAI_API_KEY=sk-...  # Get from: https://platform.openai.com/api-keys
GEMINI_API_KEY=AIza...  # Get from: https://aistudio.google.com/app/apikey
DATABASE_URL=sqlite:///./robot_voice_control.db
ROS_DOMAIN_ID=0
HOST=0.0.0.0
PORT=8000
DEBUG=True
CORS_ORIGINS=["http://localhost:5173"]
LOG_LEVEL=INFO
```

### 2. Create Frontend .env Manually

```bash
cd ../project
cp .env.example .env
nano .env
```

Fill in:

```bash
VITE_GEMINI_API_KEY=your_key_here  # Optional - prefer backend
VITE_BACKEND_URL=http://localhost:8000
VITE_ROSBRIDGE_URL=ws://localhost:9090
```

### 3. Retrieve Keys from Password Manager

If you stored keys in Bitwarden/1Password/LastPass:
- Search for: "ROS2 Voice Navigation" or "ros2_navigation_project"
- Copy API keys to .env files

---

## 🔍 Troubleshooting Recovery Issues

### Issue: "git-crypt unlock" fails

**Symptom:** `Error: gpg: decryption failed: No secret key`

**Solution:**
```bash
# Import GPG key first
gpg --import ~/ros2_gpg_backup.asc

# Verify key imported
gpg --list-secret-keys

# Try unlock again
git-crypt unlock
```

### Issue: ROS2 packages not found after build

**Symptom:** `Package 'cartographer_slam' not found`

**Solution:**
```bash
# Ensure you're sourcing both ROS2 AND workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verify packages
ros2 pkg list | grep cartographer_slam
```

### Issue: Backend fails to start

**Symptom:** `KeyError: 'OPENAI_API_KEY'` or `ModuleNotFoundError`

**Solution:**
```bash
# Check .env file exists and is decrypted
cat backend/.env  # Should show readable API keys

# Reinstall Python dependencies
cd backend
source venv/bin/activate
pip install --upgrade -r requirements.txt
```

### Issue: Frontend won't compile

**Symptom:** `error: Cannot find module 'roslib'`

**Solution:**
```bash
cd project

# Remove node_modules and reinstall
rm -rf node_modules bun.lock
bun install

# Verify
bun run build
```

### Issue: Git LFS files not downloading

**Symptom:** PDF files are tiny (< 1KB) or show LFS pointer text

**Solution:**
```bash
# Install LFS and pull files
git lfs install
git lfs pull

# Verify
ls -lh *.pdf  # Should show actual file sizes (1MB+)
```

---

## 📦 Restoring from Full Backup Archive

If GitHub is down or you have a complete backup:

```bash
# Extract full backup archive
cd ~
tar -xzf ros2_navigation_FULL_20251112.tar.gz
cd ros2_navigation_project

# Unlock secrets (if encrypted)
git-crypt unlock ~/ros2_gpg_backup.asc

# Source and test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep cartographer_slam  # Should show package

# Skip steps 5-7 since build artifacts are included
# Proceed directly to verification test
```

---

## ✅ Recovery Checklist

Use this checklist to verify complete recovery:

- [ ] Repository cloned from GitHub
- [ ] Git-crypt unlocked (or .env files manually created)
- [ ] Git LFS files downloaded (PDFs present)
- [ ] ROS2 workspace built successfully
- [ ] Backend dependencies installed
- [ ] Frontend dependencies installed
- [ ] All 4 custom ROS2 packages available
- [ ] Backend server starts without errors
- [ ] Frontend builds without errors
- [ ] Turtlesim node launches successfully
- [ ] API documentation accessible at /docs

---

## 🎓 Understanding the Recovery Architecture

### Why This Recovery Works

**Multi-Layer Protection:**

1. **Git (Layer 1):** All source code on GitHub
2. **Git-Crypt (Layer 2):** Encrypted API keys in repo
3. **Git LFS (Layer 3):** Large files efficiently stored
4. **Backup Archive (Layer 4):** Complete state snapshot
5. **Documentation (Layer 5):** Human-readable instructions

**Recovery from Any Scenario:**

| Failure Scenario | Recovery Method | Time |
|------------------|-----------------|------|
| WSL corruption | Clone repo + unlock | 10 min |
| Deleted files | `git reset --hard` | < 1 min |
| Lost API keys | Git-crypt OR password manager | 2 min |
| GitHub down | Backup archive | 15 min |
| Lost GPG key | Manual .env recreation | 5 min |
| Complete laptop loss | Any computer + GitHub | 15 min |

### What Gets Recovered

**Automatically (from Git):**
- ✅ All source code (.py, .ts, .tsx, .cpp)
- ✅ Configuration files (.yaml, .lua, .json)
- ✅ Launch files (.launch.py)
- ✅ Documentation (.md, .pdf via LFS)
- ✅ Scripts (.sh)
- ✅ API keys (.env via git-crypt)

**Rebuilt (2 minutes):**
- 🔄 ROS2 build artifacts (`colcon build`)
- 🔄 Python packages (`pip install`)
- 🔄 Node modules (`bun install`)

**Not Needed:**
- ❌ Database (SQLite created on first run)
- ❌ Logs (ephemeral)
- ❌ Cache files (regenerated)

---

## 📞 Support & Resources

### Documentation
- **Main README:** [README.md](README.md)
- **Setup Guide:** [SETUP.md](SETUP.md)
- **Backup Guide:** [BACKUP_GUIDE.md](BACKUP_GUIDE.md)
- **Dependencies:** [DEPENDENCIES.md](DEPENDENCIES.md)
- **Architecture:** [CLAUDE.md](CLAUDE.md)

### External Resources
- **ROS2 Humble Docs:** https://docs.ros.org/en/humble/
- **Nav2 Documentation:** https://navigation.ros.org/
- **TurtleBot3 Manual:** https://emanual.robotis.com/docs/en/platform/turtlebot3/

### Community
- **ROS2 Answers:** https://answers.ros.org
- **GitHub Issues:** https://github.com/howdoiusekeyboard/ros2_navigation_project/issues

---

## 🔒 Security Considerations

### After Recovery:

1. **Verify API Keys:** Check they haven't been exposed/compromised
2. **Rotate Keys:** Generate new keys if repository was public
3. **Check Git History:** Ensure no sensitive data in commit history
4. **Update Passwords:** Change GitHub password if compromised
5. **Backup GPG Key:** Store in multiple secure locations

### Best Practices:

- ✅ Never commit unencrypted .env files
- ✅ Use git-crypt for sensitive data
- ✅ Store GPG key in password manager
- ✅ Enable 2FA on GitHub account
- ✅ Regular backups (see BACKUP_GUIDE.md)

---

**Recovery Status Tracking:**

| Recovery Date | Time Taken | Issues Encountered | Resolution |
|---------------|------------|-------------------|------------|
| ___________ | _________ | _________________ | __________ |

**Notes:**

---

**🎉 Congratulations! Your system is recovered and operational.**

For ongoing backup procedures, see [BACKUP_GUIDE.md](BACKUP_GUIDE.md).

