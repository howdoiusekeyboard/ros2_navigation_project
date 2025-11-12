# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a complete ROS 2 Humble autonomous navigation system with an integrated React-based web dashboard. The project demonstrates the full navigation pipeline: SLAM mapping → localization → path planning, designed for TurtleBot3 in Gazebo simulation.

## Build System & Common Commands

### ROS 2 Workspace

```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select <package_name>

# Source workspace (required in each terminal)
source install/setup.bash

# Set TurtleBot3 model (required)
export TURTLEBOT3_MODEL=burger
```

### Testing

```bash
# Run tests for all packages
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>

# View test results
colcon test-result --all
```

### Web Dashboard

```bash
# Start complete system (rosbridge + turtlesim + web server)
./start_robot_dashboard.sh

# Manual web dashboard development
cd project
npm install  # or bun install
npm run dev  # starts at http://localhost:5173
```

## Architecture & Key Concepts

### Navigation Pipeline Flow

The system operates in distinct phases that must be understood when modifying code:

1. **SLAM Mapping Phase** (`cartographer_slam`): Creates occupancy grid maps using Google Cartographer. Outputs .pgm/.yaml map files to `map_server/config/`.

2. **Localization Phase** (`localization_server`): Uses AMCL (particle filter) to estimate robot pose against the saved map. Publishes the critical `map → odom` transform.

3. **Planning Phase** (`path_planner_server`): Four coordinated Nav2 servers work together:
   - **Planner Server**: Global path planning using NavfnPlanner (Dijkstra/A*)
   - **Controller Server**: Local trajectory generation using DWB (Dynamic Window Approach)
   - **BT Navigator**: Orchestrates planning/control via behavior trees
   - **Behavior Server**: Handles recovery behaviors (spin, backup, wait)

### Coordinate Frame Hierarchy

Understanding this hierarchy is critical when debugging transforms or adding sensors:

```
map (global, fixed reference)
 └─> odom (published by AMCL, corrects for drift)
      └─> base_footprint / base_link (robot center)
           └─> laser (sensor frame)
```

### Lifecycle Management Pattern

All major nodes use Nav2's `lifecycle_manager` for coordinated startup/shutdown. When adding new nodes to the navigation stack, they must be added to the appropriate lifecycle manager's node list.

### Costmap Architecture

Two costmaps with distinct purposes:
- **Global Costmap** (planner_server.yaml): Large, map-frame, slow update (1Hz), uses static map layer
- **Local Costmap** (controller.yaml): Small rolling window (3m × 3m), odom-frame, fast update (5Hz), no static layer

Both use layered costmaps: static layer → obstacle layer → inflation layer

## Configuration File Relationships

Understanding these relationships is essential when modifying parameters:

- **Launch files** (`.launch.py`) reference parameter files by package path using `get_package_share_directory()`
- **Parameter files** (`.yaml`) must match node names and namespaces exactly
- **Behavior tree XML** path in `bt_navigator.yaml` must be absolute after installation
- **Map files** come in pairs: `.pgm` (image) + `.yaml` (metadata with origin, resolution)
- **setup.py** in each package defines what gets installed (launch files, configs, maps)

## Critical Configuration Notes

### TurtleBot3 Model Consistency
All configurations assume `TURTLEBOT3_MODEL=burger`. Robot footprint parameters (radius: 0.15m, inflation: 0.35m) are tuned for this model. Changing models requires updating costmap parameters in both planner_server.yaml and controller.yaml.

### Simulation Time
All nodes use `use_sim_time: True` for Gazebo. When deploying to real hardware, this must be changed to `False` across all launch files.

### Frame ID Consistency
Some config files reference `base_footprint` while others use `base_link`. TurtleBot3 uses `base_footprint` as the primary robot frame. Ensure consistency when adding new nodes.

### Behavior Tree XML Path
The `default_nav_to_pose_bt_xml` parameter in `bt_navigator.yaml` requires an absolute path post-installation. It's currently passed via launch file, but direct YAML edits will fail if not using the installed path.

## Package Structure & Responsibilities

### cartographer_slam
- **Purpose**: Real-time SLAM mapping
- **Key file**: `config/cartographer.lua` - Cartographer tuning parameters (resolution, scan matching)
- **Launch**: `launch/cartographer.launch.py` - Starts cartographer_node + occupancy_grid_node
- **Output**: Generates maps saved to `map_server/config/`

### map_server
- **Purpose**: Serves static pre-built maps
- **Key file**: `config/turtlebot_area.yaml` - Map metadata (origin: [-4.31, -5.33], resolution: 0.05m)
- **Launch**: `launch/nav2_map_server.launch.py` - Uses lifecycle management for reliable startup

### localization_server
- **Purpose**: AMCL-based robot localization
- **Key file**: `config/amcl_config.yaml` - Particle filter parameters (200-8000 particles, 60 laser beams)
- **Launch**: `launch/localization.launch.py` - Starts both map_server AND amcl together
- **Initial pose**: Default at x=-4.44, y=2.32, yaw=0.33 (configurable via launch args)

### path_planner_server
- **Purpose**: Complete Nav2 navigation stack
- **Key files**:
  - `config/planner_server.yaml` - Global planning + global costmap
  - `config/controller.yaml` - Local planning + local costmap + velocity limits
  - `config/bt_navigator.yaml` - Behavior tree plugin configuration
  - `config/behavior.xml` - Navigation behavior tree logic with recovery sequence
  - `config/recovery.yaml` - Recovery behavior parameters
- **Launch**: `launch/pathplanner.launch.py` - Orchestrates all four Nav2 servers with lifecycle management

### project/ (Web Dashboard)
- **Tech stack**: React 18 + TypeScript + Vite + Tailwind CSS
- **ROS integration**: rosbridge_websocket on port 9090
- **Pages**: RobotDashboard, SpeechProcessing, MemoryManagement, CommandControl, Settings
- **Control topic**: Publishes to `/turtle1/cmd_vel`

## Development Patterns

### Adding a New Navigation Node
1. Add node to appropriate package's `setup.py` entry_points
2. Add node name to lifecycle_manager's `node_names` list in launch file
3. Create parameter file in `config/` directory
4. Load parameters in launch file using `Node` parameter argument
5. Install config file via `data_files` in `setup.py`

### Modifying Costmap Behavior
- **Obstacle detection sensitivity**: Adjust `obstacle_range` and `raytrace_range` in costmap configs
- **Safety margins**: Modify `inflation_radius` in inflation_layer parameters
- **Performance**: Balance `update_frequency` vs computational load

### Changing Path Planning Behavior
- **Path smoothness**: Adjust `tolerance` in NavfnPlanner plugin
- **Speed/agility tradeoff**: Modify `max_vel_x`, `max_vel_theta` in controller.yaml
- **Trajectory evaluation**: Tune DWB critics in controller.yaml (PathAlign, GoalAlign, PathDist, GoalDist weights)
- **Recovery logic**: Edit behavior.xml to change retry sequences, or modify recovery.yaml for behavior parameters

### Map Resolution Consistency
Current system uses 0.05m resolution throughout (cartographer.lua, map YAML files, costmap configs). Changing this requires coordinated updates across all configuration files.

## Running the System

### Full Navigation Stack
Requires 4 terminals (in order):
```bash
# Terminal 1: Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Localization (map_server + AMCL)
ros2 launch localization_server localization.launch.py

# Terminal 3: Path planning (Nav2 stack)
ros2 launch path_planner_server pathplanner.launch.py

# Terminal 4: Visualization
rviz2
# MUST set initial pose with "2D Pose Estimate" tool first
# Then send goals with "Nav2 Goal" tool
```

### SLAM Mapping (to create new maps)
```bash
# Terminal 1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Cartographer
ros2 launch cartographer_slam cartographer.launch.py

# Terminal 3: Teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: RViz for visualization
rviz2
```

### Web Dashboard Demo
```bash
# All-in-one startup script
./start_robot_dashboard.sh
# Opens browser at http://localhost:5173
# Requires rosbridge_server, ros-humble-turtlesim
```

## Important Debugging Notes

### AMCL Localization Issues
- **Symptom**: Robot pose jumps or diverges
- **Solution**: Set better initial pose estimate in RViz using "2D Pose Estimate"
- **Config**: Increase `min_particles` or `max_particles` in amcl_config.yaml
- **Visual check**: View `/particle_cloud` in RViz to see particle spread

### Navigation Failures
- **Symptom**: "No valid path found" or robot gets stuck
- **Global planner**: Check if goal is in free space on global costmap
- **Local planner**: Verify local costmap is not completely occupied (increase `obstacle_range`)
- **Recovery**: Behavior tree attempts 6 retries with costmap clearing, spinning, and waiting

### Transform Errors (TF)
- **Symptom**: "Transform from X to Y does not exist"
- **Check**: Verify AMCL is running (provides map→odom)
- **Check**: Verify simulation provides odom→base_footprint and sensor frames
- **Tool**: `ros2 run tf2_tools view_frames` to generate frame tree PDF

### Performance Issues
- **AMCL**: Reduce `max_particles` if CPU usage too high
- **Costmaps**: Decrease update frequencies or reduce costmap size
- **Cartographer**: Adjust `num_laser_scans` or `num_point_clouds` in cartographer.lua

## Package Dependencies

When adding new functionality, be aware of these dependency patterns:
- All packages depend on: `rclpy`, `launch`, `launch_ros`
- Navigation packages depend on: `nav2_common`, `nav2_bringup`
- SLAM packages depend on: `cartographer_ros`
- Web dashboard depends on: `rosbridge_server` (install: `sudo apt install ros-humble-rosbridge-server`)

Dependencies are declared in `package.xml` and must match versions (currently ROS 2 Humble).
