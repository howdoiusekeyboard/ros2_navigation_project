# ROS 2 Humble Navigation Project (Mapping, Localization, Path Planning)

This project demonstrates a complete navigation pipeline using ROS 2 Humble, including mapping with Cartographer, localization with AMCL, and path planning with the Nav2 stack in a Gazebo simulation (TurtleBot3).

## Packages

This workspace contains the following custom packages:
*   `cartographer_slam`: Configuration and launch files for Cartographer mapping.
*   `map_server`: Stores the saved map and provides a launch file to serve it.
*   `localization_server`: Configuration and launch file for AMCL localization.
*   `path_planner_server`: Configuration (YAMLs, BT XML) and launch file for the Nav2 planning/control stack (Planner, Controller, BT Nav, Behaviors).

## Dependencies

*   **ROS 2 Humble:** Assumes a working desktop installation.
*   **Gazebo:** For simulation.
*   **Cartographer:** `sudo apt install ros-humble-cartographer ros-humble-cartographer-ros`
*   **TurtleBot3 Simulation:** `sudo apt install ros-humble-turtlebot3-simulations` (or `ros-humble-turtlebot3-gazebo`) `ros-humble-teleop-twist-keyboard`
*   **Nav2:** `sudo apt install ros-humble-nav2-bringup`

## Setup & Build

1.  Clone this repository (or extract the ZIP) into a ROS 2 workspace (e.g., `~/ros2_ws/src`).
2.  Navigate to the workspace root (e.g., `cd ~/ros2_ws`).
3.  Install dependencies (see above).
4.  Build the packages: `colcon build --symlink-install`

## Running the Full Stack

**Important:** Source the workspace in each new terminal: `source ~/ros2_ws/install/setup.bash` (replace `~/ros2_ws` if needed). Also, set the TurtleBot3 model: `export TURTLEBOT3_MODEL=burger` (or `waffle`, `waffle_pi`).

1.  **Terminal 1 (Simulation):**
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```
2.  **Terminal 2 (Localization):**
    ```bash
    ros2 launch localization_server localization.launch.py
    ```
3.  **Terminal 3 (Path Planning):**
    ```bash
    ros2 launch path_planner_server pathplanner.launch.py
    ```
4.  **Terminal 4 (RViz):**
    ```bash
    rviz2
    ```
    *   In RViz, configure displays (TF, Map `/map`, LaserScan `/scan`, PoseArray `/particle_cloud`, Path `/plan`). Set Fixed Frame to `map`.
    *   **(Crucial):** Use the "2D Pose Estimate" tool to give AMCL an initial position guess based on the robot's location in Gazebo.
    *   Use the "Nav2 Goal" tool to send navigation goals.

## Configuration Notes

*   **[IMPORTANT - If path not passed via launch]** The path to the behavior tree XML is hardcoded in `path_planner_server/config/bt_navigator.yaml`. You **MUST** edit the `default_nav_to_pose_bt_xml` parameter in this file to match the absolute path on your system after cloning/extracting.
*   The saved map files (`turtlebot_area.pgm`, `turtlebot_area.yaml`) are located in `map_server/config/`.
*   Most Nav2 parameters are configured via YAML files in `localization_server/config/` and `path_planner_server/config/`.
*   `use_sim_time` is set to `True` across configurations for use with Gazebo.