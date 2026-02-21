#!/bin/bash
# Direct Gazebo launch with weighted XAI demo world

set -e

source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/.gazebo/models

# Launch Gazebo with custom world
gazebo --verbose worlds/weighted_xai_demo.world &
GAZEBO_PID=$!

sleep 8

# Spawn TurtleBot3
ros2 run gazebo_ros spawn_entity.py \
    -entity turtlebot3_burger \
    -file $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
    -x -6.0 -y -6.0 -z 0.01 &

# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(xacro $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf)" &

echo "Weighted XAI Demo World Loaded!"
echo "Person 1 at (2.0, 0.0)"
echo "Person 2 at (-3.0, 4.0)"
echo "Cart at (-2.0, -2.0)"
echo "Chairs at (4.0, 3.0), (4.5, 4.5), (-4.0, -4.0)"

wait
