#!/bin/bash
# Restart perception node with updated parameters

echo "Restarting perception node..."

cd /opt/ros_ws
source devel/setup.bash

# Kill existing node
rosnode kill /perception_node 2>/dev/null
sleep 1

# Restart with same color (or use argument)
TARGET_COLOR=${1:-red}

echo "Starting perception for color: $TARGET_COLOR"
roslaunch franka_perception sim_perception.launch target_color:=$TARGET_COLOR
