#!/bin/bash
# Complete test script for perception accuracy
# This script will:
# 1. Launch Gazebo with cubes
# 2. Start perception node
# 3. Run accuracy test
# 4. Display results

set -e

echo "========================================"
echo "Perception Accuracy Test Suite"
echo "========================================"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default target color
TARGET_COLOR=${1:-red}

echo "Target color: $TARGET_COLOR"
echo ""

# Source workspace
cd /opt/ros_ws
source devel/setup.bash

echo "Step 1: Checking if roscore is running..."
if ! rostopic list &> /dev/null; then
    echo -e "${YELLOW}roscore not running. Please start it first:${NC}"
    echo "  roscore"
    exit 1
fi
echo -e "${GREEN}✓ roscore is running${NC}"
echo ""

echo "Step 2: Checking Gazebo..."
if ! rostopic list | grep -q "/gazebo/"; then
    echo -e "${YELLOW}Gazebo not running. Please start it first:${NC}"
    echo "  roslaunch franka_zed_gazebo moveit_gazebo_panda.launch"
    exit 1
fi
echo -e "${GREEN}✓ Gazebo is running${NC}"
echo ""

echo "Step 3: Checking if cubes exist in Gazebo..."
rosservice call /gazebo/get_world_properties | grep -q "cube_" || {
    echo -e "${YELLOW}No cubes found in Gazebo!${NC}"
    echo "Spawn some cubes first using the grasp planning demo"
    exit 1
}
echo -e "${GREEN}✓ Cubes found in Gazebo${NC}"
echo ""

echo "Step 4: Launching perception node..."
# Kill existing perception node if any
rosnode kill /perception_node 2>/dev/null || true
sleep 1

# Launch perception in background
roslaunch franka_perception sim_perception.launch target_color:=$TARGET_COLOR &
PERCEPTION_PID=$!
sleep 3

# Check if perception started
if ! rosnode list | grep -q "/perception_node"; then
    echo -e "${RED}✗ Failed to start perception node${NC}"
    kill $PERCEPTION_PID 2>/dev/null || true
    exit 1
fi
echo -e "${GREEN}✓ Perception node started${NC}"
echo ""

echo "Step 5: Waiting for perception to initialize..."
sleep 2
echo -e "${GREEN}✓ Ready${NC}"
echo ""

echo "========================================" 
echo "Running Accuracy Test (10 seconds)..."
echo "========================================"
echo ""

# Run test
rosrun franka_perception test_perception_accuracy.py $TARGET_COLOR

echo ""
echo "========================================"
echo "Test Complete!"
echo "========================================"

# Cleanup
echo ""
echo "Cleaning up..."
kill $PERCEPTION_PID 2>/dev/null || true
rosnode kill /perception_node 2>/dev/null || true

echo "Done!"
