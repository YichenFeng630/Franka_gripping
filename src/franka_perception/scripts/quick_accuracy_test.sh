#!/bin/bash
# One-stop perception test launcher
# Assumes Gazebo is already running with cubes

set -e

TARGET_COLOR=${1:-red}

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║       Franka Perception - Quick Accuracy Test                 ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Source workspace
cd /opt/ros_ws
source devel/setup.bash

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check roscore
echo -n "Checking ROS master... "
if ! rostopic list &> /dev/null; then
    echo -e "${RED}✗${NC}"
    echo ""
    echo -e "${YELLOW}Please start roscore first:${NC}"
    echo "  roscore"
    exit 1
fi
echo -e "${GREEN}✓${NC}"

# Check Gazebo
echo -n "Checking Gazebo... "
if ! rostopic list 2>/dev/null | grep -q "/gazebo/"; then
    echo -e "${RED}✗${NC}"
    echo ""
    echo -e "${YELLOW}Please start Gazebo first:${NC}"
    echo "  roslaunch franka_zed_gazebo moveit_gazebo_panda.launch"
    exit 1
fi
echo -e "${GREEN}✓${NC}"

# Check cubes
echo -n "Checking cubes in Gazebo... "
if ! rosservice call /gazebo/get_world_properties 2>/dev/null | grep -q "cube_"; then
    echo -e "${RED}✗${NC}"
    echo ""
    echo -e "${YELLOW}No cubes found. They should auto-spawn with Gazebo.${NC}"
    echo "If Gazebo was started without cubes, manually spawn them:"
    echo "  roslaunch franka_zed_gazebo spawn_colored_cubes.launch mode:=test"
    exit 1
fi
echo -e "${GREEN}✓${NC}"
NUM_CUBES=$(rosservice call /gazebo/get_world_properties 2>/dev/null | grep -o "cube_" | wc -l)
echo "  Found $NUM_CUBES cubes in scene"
echo ""

# Kill existing perception if running
if rosnode list 2>/dev/null | grep -q "/perception_node"; then
    echo "Stopping existing perception node..."
    rosnode kill /perception_node 2>/dev/null || true
    sleep 1
fi

# Launch perception
echo "Launching perception node for color: ${BLUE}${TARGET_COLOR}${NC}"
roslaunch franka_perception sim_perception.launch target_color:=$TARGET_COLOR > /tmp/perception_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 3

# Verify perception started
echo -n "Verifying perception node... "
if ! rosnode list 2>/dev/null | grep -q "/perception_node"; then
    echo -e "${RED}✗${NC}"
    echo ""
    echo "Failed to start perception node. Check logs:"
    tail -20 /tmp/perception_launch.log
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi
echo -e "${GREEN}✓${NC}"

# Wait for first data
echo -n "Waiting for perception data... "
sleep 2
echo -e "${GREEN}✓${NC}"
echo ""

# Run test
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                    Running Accuracy Test                      ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

rosrun franka_perception quick_test.py $TARGET_COLOR

EXIT_CODE=$?

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                      Test Complete                            ║"
echo "╚════════════════════════════════════════════════════════════════╝"

# Cleanup
echo ""
read -p "Keep perception running? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Stopping perception node..."
    kill $LAUNCH_PID 2>/dev/null || true
    rosnode kill /perception_node 2>/dev/null || true
    echo "Done!"
else
    echo "Perception node still running. PID: $LAUNCH_PID"
    echo "To stop: kill $LAUNCH_PID"
fi

exit $EXIT_CODE
