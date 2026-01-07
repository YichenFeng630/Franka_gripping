#!/bin/bash
# Complete end-to-end perception test with Gazebo
# This will start everything needed for testing

set -e

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║     Complete Perception Test - End to End                     ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

TARGET_COLOR=${1:-red}

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

cd /opt/ros_ws
source devel/setup.bash

echo "This script will:"
echo "  1. Check/start Gazebo (with auto-spawned cubes)"
echo "  2. Launch perception node"
echo "  3. Run accuracy test"
echo ""

# Check if roscore is running
echo -n "Checking ROS master... "
if ! rostopic list &> /dev/null; then
    echo -e "${RED}✗${NC}"
    echo ""
    echo -e "${YELLOW}Starting roscore in background...${NC}"
    roscore > /tmp/roscore.log 2>&1 &
    ROSCORE_PID=$!
    sleep 3
    
    if ! rostopic list &> /dev/null; then
        echo -e "${RED}Failed to start roscore!${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ roscore started${NC}"
else
    echo -e "${GREEN}✓${NC}"
    ROSCORE_PID=""
fi

# Check if Gazebo is running
echo -n "Checking Gazebo... "
if ! rostopic list 2>/dev/null | grep -q "/gazebo/"; then
    echo -e "${YELLOW}Not running${NC}"
    echo ""
    echo "Gazebo needs to be started. Do you want to:"
    echo "  1) Start Gazebo now (will take ~30 seconds)"
    echo "  2) Exit and start manually"
    read -p "Choice (1/2): " -n 1 -r
    echo ""
    
    if [[ $REPLY == "1" ]]; then
        echo "Starting Gazebo (headless mode for better performance)..."
        roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=false > /tmp/gazebo.log 2>&1 &
        GAZEBO_PID=$!
        
        echo -n "Waiting for Gazebo to initialize"
        for i in {1..60}; do
            if rostopic list 2>/dev/null | grep -q "/gazebo/"; then
                echo ""
                echo -e "${GREEN}✓ Gazebo started${NC}"
                break
            fi
            echo -n "."
            sleep 1
        done
        
        if ! rostopic list 2>/dev/null | grep -q "/gazebo/"; then
            echo ""
            echo -e "${RED}Gazebo failed to start in 60 seconds${NC}"
            exit 1
        fi
    else
        echo "Please start Gazebo first:"
        echo "  roslaunch franka_zed_gazebo moveit_gazebo_panda.launch"
        exit 1
    fi
else
    echo -e "${GREEN}✓${NC}"
    GAZEBO_PID=""
fi

# Check cubes
echo -n "Checking cubes in Gazebo... "
if ! rosservice call /gazebo/get_world_properties 2>/dev/null | grep -q "cube_"; then
    echo -e "${YELLOW}Not found${NC}"
    echo ""
    echo "Cubes should auto-spawn with Gazebo. Trying to spawn manually..."
    roslaunch franka_zed_gazebo spawn_colored_cubes.launch mode:=test > /tmp/spawn_cubes.log 2>&1 &
    SPAWN_PID=$!
    sleep 5
    
    if ! rosservice call /gazebo/get_world_properties 2>/dev/null | grep -q "cube_"; then
        echo -e "${RED}Still no cubes found${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Cubes spawned${NC}"
else
    echo -e "${GREEN}✓${NC}"
    SPAWN_PID=""
fi

# Launch perception
echo ""
echo "Launching perception node for ${BLUE}${TARGET_COLOR}${NC} target..."
roslaunch franka_perception sim_perception.launch target_color:=$TARGET_COLOR > /tmp/perception.log 2>&1 &
PERCEPTION_PID=$!
sleep 3

echo -n "Verifying perception node... "
if rosnode list 2>/dev/null | grep -q "/perception_node"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
    echo "Failed to start perception node"
    cat /tmp/perception.log
    exit 1
fi

# Wait for data
echo -n "Waiting for perception to process data"
for i in {1..5}; do
    echo -n "."
    sleep 1
done
echo " ${GREEN}✓${NC}"

# Run the test
echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                   Running Accuracy Test                       ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

rosrun franka_perception quick_test.py $TARGET_COLOR
TEST_RESULT=$?

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                     Test Complete!                            ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Cleanup options
echo "Cleanup options:"
echo "  1) Keep everything running"
echo "  2) Stop perception only"
echo "  3) Stop all (perception + cubes)"
echo "  4) Stop everything (including Gazebo)"
read -p "Choice (1-4): " -n 1 -r
echo ""

case $REPLY in
    2)
        echo "Stopping perception..."
        kill $PERCEPTION_PID 2>/dev/null || true
        rosnode kill /perception_node 2>/dev/null || true
        ;;
    3)
        echo "Stopping perception only..."
        kill $PERCEPTION_PID 2>/dev/null || true
        rosnode kill /perception_node 2>/dev/null || true
        if [ -n "$SPAWN_PID" ]; then
            kill $SPAWN_PID 2>/dev/null || true
            rosnode kill /spawn_colored_cubes 2>/dev/null || true
        fi
        ;;
    4)
        echo "Stopping everything..."
        kill $PERCEPTION_PID 2>/dev/null || true
        kill $SPAWN_PID 2>/dev/null || true
        rosnode kill /perception_node 2>/dev/null || true
        rosnode kill /spawn_colored_cubes 2>/dev/null || true
        if [ -n "$GAZEBO_PID" ]; then
            kill $GAZEBO_PID 2>/dev/null || true
        fi
        if [ -n "$ROSCORE_PID" ]; then
            kill $ROSCORE_PID 2>/dev/null || true
        fi
        ;;
    *)
        echo "Keeping everything running"
        echo ""
        echo "To stop later:"
        echo "  Perception: rosnode kill /perception_node"
        echo "  Cubes: rosnode kill /spawn_colored_cubes"
        ;;
esac

echo ""
echo "Done!"

exit $TEST_RESULT
