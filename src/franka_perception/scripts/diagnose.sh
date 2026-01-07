#!/bin/bash
# Quick diagnostic script to check perception setup

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║          Perception System Diagnostics                        ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

cd /opt/ros_ws
source devel/setup.bash

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check ROS master
echo -n "1. ROS Master... "
if rostopic list &> /dev/null; then
    echo -e "${GREEN}✓ Running${NC}"
else
    echo -e "${RED}✗ Not running${NC}"
    echo "   Please start: roscore"
    exit 1
fi

# Check Gazebo
echo -n "2. Gazebo... "
if rostopic list 2>/dev/null | grep -q "/gazebo/"; then
    echo -e "${GREEN}✓ Running${NC}"
else
    echo -e "${RED}✗ Not running${NC}"
    echo "   Please start: roslaunch franka_zed_gazebo moveit_gazebo_panda.launch"
    exit 1
fi

# Check for cubes
echo -n "3. Test cubes... "
CUBE_COUNT=$(rosservice call /gazebo/get_world_properties 2>/dev/null | grep -o "cube_" | wc -l)
if [ "$CUBE_COUNT" -gt 0 ]; then
    echo -e "${GREEN}✓ Found $CUBE_COUNT cubes${NC}"
else
    echo -e "${YELLOW}⚠ No cubes found${NC}"
    echo "   Spawn cubes: roslaunch franka_zed_gazebo spawn_colored_cubes.launch mode:=test"
fi

# Check ZED2 topics
echo ""
echo "4. ZED2 Camera Topics:"

echo -n "   RGB Image... "
RGB_TOPIC=$(rostopic list 2>/dev/null | grep "zed2.*image_rect_color" | head -1)
if [ -n "$RGB_TOPIC" ]; then
    HZ=$(timeout 2 rostopic hz $RGB_TOPIC 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$HZ" ]; then
        echo -e "${GREEN}✓ Publishing at ${HZ} Hz${NC}"
        echo "      Topic: $RGB_TOPIC"
    else
        echo -e "${YELLOW}⚠ Topic exists but no data${NC}"
        echo "      Topic: $RGB_TOPIC"
    fi
else
    echo -e "${RED}✗ Not found${NC}"
    echo "      Expected: /zed2/zed_node/left/image_rect_color (sim)"
    echo "             or /zed2/zed_node/rgb/image_rect_color (real)"
fi

echo -n "   Point Cloud... "
CLOUD_TOPIC=$(rostopic list 2>/dev/null | grep "zed2.*cloud_registered" | head -1)
if [ -n "$CLOUD_TOPIC" ]; then
    HZ=$(timeout 2 rostopic hz $CLOUD_TOPIC 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$HZ" ]; then
        echo -e "${GREEN}✓ Publishing at ${HZ} Hz${NC}"
        echo "      Topic: $CLOUD_TOPIC"
    else
        echo -e "${YELLOW}⚠ Topic exists but no data${NC}"
        echo "      Topic: $CLOUD_TOPIC"
    fi
else
    echo -e "${RED}✗ Not found${NC}"
    echo "      Expected: /zed2/zed_node/point_cloud/cloud_registered"
fi

# Check perception node
echo ""
echo -n "5. Perception Node... "
if rosnode list 2>/dev/null | grep -q "/perception_node"; then
    echo -e "${GREEN}✓ Running${NC}"
    
    # Check perception outputs
    echo ""
    echo "6. Perception Outputs:"
    
    echo -n "   /detected_objects... "
    if rostopic list 2>/dev/null | grep -q "^/detected_objects$"; then
        MSGS=$(timeout 2 rostopic echo /detected_objects -n 1 2>&1)
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ Publishing${NC}"
        else
            echo -e "${YELLOW}⚠ Topic exists but no messages yet${NC}"
        fi
    else
        echo -e "${RED}✗ Not publishing${NC}"
    fi
    
    echo -n "   /object_pose... "
    if rostopic list 2>/dev/null | grep -q "^/object_pose$"; then
        echo -e "${GREEN}✓ Available${NC}"
    else
        echo -e "${RED}✗ Not available${NC}"
    fi
else
    echo -e "${RED}✗ Not running${NC}"
    echo "   Start with: roslaunch franka_perception sim_perception.launch target_color:=red"
    exit 1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                      Diagnostic Complete                      ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Summary
if [ "$CUBE_COUNT" -gt 0 ] && [ -n "$RGB_TOPIC" ] && [ -n "$CLOUD_TOPIC" ]; then
    echo -e "${GREEN}✓ System is ready for testing!${NC}"
    echo ""
    echo "Run test with:"
    echo "  rosrun franka_perception quick_test.py red"
else
    echo -e "${YELLOW}⚠ System is partially configured${NC}"
    echo ""
    echo "Complete setup:"
    if [ "$CUBE_COUNT" -eq 0 ]; then
        echo "  1. roslaunch franka_zed_gazebo spawn_colored_cubes.launch mode:=test"
    fi
    if [ -z "$RGB_TOPIC" ] || [ -z "$CLOUD_TOPIC" ]; then
        echo "  2. Check camera in Gazebo (should auto-start with moveit_gazebo_panda.launch)"
    fi
fi
