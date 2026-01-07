#!/bin/bash
# Quick test script for franka_perception package

echo "========================================"
echo "Testing franka_perception package"
echo "========================================"

# Check if roscore is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "✗ roscore is not running"
    echo "Please start roscore first: roscore &"
    exit 1
fi

echo "✓ roscore is running"

# Check if perception node is running
if ! rosnode list | grep -q "perception_node"; then
    echo "✗ perception_node is not running"
    echo ""
    echo "To start perception node, run:"
    echo "  roslaunch franka_perception perception.launch"
    exit 1
fi

echo "✓ perception_node is running"

# Check topics
echo ""
echo "Checking ROS topics..."

if rostopic list | grep -q "/detected_objects"; then
    echo "✓ /detected_objects topic exists"
else
    echo "✗ /detected_objects topic not found"
fi

if rostopic list | grep -q "/object_pose"; then
    echo "✓ /object_pose topic exists"
else
    echo "✗ /object_pose topic not found"
fi

if rostopic list | grep -q "/detection_status"; then
    echo "✓ /detection_status topic exists"
else
    echo "✗ /detection_status topic not found"
fi

# Test detection for 5 seconds
echo ""
echo "Monitoring detection for 5 seconds..."
timeout 5 rostopic echo /detected_objects -n 1 > /tmp/perception_test.txt 2>&1

if [ -s /tmp/perception_test.txt ]; then
    echo "✓ Detection is working!"
    echo ""
    echo "Sample output:"
    head -n 10 /tmp/perception_test.txt
else
    echo "⚠ No detection received in 5 seconds"
    echo "This might be normal if no objects are visible"
fi

echo ""
echo "========================================"
echo "Test complete!"
echo "========================================"

# Cleanup
rm -f /tmp/perception_test.txt
