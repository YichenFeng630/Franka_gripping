#!/bin/bash
# 快速启动 V3 Pipeline 进行测试

set -e

cd /opt/ros_ws
source devel/setup.bash

echo "=========================================="
echo "   快速启动 V3 Pipeline 测试"
echo "=========================================="
echo ""

# 清理旧进程
echo "[1] 清理旧进程..."
pkill -9 -f "gazebo|roslaunch|grasp_pipeline_node_v3" 2>/dev/null || true
sleep 2

# 启动 Gazebo
echo "[2] 启动 Gazebo 仿真..."
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=true &
sleep 15

# 启动 V3 Pipeline
echo ""
echo "[3] 启动 Grasp Pipeline V3..."
rosrun panda_grasp_planning grasp_pipeline_node_v3.py &
sleep 3

echo ""
echo "=========================================="
echo "✅ 系统启动完成！"
echo "=========================================="
echo ""
echo "在另外两个终端运行："
echo ""
echo "  Terminal 2:"
echo "    cd /opt/ros_ws && source devel/setup.bash"
echo "    rostopic echo /grasp_planning_status"
echo ""
echo "  Terminal 3 (手动发送测试目标):"
echo "    cd /opt/ros_ws && source devel/setup.bash"
echo "    rostopic pub /target_cube_pose geometry_msgs/PoseStamped '"
echo "    {header: {frame_id: \"panda_link0\"},"
echo "     pose: {position: {x: 0.5, y: 0.0, z: 0.12},"
echo "            orientation: {w: 1.0}}}' --once"
echo ""
echo "按 Ctrl+C 停止所有进程"
echo ""

wait
