#!/bin/bash
# 原始V3版本启动脚本

set -e

cd /opt/ros_ws
source devel/setup.bash

echo "=========================================="
echo "   Panda Grasp Planning V3 - 快速启动"
echo "=========================================="
echo ""

# 清理旧进程
echo "[1/3] 清理旧进程..."
pkill -9 -f "gazebo|roslaunch|grasp_pipeline" 2>/dev/null || true
sleep 2

# 启动 Gazebo + Panda
echo "[2/3] 启动 Gazebo + Panda 机器人..."
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=true &
sleep 15

# 启动 V3 Pipeline
echo "[3/3] 启动 Grasp Pipeline V3..."
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch &
sleep 3

echo ""
echo "=========================================="
echo "✅ 系统启动完成！"
echo "=========================================="
echo ""
echo "在另外的终端运行:"
echo ""
echo "【Terminal 2】监控 Pipeline 状态:"
echo "  rostopic echo /grasp_planning_status"
echo ""
echo "【Terminal 3】手动发送测试目标:"
echo "  rostopic pub /target_cube_pose geometry_msgs/PoseStamped '"
echo "  {header: {frame_id: \"panda_link0\"},"
echo "   pose: {position: {x: 0.5, y: 0.0, z: 0.12},"
echo "          orientation: {w: 1.0}}}' --once"
echo ""
echo "【Terminal 4】查看运行结果:"
echo "  tail -f test_results/phase4_grasp_results.csv"
echo ""
echo "按 Ctrl+C 停止所有进程"
echo ""

wait
