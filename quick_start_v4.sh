#!/bin/bash
# Phase 1S 完成 - V4 生产版本启动脚本
# 替代了旧的 quick_start_v3.sh / start_v3.sh

set -e

cd /opt/ros_ws
source devel/setup.bash

echo "=========================================="
echo "   Panda Grasp Pipeline V4 - 启动"
echo "=========================================="
echo ""

# 清理旧进程
echo "[1] 清理旧进程..."
pkill -9 -f "gazebo|roslaunch|python.*v4_demo" 2>/dev/null || true
sleep 2

# 启动 Gazebo + Panda
echo "[2] 启动 Gazebo + Panda 机器人..."
roslaunch panda_grasp_planning panda_grasp_complete.launch sim:=true rviz:=false &
GAZEBO_PID=$!
sleep 5

echo ""
echo "=========================================="
echo "   V4 Pipeline 已启动"
echo "=========================================="
echo ""
echo "用法: python3 src/panda_grasp_planning/scripts/v4_demo.py [选项]"
echo ""
echo "选项:"
echo "  --trials=N          运行 N 个试验（默认：1）"
echo "  --color=COLOR       指定颜色: red, blue, green, yellow"
echo "  --enable-place      执行放置到分类箱（默认启用）"
echo "  --verbose           详细日志输出"
echo ""
echo "示例:"
echo "  python3 src/panda_grasp_planning/scripts/v4_demo.py --trials=20"
echo "  python3 src/panda_grasp_planning/scripts/v4_demo.py --trials=5 --color=red"
echo ""
echo "查看文档: cat src/panda_grasp_planning/doc/QUICK_START.md"
echo "查看状态: cat src/panda_grasp_planning/doc/PHASE_1S_STATUS.md"
echo ""
echo "等待您的命令..."
echo ""

wait $GAZEBO_PID
