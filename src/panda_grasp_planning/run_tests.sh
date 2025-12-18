#!/bin/bash
# 完整的测试工作流脚本
# Complete Testing Workflow Script

set -e

echo "=========================================================================="
echo "PANDA 抓取管道 - 完整测试工作流"
echo "=========================================================================="

# 默认参数
TRIALS=${1:-5}
OUTPUT_DIR=${2:-test_results}

echo ""
echo "配置:"
echo "  试验次数: $TRIALS"
echo "  输出目录: $OUTPUT_DIR"
echo ""

# 确保目录存在
mkdir -p "$OUTPUT_DIR"

# 检查ROS环境
echo "检查ROS环境..."
if ! command -v roslaunch &> /dev/null; then
    echo "✗ ROS未安装或未配置"
    exit 1
fi
echo "✓ ROS环境OK"

# 检查Gazebo
echo "检查Gazebo环境..."
if ! command -v gazebo &> /dev/null; then
    echo "✗ Gazebo未安装"
    exit 1
fi
echo "✓ Gazebo环境OK"

echo ""
echo "=========================================================================="
echo "第1步: 启动Gazebo和MoveIt"
echo "=========================================================================="
echo ""
echo "请在另一个终端运行:"
echo "  cd /opt/ros_ws"
echo "  source devel/setup.bash"
echo "  roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=true"
echo ""
echo "和"
echo "  cd /opt/ros_ws"
echo "  source devel/setup.bash"
echo "  roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch"
echo ""
read -p "按Enter继续(确保Gazebo已启动)..."

echo ""
echo "=========================================================================="
echo "第2步: 运行综合性能测试"
echo "=========================================================================="
echo ""

cd /opt/ros_ws/src/panda_grasp_planning

# 测试V3
echo "测试V3管道..."
python3 scripts/comprehensive_test.py \
    --version v3 \
    --num-trials "$TRIALS" \
    --output-dir "$OUTPUT_DIR"

echo ""
echo "✓ 测试完成!"
echo ""
echo "=========================================================================="
echo "第3步: 查看结果"
echo "=========================================================================="
echo ""
echo "测试结果已保存到: $OUTPUT_DIR"
ls -lh "$OUTPUT_DIR"

echo ""
echo "=========================================================================="
echo "可选: 对比测试V2和V3"
echo "=========================================================================="
echo ""
echo "运行对比测试:"
echo "  cd /opt/ros_ws/src/panda_grasp_planning"
echo "  python3 scripts/comparison_test.py --num-trials 10 --output-dir test_results"
echo ""

echo "=========================================================================="
echo "完成!"
echo "=========================================================================="
