#!/bin/bash
# 快速测试脚本 - 自动化轨迹规划测试流程
# Quick Test Script - Automated trajectory planning test workflow

set -e  # 有错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本配置
WORKSPACE="/opt/ros_ws"
PACKAGE_PATH="$WORKSPACE/src/panda_grasp_planning"
TEST_DIR="$PACKAGE_PATH/test_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# 默认参数
VERSION="v3"
NUM_TRIALS=10
MODE="basic"  # basic, detailed, compare

# 解析命令行参数
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -v|--version)
                VERSION="$2"
                shift 2
                ;;
            -n|--num-trials)
                NUM_TRIALS="$2"
                shift 2
                ;;
            -m|--mode)
                MODE="$2"
                shift 2
                ;;
            -h|--help)
                print_help
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                print_help
                exit 1
                ;;
        esac
    done
}

# 打印帮助信息
print_help() {
    cat << EOF
使用方法: $(basename "$0") [选项]

选项:
    -v, --version VERSION       选择管道版本 (v1, v2, v3; 默认: v3)
    -n, --num-trials NUM        试验数量 (默认: 10)
    -m, --mode MODE             测试模式:
                                  - basic: 快速测试 (默认)
                                  - detailed: 详细分析
                                  - compare: 对比多个版本
    -h, --help                  显示此帮助信息

示例:
    # 快速测试 V3 版本，10个试验
    $(basename "$0")
    
    # 测试 V2 版本，20个试验，详细分析
    $(basename "$0") -v v2 -n 20 -m detailed
    
    # 对比三个版本的性能
    $(basename "$0") -m compare

EOF
}

# 打印标题
print_title() {
    local title="$1"
    echo -e "\n${BLUE}╔═════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC} $title"
    echo -e "${BLUE}╚═════════════════════════════════════════════════════════════════╝${NC}\n"
}

# 打印状态信息
print_status() {
    local status="$1"
    echo -e "${GREEN}[✓]${NC} $status"
}

# 打印警告信息
print_warning() {
    local warning="$1"
    echo -e "${YELLOW}[⚠]${NC} $warning"
}

# 打印错误信息
print_error() {
    local error="$1"
    echo -e "${RED}[✗]${NC} $error"
}

# 检查环境
check_environment() {
    print_title "环境检查 (Environment Check)"
    
    # 检查 ROS 环境
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS 环境未初始化"
        print_warning "请先运行: source /opt/ros/\$ROS_DISTRO/setup.bash"
        return 1
    fi
    print_status "ROS 环境: $ROS_DISTRO"
    
    # 检查工作空间
    if [ ! -d "$WORKSPACE" ]; then
        print_error "工作空间不存在: $WORKSPACE"
        return 1
    fi
    print_status "工作空间: $WORKSPACE"
    
    # 检查包
    if [ ! -d "$PACKAGE_PATH" ]; then
        print_error "包不存在: $PACKAGE_PATH"
        return 1
    fi
    print_status "包路径: $PACKAGE_PATH"
    
    # 检查脚本
    if [ ! -f "$PACKAGE_PATH/scripts/test_trajectory_planning.py" ]; then
        print_error "测试脚本不存在"
        return 1
    fi
    print_status "测试脚本存在"
    
    # 检查分析脚本
    if [ ! -f "$PACKAGE_PATH/scripts/analyze_trajectory_results.py" ]; then
        print_error "分析脚本不存在"
        return 1
    fi
    print_status "分析脚本存在"
    
    # 创建测试目录
    mkdir -p "$TEST_DIR"
    print_status "测试目录: $TEST_DIR"
    
    return 0
}

# 检查 ROS 节点
check_ros_nodes() {
    print_title "检查 ROS 节点 (Check ROS Nodes)"
    
    # 检查 roscore
    if ! pgrep -x "roscore" > /dev/null; then
        print_error "roscore 未运行"
        return 1
    fi
    print_status "roscore 正在运行"
    
    # 检查管道节点
    sleep 1
    if ! rostopic list | grep -q "grasp_planning_status"; then
        print_error "管道节点未启动或未发布 /grasp_planning_status"
        print_warning "请先运行: roslaunch panda_grasp_planning grasp_planning_pipeline_$VERSION.launch"
        return 1
    fi
    print_status "管道节点正在运行"
    
    # 检查目标位姿主题
    if ! rostopic list | grep -q "target_cube_pose"; then
        print_error "/target_cube_pose 主题不存在"
        return 1
    fi
    print_status "/target_cube_pose 主题存在"
    
    return 0
}

# 运行单版本测试
run_test() {
    local version="$1"
    local num_trials="$2"
    
    print_title "运行轨迹规划测试 - 版本 $version (Run Trajectory Planning Test)"
    
    echo "参数设置:"
    echo "  版本: $version"
    echo "  试验数: $num_trials"
    echo "  输出目录: $TEST_DIR"
    echo ""
    
    cd "$PACKAGE_PATH"
    
    # 运行测试
    python3 scripts/test_trajectory_planning.py \
        --version "$version" \
        --num-trials "$num_trials" \
        --output-dir "test_results"
    
    print_status "测试完成"
}

# 分析结果
analyze_results() {
    local test_dir="$1"
    
    print_title "分析测试结果 (Analyze Results)"
    
    cd "$PACKAGE_PATH"
    
    # 找到最新的 CSV 文件
    latest_csv=$(ls -t "$test_dir"/trajectory_test_*.csv 2>/dev/null | head -1)
    
    if [ -z "$latest_csv" ]; then
        print_error "未找到测试结果 CSV 文件"
        return 1
    fi
    
    print_status "找到结果文件: $(basename $latest_csv)"
    echo ""
    
    # 运行分析
    python3 scripts/analyze_trajectory_results.py "$latest_csv" --detailed
    
    print_status "分析完成"
    
    return 0
}

# 快速测试模式
run_basic_test() {
    print_title "快速测试模式 (Basic Test Mode)"
    
    if ! check_ros_nodes; then
        print_error "ROS 节点检查失败"
        return 1
    fi
    
    run_test "$VERSION" "$NUM_TRIALS"
    analyze_results "$TEST_DIR"
}

# 详细测试模式
run_detailed_test() {
    print_title "详细测试模式 (Detailed Test Mode)"
    
    if ! check_ros_nodes; then
        print_error "ROS 节点检查失败"
        return 1
    fi
    
    # 运行更多试验
    local detailed_trials=$((NUM_TRIALS * 2))
    run_test "$VERSION" "$detailed_trials"
    
    # 生成详细分析
    echo ""
    echo "生成详细分析报告..."
    cd "$PACKAGE_PATH"
    python3 scripts/analyze_trajectory_results.py \
        "test_results/trajectory_test_*.csv" \
        --detailed --output "test_results/detailed_report_$TIMESTAMP.txt"
    
    print_status "详细分析完成"
}

# 版本对比模式
run_compare_test() {
    print_title "版本对比测试 (Version Comparison Mode)"
    
    local versions=("v1" "v2" "v3")
    local results_dir="test_results/comparison_$TIMESTAMP"
    mkdir -p "$results_dir"
    
    for version in "${versions[@]}"; do
        print_title "测试版本 $version"
        
        echo "手动操作需要:"
        echo "  1. 启动 Gazebo 仿真: roslaunch franka_zed_gazebo moveit_gazebo_panda.launch"
        echo "  2. 启动管道: roslaunch panda_grasp_planning grasp_planning_pipeline_$version.launch"
        echo ""
        
        read -p "按 Enter 继续或 Ctrl+C 退出..."
        
        # 检查节点
        if ! check_ros_nodes; then
            print_warning "版本 $version 的节点检查失败，跳过"
            continue
        fi
        
        # 运行测试
        run_test "$version" "$NUM_TRIALS"
        
        # 复制结果到对比目录
        cp test_results/trajectory_test_*.csv "$results_dir/trajectory_test_${version}_$TIMESTAMP.csv" 2>/dev/null || true
        
        # 停止当前版本
        echo "准备测试下一个版本..."
        sleep 3
    done
    
    # 对比分析
    print_title "对比分析 (Comparison Analysis)"
    cd "$PACKAGE_PATH"
    python3 scripts/analyze_trajectory_results.py \
        "$results_dir/trajectory_test_*.csv" \
        --detailed --output "$results_dir/comparison_report_$TIMESTAMP.txt"
    
    print_status "对比分析完成"
    echo "结果目录: $results_dir"
}

# 生成报告
generate_report() {
    print_title "生成最终报告 (Generate Final Report)"
    
    local report_file="$TEST_DIR/final_report_$TIMESTAMP.txt"
    
    {
        echo "╔════════════════════════════════════════════════════════════════╗"
        echo "║           轨迹规划测试最终报告 (Final Report)                  ║"
        echo "╚════════════════════════════════════════════════════════════════╝"
        echo ""
        echo "测试信息 (Test Information):"
        echo "  时间戳: $TIMESTAMP"
        echo "  版本: $VERSION"
        echo "  试验数: $NUM_TRIALS"
        echo "  模式: $MODE"
        echo ""
        echo "结果文件:"
        ls -lh "$TEST_DIR"/trajectory_test_*.csv 2>/dev/null | tail -1 | awk '{print "  CSV: " $NF}'
        ls -lh "$TEST_DIR"/trajectory_analysis_*.txt 2>/dev/null | tail -1 | awk '{print "  Analysis: " $NF}'
        echo ""
        echo "下一步操作 (Next Steps):"
        echo "  1. 查看详细报告: cat $TEST_DIR/trajectory_analysis_*.txt"
        echo "  2. 导出为 JSON: python3 analyze_trajectory_results.py --json ..."
        echo "  3. 对比多个版本: bash quick_test.sh -m compare"
    } | tee "$report_file"
    
    print_status "报告已生成: $report_file"
}

# 清理日志
cleanup() {
    print_title "清理 (Cleanup)"
    
    echo "清理旧日志文件..."
    find "$TEST_DIR" -name "*.csv" -mtime +30 -delete 2>/dev/null
    find "$TEST_DIR" -name "*.txt" -mtime +30 -delete 2>/dev/null
    
    print_status "清理完成"
}

# 主程序
main() {
    parse_args "$@"
    
    print_title "Panda 轨迹规划测试工具"
    echo "时间: $TIMESTAMP"
    echo "工作空间: $WORKSPACE"
    echo ""
    
    # 环境检查
    if ! check_environment; then
        print_error "环境检查失败"
        exit 1
    fi
    
    # 根据模式运行
    case "$MODE" in
        basic)
            run_basic_test
            ;;
        detailed)
            run_detailed_test
            ;;
        compare)
            run_compare_test
            ;;
        *)
            print_error "未知模式: $MODE"
            exit 1
            ;;
    esac
    
    # 生成报告
    generate_report
    
    # 清理
    cleanup
    
    print_title "测试完成 ✓"
    echo "所有结果保存在: $TEST_DIR"
    echo ""
}

# 运行主程序
main "$@"
