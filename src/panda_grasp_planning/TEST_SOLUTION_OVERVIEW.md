# 测试框架说明

完整的轨迹规划测试框架，包括单版本测试和 V2/V3 对比。

## 快速开始

启动 3 个终端:

终端 1 - 启动仿真:
```bash
cd /opt/ros_ws
source devel/setup.bash
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=true
```

终端 2 - 启动管道 (V3 推荐):
```bash
cd /opt/ros_ws
source devel/setup.bash
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch
```

终端 3 - 运行测试:
```bash
cd /opt/ros_ws/src/panda_grasp_planning
python3 scripts/comprehensive_test.py --version v3 --num-trials 10
```

## 测试工具

### 1. 综合测试 (comprehensive_test.py)

单版本多试验测试:
```bash
python3 scripts/comprehensive_test.py --version v3 --num-trials 10 --output-dir results
```

参数:
- --version: v1, v2, v3 (默认 v3)
- --num-trials: 试验次数 (默认 5)
- --output-dir: 输出目录 (默认 test_results)

输出:
- comprehensive_v3.csv - 原始测试数据
- comprehensive_v3.txt - 统计报告
- comprehensive_v3.json - 结构化数据

### 2. 对比测试 (comparison_test.py)

V2/V3 性能对比:
```bash
python3 scripts/comparison_test.py --num-trials 10 --output-dir results
```

输出:
- comparison_v2_v3.csv - 对比数据
- comparison_v2_v3.txt - 对比分析
- comparison_v2_v3.json - 结构化数据

## 结果查看

查看文本报告:
```bash
cat results/comprehensive_v3.txt
```

查看 CSV 数据:
```bash
head -20 results/comprehensive_v3.csv
```

## 性能指标解释

- Success Rate: 成功完成抓取的比例
- Avg Total Time: 平均总耗时 (HOME 到 HOME)
- Avg Planning Time: 平均规划耗时
- Avg Execution Time: 平均执行耗时
- Candidates Tried: 平均尝试的候选数量
- Retries: 平均重试次数
