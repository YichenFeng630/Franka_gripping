# Panda Grasp Planning - 测试指南

完整的测试流程文档，包括数据记录和结果可视化。

## 相关文档
- [README.md](../README.md) - 项目概述和快速开始
- [API_SPECIFICATION.md](API_SPECIFICATION.md) - 接口定义和参数配置

---

## 📋 测试流程

**渐进式测试**：Phase 1 → Phase 2 → Phase 3，每个阶段验证通过后再进入下一阶段。

### 总体流程
1. **Phase 1** (5 分钟) - 验证基础规划能力
2. **Phase 2** (10 分钟) - 验证完整抓取流程，记录性能数据
3. **Phase 3** (15 分钟) - 大规模性能评估和统计分析

---

## 🚀 开始测试

### 环境准备（所有 Phase 共用）

**终端 1：启动仿真环境**
```bash
cd /opt/ros_ws
source devel/setup.bash
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=true
```

**说明**：
- 保持此终端运行，后续所有测试都使用这个仿真环境
- `gazebo_gui:=true` - 显示 Gazebo GUI，便于观察机器人运动
- Phase 3 时可以改为 `gazebo_gui:=false` 加速仿真

等待 Gazebo 和 MoveIt 完全启动（约 10-20 秒），看到机器人模型加载完成。

---

## Phase 1: 基础规划测试 ✅

**目标**：验证 MoveIt + RRTConnect 规划器的基础功能

**验证点**：
- ✅ 规划器能成功规划从 home 到目标位姿的轨迹
- ✅ 规划的轨迹能成功执行
- ✅ 机器人能安全返回 home 位置

### 步骤 1：运行 Phase 1 测试

**终端 2：启动测试**
```bash
cd /opt/ros_ws
source devel/setup.bash
roslaunch panda_grasp_planning test_phase1.launch
```

### 预期输出

```
============================================================
PHASE 1 TEST: Basic Motion Planning with RRTConnect
============================================================

[Test 1/3] Planning to target 1...
✓ Planning succeeded! Time: 0.023s
✓ Execution succeeded!

[Test 2/3] Planning to target 2...
✓ Planning succeeded! Time: 0.019s
✓ Execution succeeded!

[Test 3/3] Planning to target 3...
✓ Planning succeeded! Time: 0.021s
✓ Execution succeeded!

============================================================
✓ PHASE 1 TEST COMPLETED SUCCESSFULLY!
============================================================
Average planning time: 0.021s
```

### 步骤 2：验证结果

**✅ Phase 1 通过标准**：
- ✅ 所有 3 次规划都成功（无超时或 IK 失败）
- ✅ 所有 3 次执行都成功（机器人到达目标位置）
- ✅ 平均规划时间 < 0.1s

**如果 Phase 1 失败**，请参考 [故障排查](#故障排查) 部分，解决问题后重新测试。

**✅ Phase 1 成功后，继续 Phase 2**

---

## Phase 2: 完整抓取测试（带数据记录）✅

**目标**：测试完整的抓取流程，记录性能数据

**验证点**：
- ✅ 自动生成 pre-grasp、grasp、lift 三个位姿
- ✅ 执行完整的抓取序列（7 个阶段）
- ✅ 记录每次抓取的详细数据
- ✅ 生成可视化图表分析性能

### 步骤 1：启动抓取 Pipeline

**终端 2：启动 pipeline（复用 Phase 1 的终端）**
```bash
cd /opt/ros_ws
source devel/setup.bash
roslaunch panda_grasp_planning grasp_planning_pipeline.launch
```

**参数说明**：
- 默认参数即可（`use_gripper:=false` 适用于仿真）
- 数据自动保存到 `phase2_grasp_results.csv`

### 步骤 2：自动化测试（推荐）

**终端 3：运行自动测试脚本**
```bash
cd /opt/ros_ws
source devel/setup.bash
rosrun panda_grasp_planning auto_test_phase2.py
```

脚本会自动依次发送 6 个测试目标：
- 正前方目标 (0.5, 0.0, 0.10)
- 右侧目标 (0.4, 0.2, 0.10)
- 左侧偏高目标 (0.6, -0.1, 0.12)
- 边缘目标 (0.3, 0.3, 0.10)
- 远端目标 (0.7, 0.0, 0.15)
- 左前方目标 (0.45, -0.15, 0.11)

**说明**：
- 脚本会自动等待每次抓取完成（约 25 秒）
- 预计总时长：约 2.5 分钟
- 按 `Ctrl+C` 可随时中断测试
- **重要**：Z坐标必须 ≥ 0.10m，否则grasp位置会低于工作空间下界
  - grasp位置 = target_z - grasp_offset_z(0.02m)
  - 例：target_z=0.05 → grasp_z=0.03m ✗ （超出范围）
  - 例：target_z=0.10 → grasp_z=0.08m ✓ （安全）

---

<details>
<summary><b>方法 2：手动发送单个目标（点击展开）</b></summary>

如果需要手动测试单个目标，可以使用以下命令：

**测试 1 - 正前方目标**
```bash
rostopic pub /target_cube_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "panda_link0"},
  pose: {
    position: {x: 0.5, y: 0.0, z: 0.05},
    orientation: {x: 0, y: 0, z: 0, w: 1}
  }
}' -1
```

**测试 2 - 右侧目标**
```bash
rostopic pub /target_cube_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "panda_link0"},
  pose: {
    position: {x: 0.4, y: 0.2, z: 0.05},
    orientation: {x: 0, y: 0, z: 0, w: 1}
  }
}' -1
```

等待机器人完成每次抓取序列（约 20-30 秒）后再发送下一个目标。

</details>

### 预期输出（每次测试）

```
============================================================
STARTING GRASP SEQUENCE
============================================================

[1/5] Moving to HOME configuration
✓ Planning succeeded! Time: 0.034s
✓ Execution succeeded!

[2/5] Opening gripper
✓ Gripper opened successfully

[3/5] Descending to GRASP position (approaching cube)
✓ Planning succeeded! Time: 0.028s
✓ Execution succeeded!

[4/5] Closing gripper to grasp object
✓ Gripper grasp command sent (simulation mode)

[5/5] Lifting object and returning to HOME
✓ Planning succeeded! Time: 0.025s
✓ Execution succeeded!

============================================================
✓✓✓ GRASP PIPELINE COMPLETED SUCCESSFULLY! ✓✓✓
============================================================

============================================================
PHASE 2 STATISTICS
============================================================
Total trials: 1
Successful: 1 (100.0%)
Failed: 0 (0.0%)
Avg total time: 15.23s
============================================================

✓ Results saved to: phase2_grasp_results.csv
```

### 步骤 3：分析结果和生成图表

**完成至少 5 次测试后**，在新终端运行分析：

**终端 4：分析数据**
```bash
cd /opt/ros_ws/src/panda_grasp_planning
source /opt/ros_ws/devel/setup.bash
rosrun panda_grasp_planning analyze_phase2_results.py \
  test_results/phase2_grasp_results.csv \
  --plot
```

**说明**：数据文件自动保存在 `test_results/` 目录中

### 生成的数据文件

#### `phase2_grasp_results.csv`
包含每次抓取的详细数据：

| 字段 | 说明 |
|-----|------|
| `trial` | 试验编号 |
| `timestamp` | 执行时间戳 |
| `target_x/y/z` | 目标位置坐标（米） |
| `success` | 是否成功 (True/False) |
| `total_time` | 总耗时（秒） |
| `avg_planning_time` | 平均规划时间（秒） |
| `avg_execution_time` | 平均执行时间（秒） |
| `num_stages` | 完成的阶段数 |
| `failure_reason` | 失败原因（如果失败） |

### 生成的可视化图表

所有图表保存在 `phase2_plots/` 目录：

#### 1. `success_rate.png` - 成功率饼图
- 显示成功和失败试验的比例
- 绿色：成功，红色：失败

#### 2. `time_distribution.png` - 时间分布直方图
包含 3 个子图：
- **总时间分布**：完整抓取序列的总耗时
- **规划时间分布**：每个阶段的平均规划时间
- **执行时间分布**：每个阶段的平均执行时间

#### 3. `target_positions.png` - 目标位置可视化
- **左图（3D）**：工作空间中的目标位置三维散点图
- **右图（2D）**：X-Y 平面俯视图
- 绿色圆点：成功的抓取
- 红色叉号：失败的抓取

#### 4. `failure_reasons.png` - 失败原因分析
- 水平柱状图显示各类失败原因的数量
- 常见失败原因：
  - `HOME_PLANNING_FAILED` - 无法规划到 home
  - `PRE_GRASP_PLANNING_FAILED` - 无法到达 pre-grasp
  - `GRASP_PLANNING_FAILED` - 无法到达 grasp
  - `LIFT_PLANNING_FAILED` - 无法执行 lift

### 分析报告示例

```
======================================================================
PHASE 2 GRASP PIPELINE ANALYSIS
======================================================================

Dataset: phase2_grasp_results.csv
Total trials: 5
Successful: 4 (80.0%)
Failed: 1 (20.0%)

----------------------------------------------------------------------
SUCCESS METRICS
----------------------------------------------------------------------
Total time per grasp:
  Mean:   18.23s
  Std:    2.14s
  Min:    15.67s
  Max:    21.05s

Average planning time per stage:
  Mean:   0.028s
  Std:    0.006s

Average execution time per stage:
  Mean:   3.45s
  Std:    0.52s

----------------------------------------------------------------------
FAILURE ANALYSIS
----------------------------------------------------------------------
  PRE_GRASP_PLANNING_FAILED: 1 (100.0%)

======================================================================

Generating plots...
  ✓ success_rate.png
  ✓ time_distribution.png
  ✓ target_positions.png
  ✓ failure_reasons.png
✓ Plots saved to: phase2_plots/
```

### 步骤 4：验证结果

**✅ Phase 2 通过标准**：
- ✅ 成功率 ≥ 70%（5 次测试中至少 3-4 次成功）
- ✅ 平均总时间 < 25s
- ✅ 平均规划时间 < 0.1s
- ✅ 数据正确保存到 CSV
- ✅ 图表生成无错误

**监控话题（可选）**：
```bash
# 查看执行状态
rostopic echo /grasp_planning_status

# 查看生成的位姿
rostopic echo /pre_grasp_pose
rostopic echo /grasp_pose
rostopic echo /lift_pose
```

**✅ Phase 2 成功后，继续 Phase 3**

---

## Phase 3: 性能评估测试（带数据记录）✅

**目标**：大规模随机测试，建立性能基线

**验证点**：
- ✅ 在随机工作空间位置进行 100+ 次规划测试
- ✅ 统计成功率、规划时间、路径长度
- ✅ 分析失败模式（IK 失败、碰撞、超时等）
- ✅ 生成性能分析报告和可视化

### 步骤 1：关闭 Phase 2 节点

**终端 2 和终端 3**：按 `Ctrl+C` 停止 Phase 2 的节点

### 步骤 2：（可选）重启仿真环境加速

为了加速 Phase 3 测试，可以关闭 GUI：

**终端 1**：按 `Ctrl+C` 停止当前仿真，然后重启（无 GUI）：
```bash
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=false
```

**说明**：关闭 GUI 可以将测试时间从 15 分钟缩短到 10 分钟左右。

### 步骤 3：运行 Benchmark 评估

**终端 2：启动评估**

**快速测试（10 次）**
```bash
cd /opt/ros_ws
source devel/setup.bash
roslaunch panda_grasp_planning benchmark.launch trials:=10
```

**完整测试（100 次）**
```bash
roslaunch panda_grasp_planning benchmark.launch trials:=100
```

**大规模测试（500 次）**
```bash
roslaunch panda_grasp_planning benchmark.launch trials:=500
```

**参数说明**：
- `trials:=N` - 测试次数（默认 100）
- `output:=filename.csv` - 自定义输出文件名（可选）
- `home_interval:=10` - 每 N 次试验返回 home（默认 10）

### 预期输出

```
============================================================
RRT BENCHMARK: Starting 100 trials
============================================================

Workspace bounds:
  X: [0.20, 0.80] m
  Y: [-0.50, 0.50] m
  Z: [0.05, 0.80] m

Planning configuration:
  Planner: RRTConnect
  Planning time: 10.0s
  Max attempts: 10

------------------------------------------------------------

[Trial 1/100]
Target: (0.542, -0.123, 0.234)
✓ Planning succeeded! Time: 0.034s, Path length: 2.145 rad

[Trial 2/100]
Target: (0.678, 0.301, 0.156)
✓ Planning succeeded! Time: 0.028s, Path length: 2.867 rad

[Trial 3/100]
Target: (0.321, -0.432, 0.589)
✗ Planning failed: IK_FAILED

...

[Trial 10/100] Returning to home...
✓ Returned to home

...

[Trial 100/100]
Target: (0.456, 0.089, 0.412)
✓ Planning succeeded! Time: 0.041s, Path length: 1.923 rad

============================================================
BENCHMARK SUMMARY
============================================================
Total trials: 100
Successful: 87 (87.0%)
Failed: 13 (13.0%)

Total time: 245.6s
Avg time per trial: 2.46s

Planning time: 0.035s (±0.009s)
Path length: 2.234 rad (±0.623)

Failure breakdown:
  IK_FAILED: 8 (61.5%)
Run analyze_results.py to generate detailed analysis.
```

### 步骤 4：分析结果和生成图表

**等待 benchmark 完成后**，在新终端运行分析：

**终端 3：分析数据**

方法1 - 自动寻找最新的结果文件：
```bash
cd /opt/ros_ws/src/panda_grasp_planning
source /opt/ros_ws/devel/setup.bash
rosrun panda_grasp_planning analyze_results.py \
  $(ls -t test_results/benchmark_results_*.csv | head -1) \
  --plot
```

方法2 - 指定具体的文件名：
```bash
cd /opt/ros_ws/src/panda_grasp_planning
source /opt/ros_ws/devel/setup.bash
rosrun panda_grasp_planning analyze_results.py \
  test_results/benchmark_results_20251216_143022.csv \
  --plot
```

**提示**：可以先查看生成的文件：
```bash
ls -lh /opt/ros_ws/src/panda_grasp_planning/test_results/benchmark_results_*.csv
```

### 生成的数据文件

#### `benchmark_results_YYYYMMDD_HHMMSS.csv`
包含每次规划的详细数据（文件名中的时间戳为测试运行时间）：

| 字段 | 说明 |
|-----|------|
| `trial` | 试验编号 |
| `target_x/y/z` | 目标位置坐标（米） |
| `success` | 是否成功 (True/False) |
| `planning_time` | 规划耗时（秒） |
| `path_length` | 路径长度（弧度） |
| `num_waypoints` | 路径点数量 |
| `error_type` | 失败类型（IK_FAILED/TIMEOUT/COLLISION） |

### 生成的可视化图表

所有图表保存在 `plots/` 目录：

#### 1. `success_rate.png` - 成功率饼图
- 显示规划成功和失败的比例
- 绿色：成功，红色：失败

#### 2. `planning_time.png` - 规划时间分布
- 直方图显示成功试验的规划时间分布
- 包含平均值和标准差标注

#### 3. `path_length.png` - 路径长度分布
- 直方图显示成功试验的路径长度（joint space）
- 单位：弧度（rad）

#### 4. `target_positions.png` - 工作空间热力图
- **左图（3D）**：成功/失败位置的三维散点图
- **右图（热力图）**：X-Y 平面的成功率热力图
- 绿色：高成功率区域，红色：低成功率区域

#### 5. `failure_analysis.png` - 失败模式分析
- 柱状图显示各类失败原因的数量和百分比
- 失败类型：
  - `IK_FAILED` - 逆运动学求解失败
  - `TIMEOUT` - 规划超时（>10s）
  - `COLLISION` - 检测到碰撞

### 分析报告示例

```
======================================================================
RRT BENCHMARK ANALYSIS
======================================================================

Dataset: benchmark_results_20251216_143022.csv
Total trials: 100
Successful: 87 (87.0%)
Failed: 13 (13.0%)

----------------------------------------------------------------------
SUCCESS METRICS
----------------------------------------------------------------------
Planning time (s):
  Mean:   0.035
  Std:    0.009
  Min:    0.018
  Max:    0.067
  Median: 0.033

Path length (rad):
  Mean:   2.234
  Std:    0.623
  Min:    1.234
  Max:    4.123
  Median: 2.187

Number of waypoints:
  Mean:   45.3
  Std:    12.7

----------------------------------------------------------------------
FAILURE ANALYSIS
----------------------------------------------------------------------
Total failures: 13

Failure types:
  IK_FAILED: 8 (61.5%)
  TIMEOUT: 3 (23.1%)
  COLLISION: 2 (15.4%)

Failed target positions:
  Target 1: (0.321, -0.432, 0.589) - IK_FAILED
  Target 2: (0.789, 0.456, 0.123) - IK_FAILED
  ...

----------------------------------------------------------------------
WORKSPACE ANALYSIS
----------------------------------------------------------------------
X range: [0.200, 0.800] m - 87.0% success
Y range: [-0.500, 0.500] m - 87.0% success
Z range: [0.050, 0.800] m - 87.0% success

High success zones (>90%):
  X: [0.35, 0.65], Y: [-0.20, 0.20], Z: [0.10, 0.50]

Low success zones (<70%):
✓ Plots saved to: plots/
```

### 步骤 5：验证结果

**✅ Phase 3 通过标准**：
- ✅ 成功率 ≥ 80%
- ✅ 平均规划时间 < 0.05s
- ✅ 规划时间标准差 < 0.02s
- ✅ 失败主要原因是 IK_FAILED（说明目标在可达范围边缘）
- ✅ 数据正确保存到 CSV
- ✅ 所有图表生成无错误

---

## 🎉 测试完成总结

如果所有 Phase 都通过验证，恭喜你！系统已经完整验证：

| 阶段 | 验证内容 | 状态 |
|-----|---------|------|
| Phase 1 | 基础规划能力 | ✅ |
| Phase 2 | 完整抓取流程 + 数据记录 | ✅ |
| Phase 3 | 大规模性能评估 + 统计分析 | ✅ |

**生成的数据文件**：
- `phase2_grasp_results.csv` - Phase 2 抓取测试数据
- `benchmark_results_YYYYMMDD_HHMMSS.csv` - Phase 3 性能评估数据（含时间戳）

**生成的图表**：
- `test_results/phase2_plots/` - Phase 2 分析图表（4 张）
- `test_results/plots/` - Phase 3 分析图表（5 张）

---

## 🔧 性能优化建议

### 成功标准
- ✅ 成功率 ≥ 80%
- ✅ 平均规划时间 < 0.05s
- ✅ 规划时间标准差 < 0.02s
- ✅ 失败主要原因是 IK_FAILED（说明目标在可达范围边缘）
- ✅ 数据正确保存到 CSV
- ✅ 所有图表生成无错误

### 性能优化建议

根据分析结果优化规划器参数（编辑 `config/planning_params.yaml`）：

```yaml
planning:
  # 如果超时失败多，增加规划时间
  planning_time: 15.0  # 默认 10.0
  
  # 如果成功率低，增加尝试次数
  num_planning_attempts: 15  # 默认 10
  
  # 如果路径太长或抖动，调整速度缩放
  max_velocity_scaling_factor: 0.3  # 默认 0.5
  max_acceleration_scaling_factor: 0.3  # 默认 0.5
```

---

## 故障排查

### 问题 1: MoveIt 规划失败

**症状**：
```
✗ Planning failed: TIMEOUT
```

**解决方案**：
1. 检查目标位置是否在工作空间内
2. 增加规划时间：`planning_time: 15.0`
3. 增加尝试次数：`num_planning_attempts: 15`

### 问题 2: 数据文件未生成

**症状**：
```
✗ Error: File not found: phase2_grasp_results.csv
```

**解决方案**：
1. 确认至少完成了一次完整的测试
2. 检查当前工作目录：`pwd`
3. 查找文件：`find /opt/ros_ws -name "*.csv"`
4. 检查 pipeline 节点是否有写入权限

### 问题 3: 图表生成失败

**症状**：
```
✗ Matplotlib not available
```

**解决方案**：
```bash
pip3 install matplotlib
```

### 问题 4: Gazebo 崩溃或卡顿

**解决方案**：
1. 关闭 GUI：`gazebo_gui:=false`
2. 重启仿真：`Ctrl+C` 然后重新启动
3. 检查系统资源：`htop`

### 问题 5: 机器人不动

**检查步骤**：
1. 确认 MoveIt 节点运行：`rosnode list | grep move_group`
2. 检查话题连接：`rostopic info /target_cube_pose`
3. 查看日志：`rqt_console`
4. 手动发布测试消息验证话题连接

### 问题 6: grasp_pose_generator 节点启动失败

**症状**：
```
ModuleNotFoundError: No module named 'panda_grasp_planning'
[grasp_pose_generator-1] process has died
```

**原因**：脚本中有错误的导入语句

**解决方案**：
已修复。如果仍有问题，请重新编译：
```bash
cd /opt/ros_ws
catkin_make
source devel/setup.bash
```

### 问题 7: analyze_results.py 参数错误

**症状**：
```
error: unrecognized arguments: --csv
```

**解决方案**：
正确命令格式（CSV 文件名是位置参数，不是 `--csv`）：
```bash
# ✗ 错误
rosrun panda_grasp_planning analyze_results.py --csv benchmark_results.csv

# ✓ 正确
rosrun panda_grasp_planning analyze_results.py benchmark_results.csv --plot
```

---

## 附录

### 常用命令速查

```bash
# 查看所有节点
rosnode list

# 查看所有话题
rostopic list

# 查看话题详情
rostopic info /target_cube_pose

# 查看话题消息
rostopic echo /grasp_planning_status

# 杀死所有 ROS 节点
killall -9 roscore rosmaster rosout

# 清理 ROS 日志
rosclean purge

# 检查文件
ls -la phase2_grasp_results.csv
ls -la phase2_plots/

# 查看 CSV 内容
head -n 20 phase2_grasp_results.csv
```

### 工作空间坐标参考

```
panda_link0 (base frame)
    +Z (上)
     |
     |
     o------> +X (前)
    /
   /
 +Y (左)

典型工作空间范围：
  X: 0.2 ~ 0.8 m
  Y: -0.5 ~ 0.5 m
  Z: 0.05 ~ 0.8 m
```

### 联系方式

如有问题，请联系：
- **邮箱**: yichen.feng@stud.tu-darmstadt.de
- **仓库**: https://github.com/YichenFeng630/Franka_gripping

---

**文档版本**: 1.0  
**最后更新**: 2025-12-16
