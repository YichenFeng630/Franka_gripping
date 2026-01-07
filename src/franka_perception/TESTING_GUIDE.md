# Perception Accuracy Test Guide

## 测试目的
验证franka_perception包的视觉定位精度，通过与Gazebo中物体的真实位置对比。

## 测试脚本

### 1. quick_test.py - 快速单次测试
最简单的测试方式，只需一个命令即可对比一次检测结果。

**用法：**
```bash
# 测试红色方块（默认）
rosrun franka_perception quick_test.py

# 测试其他颜色
rosrun franka_perception quick_test.py blue
rosrun franka_perception quick_test.py green
rosrun franka_perception quick_test.py yellow
```

**输出示例：**
```
======================================================================
Quick Perception Accuracy Test
======================================================================
Target color: red

Reading Gazebo ground truth...
Found 4 cubes in Gazebo:
  cube_RED_1           (red   ): x= 0.500, y= 0.200, z= 0.045
  cube_BLUE_1          (blue  ): x= 0.600, y=-0.100, z= 0.045
  ...

Perception detected objects:
  Object 1 (red   ): x= 0.502, y= 0.198, z= 0.046 (conf: 0.95)

======================================================================
Comparison for RED cube
======================================================================
Matched Gazebo cube: cube_RED_1

Gazebo position:     x= 0.5000, y= 0.2000, z= 0.0450
Detected position:   x= 0.5020, y= 0.1980, z= 0.0460

Position Error:
  ΔX:   +2.00 mm
  ΔY:   -2.00 mm
  ΔZ:   +1.00 mm

  2D Error (XY):   2.83 mm
  3D Error:        3.00 mm
======================================================================

✓ EXCELLENT: 2D accuracy < 10mm
✓ EXCELLENT: 3D accuracy < 20mm
```

### 2. test_perception_accuracy.py - 持续精度测试
进行10秒钟的持续测试，统计多次检测的平均误差和标准差。

**用法：**
```bash
rosrun franka_perception test_perception_accuracy.py red
```

**输出包括：**
- 每次检测的误差
- 统计信息（均值、标准差、最大值）
- 性能评估

### 3. run_accuracy_test.sh - 完整自动化测试
自动检查环境、启动perception、运行测试、生成报告。

**用法：**
```bash
/opt/ros_ws/src/franka_perception/scripts/run_accuracy_test.sh red
```

## 完整测试流程

### 前置条件
1. ROS master必须运行
2. Gazebo仿真环境必须启动
3. 场景中必须有cubes

### 步骤1: 启动Gazebo仿真环境

**终端1 - 启动Gazebo + MoveIt + 自动生成cubes:**
```bash
cd /opt/ros_ws
source devel/setup.bash

# 标准模式（带GUI）
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch

# 无GUI模式（节省资源，推荐用于测试）
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=false
```

等待Gazebo和MoveIt完全启动（约10-15秒）。
**注意：** 现在会自动生成4个彩色测试方块（红、蓝、绿、黄），无需手动操作！

**可选参数：**
```bash
# 无GUI + 不自动生成cubes
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=false spawn_cubes:=false

# 无GUI + 随机模式的cubes
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=false cube_mode:=random num_cubes:=8
```

### 步骤2: 验证cubes已生成（可选）

```bash
# 检查场景中的cubes
rosservice call /gazebo/get_world_properties | grep cube_
```

你应该看到：`cube_RED_1`, `cube_BLUE_2`, `cube_GREEN_3`, `cube_YELLOW_4`

如需重新生成cubes：
```bash
# 单独启动spawn脚本会先删除旧cubes
roslaunch franka_zed_gazebo spawn_colored_cubes.launch mode:=test
```

### 步骤3: 启动Perception节点

**终端2 - 启动perception:**
```bash
cd /opt/ros_ws
source devel/setup.bash
roslaunch franka_perception sim_perception.launch target_color:=red
```

你应该看到：
```
[INFO] Perception Node Initialized
[INFO] Mode: SIMULATION
[INFO] Target Color: red
[INFO] Detection Rate: 10.0 Hz
```

### 步骤4: 运行测试

**终端3 - 运行测试:**
```bash
cd /opt/ros_ws
source devel/setup.bash

# 快速测试
rosrun franka_perception quick_test.py red

# 或者完整测试
rosrun franka_perception test_perception_accuracy.py red
```

## 一键运行方案

如果你已经有Gazebo和cubes运行中：

```bash
# 终端1: 启动perception + 运行测试
cd /opt/ros_ws && source devel/setup.bash
roslaunch franka_perception sim_perception.launch target_color:=red &
sleep 3
rosrun franka_perception quick_test.py red
```

## 测试结果解读

### 精度评级标准

**2D定位精度 (XY平面):**
- ✓ EXCELLENT: < 10mm
- ✓ GOOD: 10-20mm
- ⚠ NEEDS IMPROVEMENT: > 20mm

**3D定位精度:**
- ✓ EXCELLENT: < 20mm
- ✓ GOOD: 20-30mm
- ⚠ NEEDS IMPROVEMENT: > 30mm

### 常见问题排查

**问题1: "No cubes found in Gazebo!"**
- 确认Gazebo已启动
- 确认场景中已spawn cubes
- 检查cube命名格式（必须是 cube_COLOR_number）

**问题2: "No detection received from perception node!"**
- 确认perception node正在运行：`rosnode list | grep perception`
- 检查ZED2话题是否发布：`rostopic hz /zed2/zed_node/rgb/image_rect_color`
- 查看perception日志

**问题3: 误差很大（>50mm）**
- 检查相机到机器人基座的TF变换
- 确认相机内参正确
- 检查点云数据质量
- 调整detection_params.yaml中的参数

**问题4: "No objects detected by perception!"**
- 确认方块在相机视野内
- 调低confidence_threshold参数
- 检查颜色HSV阈值配置
- 启用debug_image查看检测过程：
  ```bash
  roslaunch franka_perception sim_perception.launch target_color:=red publish_debug:=true
  rosrun rqt_image_view rqt_image_view /debug_image
  ```

## 预期性能指标

根据README.md中的规格：

| 指标 | 仿真模式目标 | 实际性能 |
|------|------------|---------|
| 检测延迟 | < 50ms | 待测 |
| 检测频率 | 10 Hz | 10 Hz |
| XY定位精度 | < 5mm | 待测 |
| Z定位精度 | < 10mm | 待测 |

运行测试后更新"实际性能"列。

## 高级用法

### 连续测试多个颜色
```bash
for color in red blue green yellow; do
    echo "Testing $color..."
    rosrun franka_perception quick_test.py $color
    sleep 2
done
```

### 导出测试结果
```bash
rosrun franka_perception test_perception_accuracy.py red 2>&1 | tee test_results_red.log
```

### 可视化检测过程
```bash
# 终端1: 启动带调试图像的perception
roslaunch franka_perception sim_perception.launch target_color:=red publish_debug:=true

# 终端2: 查看调试图像
rosrun rqt_image_view rqt_image_view /debug_image

# 终端3: 运行测试
rosrun franka_perception quick_test.py red
```

## 参数调优建议

如果测试结果不理想，可以调整 `config/detection_params.yaml`:

```yaml
# 提高下采样精度（但会降低速度）
voxel_size: 0.003  # 从0.005改为0.003

# 放宽聚类参数（检测更多小物体）
dbscan_eps: 0.015  # 从0.02改为0.015
dbscan_min_samples: 30  # 从50改为30

# 增加时间平滑（提高稳定性但降低响应速度）
ema_alpha: 0.2  # 从0.3改为0.2
max_history_len: 10  # 从5改为10
```

调整后重启perception node并重新测试。
