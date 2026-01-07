# Perception Testing Quick Reference

## 快速测试命令

### 最简单方式 - 一键完整测试
```bash
# 自动启动所有组件并测试
/opt/ros_ws/src/franka_perception/scripts/complete_test.sh red
```
这个脚本会自动：
- 检查/启动 roscore
- 检查/启动 Gazebo
- 生成彩色测试方块
- 启动perception节点
- 运行精度测试
- 显示结果

### 如果已有Gazebo运行
```bash
# 快速测试（假设Gazebo和cubes已存在）
/opt/ros_ws/src/franka_perception/scripts/quick_accuracy_test.sh red
```

### 手动步骤

**终端1 - 启动Gazebo（自动生成cubes）:**
```bash
# 带GUI（可视化，占用资源较多）
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch

# 无GUI（推荐用于测试，节省资源）
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=false
```
*注意：现在会自动生成4个彩色测试方块！*

**终端2 - 启动perception:**
```bash
roslaunch franka_perception sim_perception.launch target_color:=red
```

**终端3 - 运行测试:**
```bash
# 快速单次测试
rosrun franka_perception quick_test.py red

# 或10秒持续测试
rosrun franka_perception test_perception_accuracy.py red
```

## 测试不同颜色

```bash
# 红色
/opt/ros_ws/src/franka_perception/scripts/complete_test.sh red

# 蓝色
/opt/ros_ws/src/franka_perception/scripts/complete_test.sh blue

# 绿色
/opt/ros_ws/src/franka_perception/scripts/complete_test.sh green

# 黄色
/opt/ros_ws/src/franka_perception/scripts/complete_test.sh yellow
```

## 理解测试输出

### 成功示例
```
======================================================================
Comparison for RED cube
======================================================================
Matched Gazebo cube: cube_RED_1

Gazebo position:     x= 0.6500, y= 0.1500, z= 0.0450
Detected position:   x= 0.6523, y= 0.1489, z= 0.0463

Position Error:
  ΔX:   +2.30 mm
  ΔY:   -1.10 mm
  ΔZ:   +1.30 mm

  2D Error (XY):   2.55 mm
  3D Error:        2.91 mm
======================================================================

✓ EXCELLENT: 2D accuracy < 10mm
✓ EXCELLENT: 3D accuracy < 20mm
```

### 精度标准
- **EXCELLENT**: 2D < 10mm, 3D < 20mm
- **GOOD**: 2D < 20mm, 3D < 30mm
- **NEEDS IMPROVEMENT**: 更大误差

## 故障排除

### "No cubes found"
```bash
# 检查Gazebo中的模型
rosservice call /gazebo/get_world_properties

# 重新生成cubes
roslaunch franka_zed_gazebo spawn_colored_cubes.launch mode:=test
```

### "No detection received"
```bash
# 检查perception是否运行
rosnode list | grep perception

# 检查话题
rostopic echo /detected_objects
rostopic hz /zed2/zed_node/rgb/image_rect_color

# 查看日志
rosnode info /perception_node
```

### 误差很大
```bash
# 启用调试图像
roslaunch franka_perception sim_perception.launch target_color:=red publish_debug:=true

# 在另一终端查看
rosrun rqt_image_view rqt_image_view /debug_image
```

## 更多信息

详细文档请查看：
- [TESTING_GUIDE.md](TESTING_GUIDE.md) - 完整测试指南
- [README.md](README.md) - 包功能说明
