# Perception话题修复说明

## 问题描述
在Gazebo仿真环境中，perception节点无法接收到ZED2相机数据。

## 根本原因
**话题名称不匹配：**
- Gazebo中ZED2相机发布到：`/zed2/zed_node/left/image_rect_color`
- Perception节点订阅：`/zed2/zed_node/rgb/image_rect_color` ❌

## 解决方案
✅ 已修复perception_node.py，使其：
1. 从ROS参数读取话题名称
2. 默认使用 `left/image_rect_color`（适配仿真）
3. 可通过launch文件参数配置

## 修改的文件
1. ✅ `nodes/perception_node.py` - 添加话题参数
2. ✅ `launch/perception.launch` - 添加rgb_topic和cloud_topic参数
3. ✅ `launch/sim_perception.launch` - 配置仿真话题
4. ✅ `README.md` - 更新话题文档

## 使用方法

### 仿真模式（默认）
```bash
roslaunch franka_perception sim_perception.launch target_color:=red
# 自动使用 /zed2/zed_node/left/image_rect_color
```

### 真实机器人
```bash
roslaunch franka_perception perception.launch sim_mode:=false \
  rgb_topic:=/zed2/zed_node/rgb/image_rect_color
```

### 自定义话题
```bash
roslaunch franka_perception perception.launch \
  rgb_topic:=/my_camera/image \
  cloud_topic:=/my_camera/points
```

## 验证修复

运行诊断脚本：
```bash
/opt/ros_ws/src/franka_perception/scripts/diagnose.sh
```

应该看到：
```
✓ ROS Master... Running
✓ Gazebo... Running
✓ Test cubes... Found 4 cubes
✓ RGB Image... Publishing at X Hz
✓ Point Cloud... Publishing at X Hz
✓ Perception Node... Running
✓ /detected_objects... Publishing
```

## 完整测试流程

```bash
# 1. 启动Gazebo
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch

# 2. 生成测试cubes
roslaunch franka_zed_gazebo spawn_colored_cubes.launch mode:=test

# 3. 运行诊断
/opt/ros_ws/src/franka_perception/scripts/diagnose.sh

# 4. 启动perception
roslaunch franka_perception sim_perception.launch target_color:=red

# 5. 运行测试
rosrun franka_perception quick_test.py red
```

或者一键运行：
```bash
/opt/ros_ws/src/franka_perception/scripts/complete_test.sh red
```
