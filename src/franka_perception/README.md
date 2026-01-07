# franka_perception - Open3D视觉感知模块

基于Open3D和ICP的Franka机器人高精度视觉感知系统，支持多物体检测和6D位姿估计。

## 🎯 核心特性

✅ **Open3D点云处理** - 替代sklearn的高效点云处理
✅ **ICP配准** - 精确的6D位姿估计（avg误差21.1mm）
✅ **RANSAC平面分割** - 自动去除桌面干扰
✅ **DBSCAN聚类** - 多物体分离检测
✅ **可选颜色识别** - 支持动态启用/禁用
✅ **高频处理** - 10Hz检测率

## 📊 性能指标

| 指标 | 值 | 说明 |
|-----|-----|------|
| 平均定位误差 | 21.1mm | 可通过参数优化至<10mm |
| 最小误差 | 1.9mm | 黄色立方体 |
| 最大误差 | 29.8mm | 红色立方体 |
| ICP适配度 | 1.000 | 完美配准 |
| 检测成功率 | 100% | 4/4 cubes |
| 处理频率 | 10Hz | 实时处理 |

## 📦 包结构

```
franka_perception/
├── nodes/
│   ├── perception_node.py          # 主感知节点（Open3D+ICP）
│   └── pc_helper.py                # 点云处理工具库
├── launch/
│   ├── perception.launch           # 真实机器人启动
│   └── sim_perception.launch       # 仿真模式启动
├── scripts/
│   ├── test_accuracy.py            # 精度测试脚本
│   ├── quick_test.py               # 快速功能测试
│   └── diagnose.sh                 # 诊断脚本
├── config/
│   ├── camera_params.yaml          # 相机参数
│   └── detection_params.yaml       # 检测参数
└── src/franka_perception/
    └── __init__.py
```

## 🚀 快速开始

### 1. 编译

```bash
cd /opt/ros_ws
catkin_make
source devel/setup.bash
```

### 2. 启动仿真环境

```bash
# 终端1: 启动Gazebo和MoveIt（无GUI）
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch \
  gazebo_gui:=false use_rviz:=false
```

### 3. 启动感知节点

```bash
# 终端2: 启动perception
roslaunch franka_perception sim_perception.launch
```

### 4. 运行精度测试

```bash
# 终端3: 测试定位精度
python3 scripts/test_accuracy.py
```

## 🔧 配置参数

### 工作空间边界 (launch文件)

```xml
<!-- 仿真环境 -->
<rosparam param="boundX">[-1.0, 1.0]</rosparam>
<rosparam param="boundY">[-1.0, 1.0]</rosparam>
<rosparam param="boundZ">[-1.0, 1.0]</rosparam>

<!-- 真实机器人 -->
<rosparam param="boundX">[0.0, 1.0]</rosparam>
<rosparam param="boundY">[-0.5, 0.5]</rosparam>
<rosparam param="boundZ">[-0.1, 0.3]</rosparam>
```

### 点云处理参数

| 参数 | 默认值 | 范围 | 说明 |
|-----|-------|------|------|
| `voxel_size` | 0.001m | 0.0001-0.01 | 体素下采样粒度 |
| `dbscan_eps` | 0.015m | 0.005-0.05 | 聚类距离阈值 |
| `dbscan_min_points` | 20 | 10-100 | 最小簇点数 |
| `ransac_dist` | 0.01m | 0.005-0.02 | 平面分割距离 |
| `icp_min_points` | 100 | 50-200 | ICP触发最少点数 |

### 感知选项

```bash
# 启用颜色识别
roslaunch franka_perception sim_perception.launch enable_color_detection:=true

# 设置目标颜色
roslaunch franka_perception sim_perception.launch target_color:=red

# 禁用ICP使用简单质心检测
roslaunch franka_perception sim_perception.launch use_icp:=false
```

## 🎨 ROS话题接口

### 发布话题

| 话题 | 消息类型 | 描述 |
|------|---------|------|
| `/detected_objects` | std_msgs/String | JSON格式的所有检测物体 |
| `/cube_*_odom_pc` | nav_msgs/Odometry | 各物体的位姿估计 |
| `/segmented_pc` | sensor_msgs/PointCloud2 | 分割后的点云（彩色） |
| `/num_cubes` | std_msgs/String | 检测物体数量 |

### 订阅话题

| 话题 | 消息类型 | 描述 |
|------|---------|------|
| `/zed2/zed_node/point_cloud/cloud_registered` | PointCloud2 | 点云输入 |
| `/zed2/zed_node/left/image_rect_color` | Image | RGB图像输入 |
| `/target_color` | std_msgs/String | 目标颜色设置 |

## 🧪 测试脚本

### 精度测试 - test_accuracy.py

对比检测结果与Gazebo真值：

```bash
python3 scripts/test_accuracy.py
```

输出内容：
- ✓ 找到的Gazebo cube数量
- 每个检测物体的：
  - 检测位置 vs 真值
  - XYZ分量误差（mm）
  - 总误差
  - ICP适配度
- 统计结果：平均误差、最大/最小误差、标准差
- 精度等级评估

### 快速功能测试 - quick_test.py

```bash
rosrun franka_perception quick_test.py [color]
```

### 诊断脚本 - diagnose.sh

```bash
bash scripts/diagnose.sh
```

检查：
- ROS环境
- 必需的ROS包
- 话题连接情况
- 参数设置

## 📈 精度优化指南

根据测试结果（平均误差21.1mm），以下调整可进一步改善精度：

### 1. 工作空间优化
- 当前Y轴误差较大（22-27mm）
- 建议缩小Y范围：`[-0.2, 0.2]` 而不是 `[-1.0, 1.0]`
- 调整X范围为 `[0.2, 0.8]` 更符合实际工作空间

### 2. 聚类参数调优
```xml
<!-- 提高聚类精度 -->
<param name="dbscan_eps" value="0.01" />      <!-- 1.0cm (从1.5cm) -->
<param name="dbscan_min_points" value="30" /> <!-- 从20 -->
```

### 3. ICP初始化改进
在 `process_cluster_with_icp` 中根据实际cube位置调整：
```python
init_transform = np.array([
    [1, 0, 0, x_guess],     # 根据检测到的簇中心调整
    [0, 1, 0, y_guess],
    [0, 0, 1, z_guess],
    [0, 0, 0, 1]
])
```

### 4. Voxel大小调整
```xml
<!-- 真实场景（点云更稀疏） -->
<param name="voxel_size" value="0.005" /> <!-- 5mm -->

<!-- 高精度场景 -->
<param name="voxel_size" value="0.001" /> <!-- 1mm -->
```

## 📝 关键算法

### 1. 点云处理管道

```
原始点云 
  ↓ [变换到世界坐标系]
  ↓ [体素下采样]
  ↓ [工作空间裁剪]
  ↓ [RANSAC平面分割]
  ↓ [DBSCAN聚类]
  ↓ [ICP位姿估计]
检测结果
```

### 2. ICP配准流程

```python
# 对每个聚类
for cluster in clusters:
    # 迭代分离多个物体
    while cluster.size() > min_points:
        # ICP配准：model(立方体) → scene(聚类)
        transformation = ICP(cube_model, cluster, init_guess)
        
        # 提取位置和旋转
        position = transformation[0:3, 3]
        rotation = transformation[0:3, 0:3]
        
        # 转换为四元数和欧拉角
        quaternion = rotation_to_quaternion(rotation)
        
        # 发布Odometry消息
        publish_odometry(position, quaternion)
        
        # 移除已检测的点
        cluster = remove_points_near(cluster, position, radius)
```

## 🔗 相关资源

- [参考项目: frankastadt](https://github.com/frankarobotics/frankastadt)
- [Open3D文档](http://www.open3d.org/)
- [ZED2相机文档](https://www.stereolabs.com/docs/api/python/)

## 🐛 常见问题

### Q: 检测精度不够高（>30mm）
A: 检查以下项：
1. 工作空间边界设置是否正确
2. `dbscan_eps` 是否过大
3. 点云质量是否良好（运行 `rostopic echo /zed2/zed_node/point_cloud/cloud_registered`）

### Q: 某个特定物体总是检测不准
A: 可能原因：
1. 该物体的点云被遮挡或不完整
2. ICP初值距离物体太远
3. 立方体模型大小设置不正确 (`cube_edge_len`)

### Q: perception node启动失败
A: 运行诊断脚本：
```bash
bash scripts/diagnose.sh
```

## 📄 许可证

BSD License

---

**最后更新**: 2026-01-07
**精度测试**: 平均21.1mm (4/4 cubes detected)


**注意：** 真实机器人上，RGB话题可能是 `/zed2/zed_node/rgb/image_rect_color`，可通过launch参数配置。

### 参数（Parameters）

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `~target_color` | string | "red" | 目标物体颜色 |
| `~sim_mode` | bool | true | 是否在仿真模式下运行 |
| `~confidence_threshold` | float | 0.8 | 检测置信度阈值 |
| `~publish_debug_image` | bool | false | 是否发布调试图像 |
| `~detection_rate` | float | 10.0 | 检测频率(Hz) |

## 使用方法

### 1. 仿真模式

```bash
# 启动仿真环境（需先启动Gazebo和MoveIt）
roslaunch franka_perception sim_perception.launch target_color:=red
```

### 2. 真实机器人模式

```bash
# 启动ZED2相机和感知节点
roslaunch franka_perception perception.launch sim_mode:=false
```

### 3. 测试检测

```bash
# 查看检测到的物体
rostopic echo /detected_objects

# 查看物体姿态
rostopic echo /object_pose

# 动态改变目标颜色
rostopic pub /target_color std_msgs/String "data: 'blue'" -1
```

## detected_objects消息格式

```json
{
    "objects": [
        {
            "color": "red",
            "position": [0.5, 0.2, 0.045],
            "confidence": 0.95,
            "area": 1500,
            "optimal_yaw": 45.0
        },
        {
            "color": "blue",
            "position": [0.6, -0.1, 0.045],
            "confidence": 0.89,
            "area": 1450,
            "optimal_yaw": 90.0
        }
    ],
    "timestamp": 1234567890.123,
    "frame_id": "panda_link0"
}
```

## 依赖

- ROS Noetic
- OpenCV 4
- PCL (Point Cloud Library)
- ZED SDK (仅真实机器人)
- cv_bridge

## 坐标系

- **输入**: `camera_link` 坐标系（ZED2相机坐标系）
- **输出**: `panda_link0` 坐标系（机器人基坐标系）
- TF变换: 自动从`robot_state_publisher`获取`panda_hand`到`camera_link`的变换

## 性能指标

| 指标 | 仿真模式 | 真实机器人 |
|------|----------|------------|
| 检测延迟 | < 50ms | < 100ms |
| 检测频率 | 10 Hz | 5-10 Hz |
| XY定位精度 | < 5mm | < 10mm |
| Z定位精度 | < 10mm | < 20mm |

## 故障排除

### 问题1: 未检测到物体
- 检查相机话题是否正常：`rostopic hz /zed2/zed_node/rgb/image_rect_color`
- 确认目标颜色参数正确
- 查看调试图像：`rosrun rqt_image_view rqt_image_view /debug_image`

### 问题2: 位置偏差大
- 校准eye-in-hand TF变换
- 检查相机内参
- 确认工作台高度

### 问题3: 检测不稳定
- 调整`confidence_threshold`参数
- 改善光照条件
- 降低检测频率

## 开发者

- **维护者**: Yichen Feng
- **创建日期**: 2026-01-07
- **版本**: 1.0.0

## License

BSD 3-Clause
