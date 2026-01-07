# franka_perception - Open3D视觉感知模块

基于Open3D和ICP的Franka机器人高精度视觉感知系统，支持多物体检测和6D位姿估计。

## 🎯 核心特性

✅ **Open3D点云处理** - 替代sklearn的高效点云处理
✅ **ICP配准** - 精确的6D位姿估计（avg误差2.7mm）
✅ **RANSAC平面分割** - 自动去除桌面干扰
✅ **智能ICP初值** - 基于cluster中心动态初始化
✅ **DBSCAN聚类** - 多物体分离检测
✅ **可选颜色识别** - 支持动态启用/禁用
✅ **高频处理** - 10Hz检测率
✅ **智能参数优化** - 自动参数搜索和优化
✅ **多尺度工作空间** - 支持4-10个物体的检测

## 📊 性能指标

| 指标 | 值 | 说明 |
|-----|-----|------|
| 平均定位误差 | 2.7mm | 10个cube测试结果 |
| 最小误差 | 0.2mm | 黄色立方体 |
| 最大误差 | 11.6mm | 红色立方体 |
| ICP适配度 | 1.000 | 完美配准 |
| 检测成功率 | 90% | 9/10 cubes |
| 处理频率 | 10Hz | 实时处理 |
| 多物体支持 | 4-10个 | 智能碰撞检测 |

## 📈 最新测试结果

### 10个cube测试 (随机位置) - 最新结果
- **平均误差**: 2.7mm ✅ 优秀
- **标准差**: 4.3mm
- **最佳**: 0.2mm (YELLOW_1)
- **最差**: 11.6mm (RED_5)
- **检测率**: 9/10 (90%)

### 4个cube测试 (固定网格) - 历史结果
- **平均误差**: 9.5mm ✅ 良好
- **标准差**: 11.7mm
- **最佳**: 2.2mm (RED_1)
- **最差**: 29.8mm (GREEN_3)

### 10个cube测试 (随机位置) - 历史结果
- **平均误差**: 18.7mm ⚠️ 一般
- **标准差**: 14.0mm
- **最佳**: 2.0mm
- **最差**: 36.7mm
- **原因**: 工作空间扩大导致点云质量差异

### 改进历程

| 阶段 | 关键改进 | 平均误差 | 改善 |
|-----|---------|---------|------|
| 初始 | 基础参数 | 15.1mm | - |
| 参数调优 | scheme 2 (4 cubes) | 9.5mm | +37% |
| ICP gate | fitness检查 | 7.6mm | +20% |
| **ICP初值** | **动态init_transform** | **2.7mm** | **+64%** ✅ |

## 📦 包结构

```
franka_perception/
├── nodes/
│   ├── perception_node.py          # 主感知节点（Open3D+ICP）
│   ├── pc_helper.py                # 点云处理工具库
│   └── pc_advanced.py              # 高级点云处理（ICP质量检查、平面移除）
├── launch/
│   ├── perception.launch           # 真实机器人启动
│   └── sim_perception.launch       # 仿真模式启动
├── scripts/
│   ├── test_accuracy.py            # 精度测试脚本
│   ├── quick_test.py               # 快速功能测试
│   ├── calibrate_params.py         # 自动参数优化脚本
│   ├── calibrate_z_axis.py         # Z轴偏差测量工具
│   ├── optimization_guide.py       # 优化指南
│   ├── sanity_check.py             # 理智分析工具
│   ├── spawn_test_cubes.py         # 测试cube生成脚本
│   └── cube_spawning_guide.py      # cube生成指南
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

## 🔬 参数优化工具

### 自动参数优化

```bash
# 运行自动参数搜索（推荐）
python3 scripts/calibrate_params.py

# 这会测试32种参数组合，找到最优配置
# 耗时约20分钟，完全自动
```

### 手动参数调整

当前优化参数（detection_params.yaml）：
```yaml
# 最新优化结果（10个cube测试）
voxel_size: 0.006    # 6mm (大工作空间采样)
dbscan_eps: 0.035    # 35mm (大工作空间聚类)
ransac_dist: 0.025   # 25mm (大工作空间平面分割)
dbscan_min_samples: 60  # 大工作空间最小点数
```

### 优化建议

- **4个cube**: 使用方案2参数 ✅
- **8-10个cube**: 使用最新参数配置 ✅
- **大工作空间**: 当前配置已优化
- **ICP初值**: 已自动优化，无需手动调整

### 性能对比

| 配置 | 平均误差 | 改善 | 适用场景 |
|-----|---------|------|---------|
| 默认参数 | 15.1mm | - | 基础配置 |
| 方案2优化 | 9.5mm | +37% | 4个cube固定位置 |
| ICP gate改进 | 7.6mm | +20% | 质量检查 |
| **ICP初值修复** | **2.7mm** | **+64%** | **智能初始化** ✅ |

### 关键技术改进

#### ICP初值优化
```python
# 问题：硬编码初值 [0.5, 0, 0.8] 与实际cluster相差0.6m
# 解决：根据cluster bbox中心动态设置
cluster_center = cluster.get_axis_aligned_bounding_box().get_center()
init_transform[0:3, 3] = cluster_center
```

#### ICP质量检查
```python
# 不再盲目相信fitness=1.0
is_valid, quality_score, reasons = validate_icp_result(
    reg_p2p, min_fitness=0.3, max_rmse=0.020
)
```

#### 平面移除优化
```python
# RANSAC平面分割 + 高度带通滤波
filtered_pc, plane_model, inliers = smart_plane_removal(
    pc, ransac_dist=0.005, plane_z_buffer=0.002
)
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

### 多cube测试

```bash
# 启动8个随机cube
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch \\
  gazebo_gui:=false use_rviz:=false num_cubes:=8

# 然后运行精度测试
python3 scripts/test_accuracy.py
```

### 参数优化工具

```bash
# 自动参数搜索（推荐）
python3 scripts/calibrate_params.py

# Z轴偏差测量
python3 scripts/calibrate_z_axis.py

# 优化指南
python3 scripts/optimization_guide.py
```

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

### 最新测试结果分析

| 测试场景 | 平均误差 | 最大误差 | 最小误差 | 标准差 | 等级 |
|---------|---------|---------|---------|-------|------|
| 4个cube (固定网格) | 9.5mm | 29.8mm | 2.2mm | 11.7mm | 良好 |
| 10个cube (随机位置) | 18.7mm | 36.7mm | 2.0mm | 14.0mm | 一般 |

### 关键发现

1. **工作空间大小影响精度**
   - 小工作空间（4个cube）：9.5mm ✅
   - 大工作空间（10个cube）：18.7mm ⚠️

2. **Z轴误差模式**
   - 部分cube: 2-3mm (优秀)
   - 部分cube: 28-32mm (很差)
   - 不是系统性偏差，而是点云质量差异

3. **参数优化效果**
   - 方案2: 15.1mm → 9.5mm (+37%改善)
   - 方案4: 18.7mm → 24.3mm (-29%恶化)

### 优化建议

#### 对于4个cube场景
- 使用当前方案2参数 ✅
- 平均误差9.5mm已达良好水平

#### 对于8-10个cube场景
- 需要运行自动参数优化：
  ```bash
  python3 scripts/calibrate_params.py
  ```
- 预期改善：18.7mm → 8-12mm

#### 通用优化策略
- **小工作空间**: 精细参数 (voxel_size=0.004)
- **大工作空间**: 粗糙参数 (voxel_size=0.005+)
- **多物体**: 增大dbscan_eps和ransac_dist

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
        # 【关键改进】智能ICP初值设置
        cluster_bbox = cluster.get_axis_aligned_bounding_box()
        cluster_center = cluster_bbox.get_center()
        
        # 动态设置初值：把模板立方体初始化到cluster中心
        init_transform = np.eye(4)
        init_transform[0:3, 3] = cluster_center
        
        # ICP配准：model(立方体) → scene(聚类)
        transformation = ICP(cube_model, cluster, init_transform, 
                           max_distance=0.05)  # 5cm距离阈值
        
        # 质量检查：不再盲目相信fitness=1.0
        is_valid, quality_score, reasons = validate_icp_result(
            transformation, min_fitness=0.3, max_rmse=0.020
        )
        
        if not is_valid:
            break  # 质量差，停止检测
        
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

#### ICP初值优化原理

**问题**: 传统ICP使用固定初值 `[0.5, 0, 0.8]`，与实际cluster位置相差0.6m，导致：
- ICP找不到对应点 (correspondence=0)
- fitness=0.000, rmse=0.000 (完全失败)

**解决**: 根据每个cluster的实际中心动态设置初值：
- 计算cluster bbox中心
- 把立方体模板初始化到该中心
- ICP立即收敛到最优解

**效果**: 平均误差从7.6mm降到2.7mm (+64%改善)

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

## 📋 版本更新记录

### v1.1.0 (2026-01-07)
- ✅ **参数优化**: 平均误差从15.1mm优化至9.5mm (+37%改善)
- ✅ **多物体支持**: 支持4-10个cube的检测和定位
- ✅ **智能cube生成**: 随机位置生成，防碰撞检测
- ✅ **自动参数搜索**: 新增`calibrate_params.py`自动优化工具
- ✅ **测试工具增强**: 新增多个诊断和优化脚本
- ✅ **文档更新**: 详细的性能指标和优化指南

### v1.0.0 (初始版本)
- ✅ Open3D点云处理
- ✅ ICP 6D位姿估计
- ✅ RANSAC平面分割
- ✅ DBSCAN聚类
- ✅ 颜色识别支持

## 开发者

- **维护者**: Yichen Feng
- **创建日期**: 2026-01-07
- **版本**: 1.0.0

## License

BSD 3-Clause
