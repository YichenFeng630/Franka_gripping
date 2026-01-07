# franka_perception

Franka机器人视觉感知模块 - 基于ZED2相机的物体检测和位置估计

## 功能概述

该包负责处理来自ZED2相机（安装在Franka夹爪上）的视觉信息，实现：
- RGB颜色检测
- 点云处理和物体分割
- 物体位置估计（6D pose）
- 支持仿真和真实机器人模式

## 包结构

```
franka_perception/
├── nodes/
│   ├── perception_node.py          # 主感知节点
│   ├── rgb_detector.py             # RGB颜色检测器
│   └── pointcloud_processor.py     # 点云处理器
├── src/franka_perception/
│   ├── object_detector.py          # 物体检测核心类
│   └── pose_estimator.py           # 姿态估计器
├── launch/
│   ├── perception.launch           # 主启动文件
│   └── sim_perception.launch       # 仿真模式启动
└── config/
    ├── camera_params.yaml          # 相机参数
    └── detection_params.yaml       # 检测参数
```

## ROS接口

### 发布话题（Publishers）

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/detected_objects` | `std_msgs/String` (JSON) | 检测到的所有物体列表 |
| `/object_pose` | `geometry_msgs/PoseStamped` | 目标物体的6D姿态 |
| `/detection_status` | `std_msgs/String` | 检测状态信息 |
| `/debug_image` | `sensor_msgs/Image` | 标注后的调试图像 |

### 订阅话题（Subscribers）

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/zed2/zed_node/left/image_rect_color` | `sensor_msgs/Image` | ZED2 RGB图像（仿真默认） |
| `/zed2/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | ZED2点云数据 |
| `/target_color` | `std_msgs/String` | 动态设置目标颜色 |

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
