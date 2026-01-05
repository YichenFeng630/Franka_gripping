# ZED2相机配置指南

**基于**: `franka_zed_gazebo`包 + README文档  
**日期**: 2026-01-05  
**状态**: ✅ 配置完整

---

## 📋 系统架构

```
Franka Panda Robot
  ├── End Effector (panda_hand)
  │   └── ZED2 Camera Mount
  │       ├── Left Camera (left_camera_link_optical)
  │       ├── Right Camera (right_camera_link_optical)
  │       └── IMU (zed2_imu_link)
  │
  └── Publishing:
      ├── RGB Images: /zed2/zed_node/left/image_rect_color
      ├── Depth: /zed2/zed_node/left/depth/depth_registered
      ├── Point Cloud: /zed2/zed_node/point_cloud/cloud_registered
      └── IMU: /zed2/zed_node/imu/data
```

---

## 🎮 仿真环境配置

### 前置要求

```bash
# 安装必需包
sudo apt-get install -y ros-noetic-franka-ros \
                       ros-noetic-panda-moveit-config \
                       ros-noetic-moveit \
                       ros-noetic-gazebo-ros \
                       ros-noetic-gazebo-ros-pkgs
```

### 启动Gazebo仿真 (推荐)

**方法1: 完整系统启动** (包含Grasp Planning + Color Detection)

```bash
cd /opt/ros_ws
source devel/setup.bash

# 启动完整的仿真系统（包括ZED2）
roslaunch panda_grasp_planning panda_grasp_complete.launch \
    sim:=true \
    rviz:=true \
    enable_place:=true
```

**方法2: 仅启动Gazebo + Panda + ZED2**

```bash
cd /opt/ros_ws
source devel/setup.bash

# 启动Gazebo和机器人仿真
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch
```

### 可视化选项

在RViz中显示ZED2相机图像：

1. 点击 `Add` → `By topic`
2. 选择：
   - `/zed2/zed_node/left/image_rect_color` - RGB图像
   - `/zed2/zed_node/left/depth/depth_registered` - 深度图
   - `/zed2/zed_node/point_cloud/cloud_registered` - 点云

### 修改camera mount选项

根据README，有两种mount选项：

**当前配置**: `camera_gripper.xacro` (包含手爪在视野中)

如需切换到不含手爪的配置：

编辑 `/opt/ros_ws/src/franka_zed_gazebo/urdf/panda_camera.urdf.xacro`：

```xml
<!-- 当前（含手爪）-->
<xacro:include filename="$(find franka_zed_gazebo)/urdf/camera_gripper.xacro" />

<!-- 改为（不含手爪）-->
<!-- <xacro:include filename="$(find franka_zed_gazebo)/urdf/camera_gripper.xacro" /> -->
<!-- <xacro:include filename="$(find franka_zed_gazebo)/urdf/camera_no_gripper.xacro" /> -->
```

然后重新启动Gazebo。

---

## 🤖 真实机器人配置

### 前置要求

1. **ZED2硬件**已正确安装在机器人手爪上
2. **ZED ROS Wrapper**已安装：
   ```bash
   sudo apt-get install ros-noetic-zed-wrapper
   ```
3. **3D打印支架**正确装配（参见 `franka_zed_gazebo/3d_prints/` 文件夹）

### 启动真实机器人 + ZED2

```bash
cd /opt/ros_ws
source devel/setup.bash

# 启动机器人控制和ZED2驱动
roslaunch franka_zed_gazebo real_robot_zed2.launch \
    robot_ip:=192.168.1.35
```

**参数说明**:
- `robot_ip`: 机器人的FCI IP地址 (根据实际修改)

### 相机坐标变换配置

根据camera mount选项修改 `launch/real_robot_zed2.launch` 中的静态变换：

**含手爪在视野中** (当前配置):
```bash
args="-0.097397 0 0.0274111  0 -0.824473 0 panda_hand zed2_left_camera_frame 100"
```

**不含手爪** (30度倾斜):
```bash
args="-0.115 0.056 0.018  0 -1.35 0 panda_hand zed2_left_camera_frame 100"
```

变换参数含义:
- `xyz`: 相机相对于panda_hand的位置 (米)
- `rpy`: 欧拉角 (弧度)
- `panda_hand`: 父链接
- `zed2_left_camera_frame`: 子链接
- `100`: 发布频率 (Hz)

---

## 📊 可用的ROS Topics

### RGB图像

| Topic | 类型 | 频率 | 描述 |
|-------|------|------|------|
| `/zed2/zed_node/left/image_raw_color` | `sensor_msgs/Image` | 15 Hz | 原始RGB (含畸变) |
| `/zed2/zed_node/left/image_rect_color` | `sensor_msgs/Image` | 15 Hz | 矫正后RGB (推荐用于检测) |
| `/zed2/zed_node/left/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | 左摄像头内参 |
| `/zed2/zed_node/right/image_raw_color` | `sensor_msgs/Image` | 15 Hz | 右摄像头原始RGB |
| `/zed2/zed_node/right/image_rect_color` | `sensor_msgs/Image` | 15 Hz | 右摄像头矫正RGB |

### 深度和点云

| Topic | 类型 | 频率 | 描述 |
|-------|------|------|------|
| `/zed2/zed_node/depth/depth_registered` | `sensor_msgs/Image` | 15 Hz | 深度图 (对齐到RGB) |
| `/zed2/zed_node/depth/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | 深度摄像头内参 |
| `/zed2/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | 15 Hz | 配准点云 |

### IMU数据

| Topic | 类型 | 频率 | 描述 |
|-------|------|------|------|
| `/zed2/zed_node/imu/data` | `sensor_msgs/Imu` | 100 Hz | IMU数据 |

### 坐标变换

| Frame | Parent | Description |
|-------|--------|-------------|
| `left_camera_link_optical` | `left_camera_link` | 左摄像头光学坐标系 |
| `right_camera_link_optical` | `right_camera_link` | 右摄像头光学坐标系 |
| `camera_link` | `panda_hand` | 相机支架坐标系 |
| `zed2_imu_link` | `left_camera_link` | IMU坐标系 |

---

## 🔧 相机内参

### ZED2左摄像头 (来自Gazebo配置)

```yaml
intrinsics:
  fx: 658.82
  fy: 658.82
  cx: 658.82
  cy: 372.26
  width: 1280
  height: 720
  horizontal_fov: 1.7633 rad (101°)

distortion:
  k1: -0.043693598
  k2: 0.0146164996
  p1: -0.006573319
  p2: -0.000216900
  k3: 0.000084328
```

### ZED2右摄像头

```yaml
intrinsics:
  fx: 658.82
  fy: 658.82
  cx: 659.30
  cy: 371.40
  width: 1280
  height: 720

distortion:
  k1: -0.040993299
  k2: 0.009593590
  p1: -0.004429849
  p2: 0.000192024
  k3: -0.000320880

baseline: 0.12 m (立体对应的基线)
```

---

## 🎨 颜色检测集成

### 使用增强颜色检测器 (推荐)

启动颜色检测节点：

```bash
# 在新终端中运行
rosrun panda_grasp_planning enhanced_color_detector_zed2.py \
    _display_debug:=true \
    _rgb_topic:=/zed2/zed_node/left/image_rect_color
```

**参数**:
- `_display_debug`: 是否显示调试图像 (default: false)
- `_rgb_topic`: RGB图像订阅topic (default: /zed2/zed_node/left/image_rect_color)
- `_target_frame`: 目标坐标系 (default: panda_link0)
- `_camera_frame`: 相机光学帧 (default: left_camera_link_optical)
- `_min_area`: 最小检测面积像素² (default: 100)

### 订阅检测结果

```bash
rostopic echo /color_coordinates
```

输出格式: `"COLOR,x,y,z"`  
示例: `"R,0.3456,0.1234,0.5000"`

---

## 📹 使用ROS Bag数据测试

参考原始README，可以使用rosbag数据进行离线测试：

```bash
# 克隆rosbag分支
git clone -b rosbags https://github.com/pearl-robot-lab/franka_zed_gazebo.git rosbag_data

# 在一个终端启动ROS master
roscore

# 在另一个终端播放rosbag（循环播放）
cd rosbag_data
rosbag play -l sample_2.bag

# 在第三个终端运行颜色检测
rosrun panda_grasp_planning enhanced_color_detector_zed2.py _display_debug:=true
```

---

## 🔍 故障排除

### 1. Gazebo未发布ZED2 topics

**症状**: 无法订阅 `/zed2/zed_node/left/image_rect_color`

**解决**:
```bash
# 检查Gazebo中的topic
rostopic list | grep zed2

# 检查Gazebo插件是否加载
rostopic list | grep camera
```

如果没有topics，检查 `camera_gripper.xacro` 中的Gazebo插件配置。

### 2. TF2坐标变换丢失

**症状**: 颜色检测时TF lookup失败

**解决**:
```bash
# 检查TF树
rosrun tf view_frames
rosrun tf_echo left_camera_link_optical panda_link0
```

确保 `panda_camera.urdf.xacro` 正确included到主URDF。

### 3. 相机图像畸变

**症状**: 检测到的物体位置不准确

**解决**:
- 使用 `image_rect_color` topic (矫正的) 而不是 `image_raw_color`
- 确保内参值正确
- 在enhanced_color_detector_zed2.py中调整 `fx`, `fy`, `cx`, `cy` 参数

### 4. 真实机器人ZED2无驱动

**症状**: `roslaunch real_robot_zed2.launch` 失败

**解决**:
```bash
# 确保zed_wrapper已安装
sudo apt-get install ros-noetic-zed-wrapper

# 检查ZED2 USB连接
lsusb | grep ZED

# 测试ZED2驱动
roslaunch zed_wrapper zed2.launch
```

---

## 🚀 完整的演示流程

### 仿真环境演示

```bash
# Terminal 1: 启动Gazebo + Panda + ZED2
roslaunch panda_grasp_planning panda_grasp_complete.launch \
    sim:=true rviz:=true enable_place:=true

# Terminal 2: 运行颜色检测
rosrun panda_grasp_planning enhanced_color_detector_zed2.py \
    _display_debug:=true

# Terminal 3: 运行拾取演示
python3 /opt/ros_ws/src/panda_grasp_planning/scripts/v4_demo.py \
    --trials=5 --enable-place --verbose

# Terminal 4: 监控检测结果
rostopic echo /color_coordinates
```

### 真实机器人演示

```bash
# Terminal 1: 启动机器人 + ZED2
roslaunch franka_zed_gazebo real_robot_zed2.launch \
    robot_ip:=192.168.1.35

# Terminal 2: 运行颜色检测
rosrun panda_grasp_planning enhanced_color_detector_zed2.py \
    _display_debug:=true

# Terminal 3: 运行拾取
python3 /opt/ros_ws/src/panda_grasp_planning/scripts/v4_demo.py \
    --trials=5 --enable-place --verbose
```

---

## 📚 参考资源

- **原始franka_zed_gazebo README**: `/opt/ros_ws/src/franka_zed_gazebo/README.md`
- **URDF定义**: `/opt/ros_ws/src/franka_zed_gazebo/urdf/camera_gripper.xacro`
- **Gazebo配置**: `urdf/camera_gripper.xacro` (lines 151-296)
- **Launch文件**: `/opt/ros_ws/src/franka_zed_gazebo/launch/`
- **颜色检测模块**: `/opt/ros_ws/src/panda_grasp_planning/scripts/enhanced_color_detector_zed2.py`
- **ZED ROS Wrapper文档**: https://www.stereolabs.com/docs/ros

---

## ✅ 配置检查清单

- [x] 相机URDF已定义 (camera_gripper.xacro)
- [x] Gazebo插件已配置 (RGB + Depth + IMU)
- [x] TF frames已建立 (camera_link → optical frames)
- [x] Launch文件已准备 (simulation + real robot)
- [x] 颜色检测器已适配ZED2 (enhanced_color_detector_zed2.py)
- [x] Topics已定义和发布
- [x] 内参已设置
- [ ] 需要的额外模块（如image_proc）可选

---

**最后更新**: 2026-01-05  
**维护者**: GitHub Copilot  
**状态**: 生产就绪

