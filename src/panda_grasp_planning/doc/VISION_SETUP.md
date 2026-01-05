# 视觉系统说明

## 📷 相机配置

### 相机类型：ZED2

**制造商**: Stereolabs  
**型号**: ZED 2  
**安装位置**: Franka Panda 末端执行器（手爪）上  
**主要特性**:
- 双目立体视觉相机
- 左右相机基线：120mm (0.06m)
- 支持深度估计
- 分辨率：2560×1440 (4K) @ 30fps

---

## 🔗 机械结构

### 安装位置细节

```yaml
Parent Link: panda_hand (Panda 手爪)

Camera Holder:
  - 链接: zed2_holder
  - 相对于 panda_hand 的姿态:
    xyz: [0, 0, 0]
    rpy: [π, 0, 0]  # 旋转 180° 绕 X 轴
  - 3D 模型: franka-ZED2-camera-holder.stl
  - 质量: ~50g

Camera Link:
  - 链接: camera_link
  - 相对于 panda_hand 的姿态:
    xyz: [-0.097397, 0, 0.0274111]  # 前 9.7cm, 下 2.7cm
    rpy: [0, -0.824473, 0]           # 俯视角约 47°
  - 质量: 161.4g
  - 3D 模型: ZED2.dae
```

### 立体相机配置

**左相机（Left Camera）**:
```
链接: left_camera_link_optical
相对于 camera_link 的位置: [0, 0.06, 0]  (右侧 6cm)
方向: 标准光学坐标系 (Z 轴向前)
```

**右相机（Right Camera）**:
```
链接: right_camera_link_optical
相对于 camera_link 的位置: [0, -0.06, 0]  (左侧 6cm)
方向: 标准光学坐标系 (Z 轴向前)
```

---

## 📊 ROS 接口

### 发布的话题

以下为 ZED ROS 包发布的典型话题（当连接真实相机时）：

```
/zed2/zed_node/
├── rgb/
│   ├── image_rect_color          # RGB 图像
│   └── camera_info               # 相机标定参数
├── left/
│   ├── image_rect_gray           # 左灰度图
│   └── camera_info
├── right/
│   ├── image_rect_gray           # 右灰度图
│   └── camera_info
├── depth/
│   ├── depth_registered          # 深度图 (配准到 RGB)
│   └── camera_info
└── point_cloud/
    └── cloud_registered          # 点云数据
```

### 话题帧 (TF Frame)

- `left_camera_link_optical` - 左相机光学中心
- `right_camera_link_optical` - 右相机光学中心
- `camera_link` - 相机中心（两个光学中心之间）

---

## 🎯 当前项目中的应用

### 当前状态

**仿真环境**：✅ Gazebo 模拟（无真实图像处理）
- 相机 URDF 已集成到 Panda 模型
- 可以在 RViz 中可视化相机位置
- **目前抓取规划不依赖视觉反馈**

### 视觉信息缺失

当前的 `panda_grasp_planning` 包中：
- ❌ 无图像处理代码
- ❌ 无物体检测代码
- ❌ 无 VLA 视觉模型
- ✅ 预留了 VLA 集成接口

目标位置 `/target_cube_pose` 的来源是**外部提供**（非视觉）。

---

## 🔮 视觉集成路径

如果要添加视觉功能，建议的流程：

### 步骤 1：物体检测模块

```python
# 新增 vision/ 包
vision/
├── cube_detector.py          # 立方体检测
├── pose_estimator.py         # 位姿估计
└── launch/
    └── vision_pipeline.launch
```

### 步骤 2：将检测结果发布为 PoseStamped

```python
# 在 cube_detector.py 中
pub = rospy.Publisher('/target_cube_pose', PoseStamped)
```

### 步骤 3：VLA 候选评分

```python
# 利用 example_vla_integration.py
vla_scores = vla_model.score(
    candidates=generator.generate(target),
    image=camera_frame
)
selected = candidates[vla_scores.argmax()]
```

---

## 📂 相关文件位置

| 文件 | 说明 |
|------|------|
| `franka_zed_gazebo/urdf/panda_camera.urdf.xacro` | 相机 URDF 定义 |
| `franka_zed_gazebo/urdf/camera_gripper.xacro` | ZED2 链接配置 |
| `franka_zed_gazebo/3d_prints/franka-ZED2-camera-holder.stl` | 相机支架 3D 模型 |
| `franka_zed_gazebo/launch/real_robot_zed2.launch` | 真实机器人启动文件 |
| `franka_zed_gazebo/meshes/ZED2.dae` | 相机外观模型 |

---

## 🚀 使用真实 ZED2 相机

### 必要条件

1. **硬件**：ZED2 相机
2. **驱动**：Stereolabs ZED SDK
3. **ROS 包**：`zed_ros_wrapper`

### 启动真实机器人 + 相机

```bash
# 使用专门的 launch 文件
roslaunch franka_zed_gazebo real_robot_zed2.launch
```

### 获取图像

```python
import rospy
from sensor_msgs.msg import Image

def image_callback(msg):
    # msg 是 RGB 图像
    # 进行检测、识别等
    pass

rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, image_callback)
```

---

## 📝 相机标定

### 标定参数位置

ZED2 的出厂标定参数通常包含在相机内存中。ROS 使用时从以下位置读取：

```
/zed2/zed_node/left/camera_info     # 左相机内参
/zed2/zed_node/right/camera_info    # 右相机内参
/zed2/zed_node/rgb/camera_info      # RGB 相机内参
```

### 获取标定信息

```bash
# 查看左相机标定
rostopic echo /zed2/zed_node/left/camera_info

# 保存标定
rostopic echo /zed2/zed_node/left/camera_info > left_camera_info.txt
```

---

## 💡 建议

### 短期（仿真验证）
✅ 当前设置足够
- 相机模型已完整配置
- 可以验证抓取规划算法
- 无需真实图像

### 中期（视觉集成）
📌 添加物体检测模块
- 使用 YOLOv5/v8 检测立方体
- 发布 `/target_cube_pose`
- 运行完整的视觉-规划流程

### 长期（VLA 集成）
🎯 实现端到端学习
- 使用 VLA 模型评分候选
- 学习最优的接近方向
- 在真实机器上部署

---

**最后更新**: 2025-12-30
**相机模型**: ZED2
**当前视觉支持**: 仅硬件集成，无图像处理
