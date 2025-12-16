# API Specification - Panda Grasp Planning

完整的接口定义、坐标系规范和参数配置说明。

## 版本信息
- **版本**: 2.0.0
- **日期**: 2025-12-16
- **作者**: Yichen Feng

## 相关文档
- [README.md](../README.md) - 项目概述和快速开始
- [TESTING_GUIDE.md](TESTING_GUIDE.md) - 详细测试指南（含数据记录和可视化）

---

## 1. 接口定义（Phase 0）

### 1.1 输入接口

#### Topic: `/target_cube_pose`
**消息类型**: `geometry_msgs/PoseStamped`

**坐标系要求**: `panda_link0` (机器人基座坐标系)

**字段说明**:
```yaml
header:
  frame_id: "panda_link0"  # 必须是 panda_link0
  stamp: [当前时间]

pose:
  position:
    x: [float]  # cube center 的 x 坐标（米）
    y: [float]  # cube center 的 y 坐标（米）
    z: [float]  # cube center 或顶面中心的 z 坐标（米）
  
  orientation:
    # 可以忽略，规划模块会根据抓取策略覆盖
    x: [float]
    y: [float]
    z: [float]
    w: [float]
```

**示例**:
```python
import geometry_msgs.msg

target = geometry_msgs.msg.PoseStamped()
target.header.frame_id = "panda_link0"
target.header.stamp = rospy.Time.now()
target.pose.position.x = 0.5   # 50cm forward
target.pose.position.y = 0.0   # centered
target.pose.position.z = 0.05  # 5cm above table
# orientation 会被覆盖，无需设置
```

---

### 1.2 输出接口

#### Topic: `/planned_trajectory`
**消息类型**: `moveit_msgs/RobotTrajectory`

包含完整的关节轨迹，可直接发送给控制器执行。

**或者**

#### Action: `/execute_grasp`
**类型**: `moveit_msgs/ExecuteTrajectoryAction`

用于执行规划好的轨迹。

---

### 1.3 状态反馈

#### Topic: `/grasp_planning_status`
**消息类型**: `std_msgs/String`

实时报告规划状态：
- `"IDLE"` - 空闲
- `"PLANNING_PRE_GRASP"` - 正在规划 pre-grasp
- `"PLANNING_GRASP"` - 正在规划 grasp
- `"PLANNING_LIFT"` - 正在规划 lift
- `"EXECUTING"` - 执行中
- `"SUCCESS"` - 成功完成
- `"FAILED"` - 失败

---

## 2. 坐标系规范

### 2.1 全局规范
**所有运动规划假设目标在 `panda_link0` 坐标系下表达。**

```
panda_link0 (base frame):
  - 原点: 机器人基座中心
  - X 轴: 向前（机器人正前方）
  - Y 轴: 向左
  - Z 轴: 向上
```

### 2.2 工作空间范围
建议的安全工作空间（相对于 `panda_link0`）:
```yaml
x: [0.2, 0.8]   # 20cm - 80cm 前方
y: [-0.5, 0.5]  # 左右各 50cm
z: [0.0, 0.8]   # 桌面到 80cm 高度
```

### 2.3 坐标变换
如果输入来自其他坐标系（如相机坐标系），**必须**先转换到 `panda_link0`：
```python
import tf2_ros
import tf2_geometry_msgs

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# 假设输入在 camera_frame
pose_in_camera = ...

# 转换到 panda_link0
pose_in_base = tf_buffer.transform(
    pose_in_camera, 
    "panda_link0",
    timeout=rospy.Duration(1.0)
)
```

---

## 3. 抓取姿态生成规则（Phase 2）

### 3.1 抓取序列
```
home → pre_grasp → grasp → (close_gripper) → lift → place
```

### 3.2 姿态定义

#### Pre-Grasp Pose
```python
position = cube_center + [0, 0, dz_pre]  # dz_pre = 0.10m (默认)
orientation = gripper_z_down             # RPY = [π, 0, 0]
```

#### Grasp Pose
```python
position = cube_center + [0, 0, dz_grasp]  # dz_grasp = 0.02m (默认)
orientation = gripper_z_down               # RPY = [π, 0, 0]
```

#### Lift Pose
```python
position = grasp_pose.position + [0, 0, dz_lift]  # dz_lift = 0.15m (默认)
orientation = gripper_z_down                      # RPY = [π, 0, 0]
```

### 3.3 方向约定
**Gripper Z-Down (夹爪朝下)**:
```python
from tf.transformations import quaternion_from_euler
import math

# Roll-Pitch-Yaw: [π, 0, 0]
q = quaternion_from_euler(math.pi, 0.0, 0.0)
# q = [x, y, z, w]
```

这使得夹爪的 z 轴朝下，适合从上方抓取物体。

---

## 4. 规划参数（Phase 1 & 3）

### 4.1 Planning Group
```yaml
planning_group: "panda_arm"
# 包含 7 个关节: panda_joint1 ~ panda_joint7
# 不包含 gripper joints
```

### 4.2 规划器配置
```yaml
planner_id: "RRTConnect"
planning_time: 10.0              # 最大规划时间（秒）
num_planning_attempts: 10         # 规划尝试次数
```

### 4.3 执行参数
```yaml
max_velocity_scaling_factor: 0.5      # 速度缩放（0.0 - 1.0）
max_acceleration_scaling_factor: 0.5  # 加速度缩放（0.0 - 1.0）
```

### 4.4 Home Configuration
```python
home_joints = [
    0.0,           # joint1
    -π/4,          # joint2
    0.0,           # joint3
    -3π/4,         # joint4
    0.0,           # joint5
    π/2,           # joint6
    π/4            # joint7
]
```

---

## 5. 错误处理

### 5.1 规划失败原因
| 错误类型 | 可能原因 | 处理策略 |
|---------|---------|---------|
| `IK_FAILED` | 目标姿态不可达 | 调整目标位置或方向 |
| `COLLISION` | 路径与障碍物碰撞 | 添加/更新 collision objects |
| `TIMEOUT` | 规划时间超时 | 增加 `planning_time` |
| `INVALID_MOTION` | 关节限制违反 | 检查目标是否在工作空间内 |

### 5.2 重试策略
```python
max_retries = 3
for attempt in range(max_retries):
    success = plan_and_execute(target)
    if success:
        break
    rospy.logwarn(f"Attempt {attempt+1} failed, retrying...")
    rospy.sleep(1.0)
```

---

## 6. 性能评估指标（Phase 3）

### 6.1 核心指标
```yaml
metrics:
  success_rate:          # 成功率（0.0 - 1.0）
    definition: "所有阶段都成功规划并执行的比例"
    
  avg_planning_time:     # 平均规划时间（秒）
    definition: "单个阶段的平均规划时间"
    
  avg_path_length:       # 平均路径长度
    definition: "关节空间弧长 Σ√(Δq_i²)"
    
  failure_modes:         # 失败模式统计
    - IK_FAILED
    - COLLISION
    - TIMEOUT
    - EXECUTION_FAILED
```

### 6.2 测试协议
```python
# 测试配置
num_trials = 100
workspace_bounds = {
    'x': [0.3, 0.7],
    'y': [-0.3, 0.3],
    'z': [0.05, 0.05]  # 固定在桌面高度
}

# 对每个随机目标
for i in range(num_trials):
    target = generate_random_target(workspace_bounds)
    result = execute_full_grasp_sequence(target)
    log_result(result)
```

---

## 7. 调试与可视化

### 7.1 RViz 配置
显示以下内容：
- `PlanningScene` - 碰撞环境
- `Trajectory` - 规划路径
- `RobotModel` - 机器人状态
- `TF` - 坐标系

### 7.2 日志级别
```python
rospy.logdebug("详细调试信息")
rospy.loginfo("正常操作信息")
rospy.logwarn("警告（非致命）")
rospy.logerr("错误（操作失败）")
```

### 7.3 关键日志点
- 目标姿态接收
- IK 求解结果
- 规划开始/结束
- 执行开始/结束
- 失败原因

---

## 8. 示例代码

### 8.1 发送目标姿态
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('send_target')
pub = rospy.Publisher('/target_cube_pose', PoseStamped, queue_size=1)
rospy.sleep(1.0)

target = PoseStamped()
target.header.frame_id = "panda_link0"
target.header.stamp = rospy.Time.now()
target.pose.position.x = 0.5
target.pose.position.y = 0.0
target.pose.position.z = 0.05

pub.publish(target)
rospy.loginfo("Target sent!")
```

### 8.2 监听状态
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def status_callback(msg):
    rospy.loginfo(f"Status: {msg.data}")

rospy.init_node('status_monitor')
rospy.Subscriber('/grasp_planning_status', String, status_callback)
rospy.spin()
```

---

## 9. 常见问题

### Q1: 为什么必须使用 panda_link0？
**A**: 统一坐标系可以避免转换错误，简化调试，且 panda_link0 是机器人的自然基座坐标系。

### Q2: 可以改变夹爪方向吗？
**A**: 可以，修改 `config/grasp_params.yaml` 中的 `gripper_orientation`。

### Q3: 规划总是失败怎么办？
**A**: 
1. 检查目标是否在工作空间内
2. 增加 `planning_time` 
3. 检查 collision objects 是否正确
4. 尝试不同的起始配置

### Q4: 如何提高规划速度？
**A**:
1. 减少 `num_planning_attempts`
2. 调整 RRTConnect 的 `range` 参数
3. 使用更快的规划器（如 RRT）

---

## 10. 更新日志

| 版本 | 日期 | 更新内容 |
|-----|------|---------|
| 1.0.0 | 2025-12-03 | 初始版本，定义 Phase 0-3 接口 |

---

## 联系方式

如有问题或建议，请联系：
- **Email**: yichenfeng630@todo.com
- **Repository**: Franka_gripping
