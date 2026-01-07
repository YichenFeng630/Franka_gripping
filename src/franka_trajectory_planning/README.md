# franka_trajectory_planning

Franka机器人轨迹规划与执行模块 - 基于MoveIt的运动控制和夹爪操作

## 功能概述

该包负责执行机器人的运动规划和控制，包括：
- Cartesian路径规划
- 关节空间轨迹规划
- 夹爪控制（开合、抓取）
- Pick-and-place动作序列
- 速度和加速度控制

## 核心功能

### 1. 轨迹执行
- MoveIt运动规划
- Cartesian直线运动
- 关节插值运动
- 实时状态反馈

### 2. 夹爪控制
- 位置控制（开合宽度）
- 力控制（抓取力度）
- 状态查询（宽度反馈）

### 3. 高级动作
- Grasp序列（预抓取→下降→夹取→提升）
- Place序列（接近→放置→松开→撤退）
- 紧急停止和安全保护

## ROS接口

### 服务（Services）

#### /execute_grasp
执行完整的抓取序列
```
Request:
  geometry_msgs/Pose grasp_pose
  float32 pre_height           # 预抓取高度（默认：0.15m）
  float32 lift_height          # 提升高度（默认：0.50m）
  float32 descent_velocity     # 下降速度（默认：0.05）
  float32 gripper_width        # 闭合宽度（默认：0.043m）
  float32 gripper_force        # 抓取力（默认：90N）

Response:
  bool success
  float32 gripper_width        # 实际夹爪宽度
  float32 execution_time       # 执行时间（秒）
  string message
```

#### /execute_place
执行放置序列
```
Request:
  geometry_msgs/Pose target_pose
  float32 approach_height      # 接近高度（默认：0.20m）
  float32 retreat_height       # 撤退高度（默认：0.30m）
  
Response:
  bool success
  float32 execution_time
  string message
```

#### /execute_trajectory
执行自定义轨迹
```
Request:
  geometry_msgs/Pose[] waypoints
  float32 velocity_scaling     # 速度比例（默认：1.0）
  float32 acceleration_scaling # 加速度比例（默认：1.0）
  bool cartesian_path          # 是否使用Cartesian插值

Response:
  bool success
  float32 execution_time
  string message
```

#### /control_gripper
控制夹爪开合
```
Request:
  string command               # "open", "close", "move", "grasp"
  float32 width                # 目标宽度（米，用于move）
  float32 force                # 抓取力（牛顿，用于grasp）
  float32 speed                # 运动速度（默认：0.05）

Response:
  bool success
  float32 current_width        # 当前宽度
  string message
```

### 话题（Topics）

#### 发布（Publish）
- `/execution_status` (std_msgs/String) - 实时执行状态
- `/gripper_state` (sensor_msgs/JointState) - 夹爪状态

#### 订阅（Subscribe）
- `/emergency_stop` (std_msgs/Bool) - 紧急停止信号

### 动作（Actions）

#### /execute_pick_place
高级pick-and-place动作
```
Goal:
  geometry_msgs/Pose pick_pose
  geometry_msgs/Pose place_pose
  PickPlaceConfig config
  
Result:
  bool success
  float32 total_time
  PickPlaceStats stats
  
Feedback:
  string current_stage         # "APPROACH", "GRASP", "LIFT", "PLACE", etc.
  float32 progress             # 0.0 - 1.0
```

### 参数（Parameters）

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `~max_velocity_scaling` | float | 1.0 | 最大速度比例 |
| `~max_acceleration_scaling` | float | 1.0 | 最大加速度比例 |
| `~cartesian_step_size` | float | 0.0025 | Cartesian步长（米）|
| `~planning_time` | float | 5.0 | 规划超时（秒）|
| `~num_planning_attempts` | int | 10 | 规划尝试次数 |
| `~gripper_open_width` | float | 0.08 | 夹爪打开宽度（米）|
| `~gripper_close_width` | float | 0.043 | 夹爪闭合宽度（米）|
| `~gripper_force` | float | 90.0 | 抓取力（牛顿）|
| `~dwell_time` | float | 2.0 | 稳定等待时间（秒）|
| `~enable_collision_check` | bool | true | 启用碰撞检测 |

## 使用方法

### 1. 独立启动
```bash
roslaunch franka_trajectory_planning trajectory_executor.launch
```

### 2. 执行抓取
```python
import rospy
from franka_trajectory_planning.srv import ExecuteGrasp, ExecuteGraspRequest

rospy.wait_for_service('/execute_grasp')
execute_grasp = rospy.ServiceProxy('/execute_grasp', ExecuteGrasp)

req = ExecuteGraspRequest()
req.grasp_pose = target_grasp_pose  # 来自grasp_generation
req.pre_height = 0.15
req.lift_height = 0.50

resp = execute_grasp(req)
if resp.success:
    print(f"Grasp succeeded! Gripper width: {resp.gripper_width:.3f}m")
else:
    print(f"Grasp failed: {resp.message}")
```

### 3. 使用Action（异步执行）
```python
import actionlib
from franka_trajectory_planning.msg import PickPlaceAction, PickPlaceGoal

client = actionlib.SimpleActionClient('/execute_pick_place', PickPlaceAction)
client.wait_for_server()

goal = PickPlaceGoal()
goal.pick_pose = pick_pose
goal.place_pose = place_pose

client.send_goal(goal, feedback_cb=lambda fb: print(fb.current_stage))
client.wait_for_result()
result = client.get_result()
```

## 抓取序列详解

### Grasp Sequence（7步）
```
1. OPEN_GRIPPER    : 打开夹爪到80mm
2. MOVE_TO_PRE     : 移动到预抓取位置（hover above object）
3. DESCEND         : 垂直下降到抓取位置（慢速 0.05m/s）
4. DWELL           : 稳定等待（2-3秒，消除振动）
5. CLOSE_GRIPPER   : 闭合夹爪（43mm，90N力）
6. LIFT            : 提升到安全高度（50cm）
7. VERIFY          : 验证抓取成功（检查夹爪宽度40-48mm）
```

### Place Sequence（5步）
```
1. APPROACH        : 移动到放置区域上方
2. DESCEND         : 下降到目标位置
3. OPEN_GRIPPER    : 松开夹爪
4. RETREAT         : 撤退到安全高度
5. CLOSE_GRIPPER   : 闭合夹爪（准备下次抓取）
```

## 速度控制策略

| 阶段 | 速度比例 | 加速度比例 | 说明 |
|------|----------|------------|------|
| 移动到预抓取 | 1.0 | 1.0 | 正常速度 |
| 下降抓取 | 0.05 | 0.05 | 极慢，避免碰撞 |
| 提升 | 0.5 | 0.5 | 中速，避免甩动 |
| 放置下降 | 0.2 | 0.2 | 慢速，温柔放置 |
| 撤退 | 0.8 | 0.8 | 较快返回 |

## 依赖

- ROS Noetic
- MoveIt
- moveit_commander (Python)
- franka_ros
- franka_gripper
- actionlib

## 文件结构

```
franka_trajectory_planning/
├── nodes/
│   ├── trajectory_executor_node.py   # 主执行节点
│   └── gripper_controller_node.py    # 夹爪控制节点
├── src/franka_trajectory_planning/
│   ├── __init__.py
│   ├── moveit_interface.py           # MoveIt接口封装
│   ├── gripper_interface.py          # 夹爪接口封装
│   ├── cartesian_planner.py          # Cartesian路径规划
│   └── safety_monitor.py             # 安全监控
├── srv/
│   ├── ExecuteGrasp.srv
│   ├── ExecutePlace.srv
│   ├── ExecuteTrajectory.srv
│   └── ControlGripper.srv
├── action/
│   └── PickPlace.action
├── launch/
│   ├── trajectory_executor.launch
│   └── test_gripper.launch
├── config/
│   └── execution_params.yaml
└── README.md
```

## 性能指标

| 指标 | 值 |
|------|-----|
| 规划时间 | < 3秒 |
| 执行精度 | < 2mm（XYZ） |
| 抓取成功率 | > 95% |
| 平均抓取时间 | 15-20秒 |

## 安全特性

### 1. 碰撞检测
- 实时场景更新
- 自碰撞检查
- 环境碰撞检查

### 2. 限位保护
- 关节角度限位
- 笛卡尔空间工作区限位
- 速度/加速度限制

### 3. 力觉反馈
- 夹爪力控制
- 接触检测
- 过载保护

### 4. 紧急停止
```bash
# 发布紧急停止信号
rostopic pub /emergency_stop std_msgs/Bool "data: true" -1
```

## 故障排除

### 问题1: 规划失败
- 检查目标姿态可达性
- 确认场景中无碰撞
- 增加`num_planning_attempts`
- 尝试不同的起始姿态

### 问题2: 轨迹执行抖动
- 降低`max_velocity_scaling`
- 增加`dwell_time`
- 检查控制器参数

### 问题3: 夹爪未正确闭合
- 检查action server连接
- 确认夹爪控制器运行
- 调整`gripper_force`和`gripper_width`

## 开发者

- **维护者**: Yichen Feng
- **创建日期**: 2026-01-07
- **版本**: 1.0.0

## License

BSD 3-Clause

## 参考

- V4 Demo原始实现: `panda_grasp_planning/scripts/v4_demo.py`
- Action Executor: `panda_grasp_planning/modules/action/action_executor.py`
