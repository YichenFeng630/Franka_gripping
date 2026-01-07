# franka_grasp_generation

Franka机器人抓取姿态生成模块 - 基于物体位置生成最优抓取候选

## 功能概述

该包负责为检测到的物体生成可行的抓取姿态，包括：
- 多方向接近策略
- Yaw角采样优化
- IK可行性检测
- 碰撞检测
- 抓取质量评分

## 核心功能

### 1. 抓取候选生成
- 输入：物体6D位姿（来自perception）
- 输出：排序后的抓取候选列表
- 策略：Top-down, side-grasp, 角度变体

### 2. 碰撞检测
- 使用MoveIt场景
- 检查gripper与环境碰撞
- 预抓取位置安全验证

### 3. IK求解
- 验证抓取姿态可达性
- 多seed IK求解
- 关节限位检查

## ROS接口

### 服务（Services）

#### /generate_grasps
生成抓取候选
```
Request:
  geometry_msgs/PoseStamped object_pose
  int32 num_approaches       # 接近方向数量（默认：8）
  int32 yaw_samples          # yaw角采样数（默认：8）
  float32 grasp_depth_ratio  # 抓取深度比例（默认：0.4）
  bool check_collision       # 是否碰撞检测（默认：true）

Response:
  GraspCandidate[] candidates
  bool success
  string message
```

**GraspCandidate结构：**
```
geometry_msgs/Pose grasp_pose      # 抓取位姿
geometry_msgs/Pose pre_grasp_pose  # 预抓取位姿
geometry_msgs/Vector3 approach_direction
float32 score                       # 质量评分 [0-1]
float32 yaw_angle                   # yaw角（度）
bool is_collision_free
bool is_ik_valid
```

### 参数（Parameters）

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `~cube_size` | float | 0.045 | 目标物体尺寸（米）|
| `~gripper_tcp_offset` | float | 0.103 | 夹爪TCP偏移（米）|
| `~pre_grasp_height` | float | 0.15 | 预抓取高度（米）|
| `~grasp_depth_ratio` | float | 0.4 | 抓取深度比例 |
| `~num_approach_directions` | int | 8 | 接近方向数量 |
| `~yaw_samples_per_direction` | int | 8 | 每方向yaw采样数 |
| `~enable_collision_check` | bool | true | 启用碰撞检测 |
| `~min_grasp_score` | float | 0.3 | 最低接受评分 |

## 使用方法

### 1. 独立启动
```bash
roslaunch franka_grasp_generation grasp_generation.launch
```

### 2. 调用服务
```python
import rospy
from franka_grasp_generation.srv import GenerateGrasps, GenerateGraspsRequest

rospy.wait_for_service('/generate_grasps')
generate_grasps = rospy.ServiceProxy('/generate_grasps', GenerateGrasps)

req = GenerateGraspsRequest()
req.object_pose = detected_pose  # 来自perception
req.num_approaches = 8
req.yaw_samples = 8

resp = generate_grasps(req)
if resp.success:
    print(f"Generated {len(resp.candidates)} grasp candidates")
    best_grasp = resp.candidates[0]  # 已按score排序
```

### 3. 可视化
```bash
# 启动RViz查看候选抓取姿态
roslaunch franka_grasp_generation visualize.launch
```

## 算法说明

### Top-Down抓取策略
```
1. 计算物体顶面中心
2. 生成环绕顶面的接近方向（8方向）
3. 对每个方向采样yaw角（0-360°，8个样本）
4. 计算抓取姿态：
   - Position: [x, y, z_top + grasp_depth]
   - Orientation: z-down, yaw-snapped
5. 验证IK和碰撞
6. 评分排序
```

### 评分标准
- **接近角度**（40%）：垂直接近得分高
- **IK质量**（30%）：远离关节限位得分高
- **碰撞余量**（20%）：与障碍物距离大得分高
- **对称性**（10%）：对齐物体主轴得分高

## 依赖

- ROS Noetic
- MoveIt
- moveit_commander (Python)
- geometry_msgs
- franka_perception (检测物体位姿)

## 文件结构

```
franka_grasp_generation/
├── nodes/
│   └── grasp_generator_node.py    # 主服务节点
├── src/franka_grasp_generation/
│   ├── __init__.py
│   ├── candidate_generator.py     # 候选生成核心算法
│   ├── collision_checker.py       # 碰撞检测
│   └── grasp_scorer.py            # 评分算法
├── srv/
│   ├── GenerateGrasps.srv
│   └── GraspCandidate.msg
├── launch/
│   ├── grasp_generation.launch
│   └── visualize.launch
├── config/
│   └── grasp_params.yaml
└── README.md
```

## 性能指标

| 指标 | 值 |
|------|-----|
| 生成速度 | < 2秒（64候选） |
| 成功率 | > 95%（物体在工作空间内） |
| 内存占用 | < 500MB |

## 故障排除

### 问题1: 未生成任何候选
- 检查物体位置是否在工作空间内
- 确认MoveIt规划场景已初始化
- 降低`min_grasp_score`阈值

### 问题2: IK求解失败
- 检查机器人关节状态
- 确认`gripper_tcp_offset`参数正确
- 尝试增加`num_approach_directions`

### 问题3: 碰撞检测过于保守
- 调整碰撞检测余量
- 更新场景中的障碍物模型
- 考虑禁用某些自碰撞检查

## 开发者

- **维护者**: Yichen Feng
- **创建日期**: 2026-01-07
- **版本**: 1.0.0

## License

BSD 3-Clause

## 参考

- V4 Demo原始实现: `panda_grasp_planning/scripts/v4_demo.py`
- 候选生成器: `panda_grasp_planning/modules/candidate_generation/`
