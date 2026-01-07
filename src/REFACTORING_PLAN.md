# Franka Gripping System - 模块化重构计划

**创建时间**: 2026-01-07  
**目标**: 将panda_grasp_planning中的复杂代码拆分为4个独立的ROS包

---

## 🎯 重构目标

将当前集中在`panda_grasp_planning`的代码，按功能拆分为以下4个模块化包：

| 包名 | 职责 | 主要功能 |
|-----|------|---------|
| **franka_task_planning** | VLA上层控制 | 语言指令解析、任务规划、决策制定 |
| **franka_perception** | 视觉定位 | ZED2相机处理、物体检测、点云处理 |
| **franka_grasp_generation** | 抓取姿态生成 | 候选抓取姿态计算、IK求解、碰撞检测 |
| **franka_trajectory_planning** | 轨迹规划执行 | MoveIt轨迹规划、Cartesian路径、gripper控制 |

---

## 📦 当前代码分析

### 主要文件
1. **scripts/v4_demo.py** (1262行)
   - 完整的抓取演示流程
   - 包含感知、规划、执行全流程
   - 需拆分为各模块

2. **scripts/grasp_pipeline_v3.py**
   - V3版本的抓取管道
   - 多方向接近 + 分层重试
   - 需拆分执行逻辑

3. **modules/** 目录
   - `perception/` - 感知相关
   - `vla/` - VLA推理
   - `action/` - 动作执行器
   - `candidate_generation/` - 候选生成
   - `sorting/` - 分类状态机

---

## 🏗️ 新包结构

### 1. franka_perception (视觉定位)

**路径**: `/opt/ros_ws/src/franka_perception/`

```
franka_perception/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── zed2_perception.launch
│   └── sim_perception.launch
├── config/
│   ├── camera_params.yaml
│   └── detection_params.yaml
├── nodes/
│   ├── perception_node.py          # 主感知节点
│   ├── rgb_detector.py             # RGB颜色检测
│   └── pointcloud_processor.py     # 点云处理
└── src/
    └── franka_perception/
        ├── __init__.py
        ├── object_detector.py
        └── pose_estimator.py
```

**ROS接口**:
- **发布话题**:
  - `/detected_objects` (String/JSON) - 检测到的物体列表
  - `/object_pose` (PoseStamped) - 单个物体姿态
  - `/detection_status` (String) - 检测状态
- **参数**:
  - `target_color` - 目标颜色
  - `confidence_threshold` - 置信度阈值
  - `sim_mode` - 仿真模式开关

---

### 2. franka_grasp_generation (抓取姿态生成)

**路径**: `/opt/ros_ws/src/franka_grasp_generation/`

```
franka_grasp_generation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── grasp_generation.launch
├── config/
│   ├── grasp_params.yaml
│   └── gripper_params.yaml
├── nodes/
│   └── grasp_generator_node.py     # 抓取候选生成节点
└── src/
    └── franka_grasp_generation/
        ├── __init__.py
        ├── candidate_generator.py   # 从modules/candidate_generation迁移
        ├── collision_checker.py
        └── ik_solver.py
```

**ROS接口**:
- **服务**:
  - `/generate_grasps` (GenerateGrasps.srv) - 生成抓取候选
    - Request: object_pose, constraints
    - Response: grasp_candidates[]
- **参数**:
  - `approach_directions` - 接近方向数量
  - `yaw_samples` - yaw角采样数
  - `collision_check_enabled` - 碰撞检测开关

---

### 3. franka_trajectory_planning (轨迹规划执行)

**路径**: `/opt/ros_ws/src/franka_trajectory_planning/`

```
franka_trajectory_planning/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── trajectory_executor.launch
├── config/
│   ├── moveit_params.yaml
│   └── gripper_params.yaml
├── nodes/
│   ├── trajectory_executor_node.py  # 轨迹执行节点
│   └── gripper_controller_node.py   # 夹爪控制节点
└── src/
    └── franka_trajectory_planning/
        ├── __init__.py
        ├── action_executor.py        # 从modules/action迁移
        ├── cartesian_planner.py
        └── gripper_interface.py
```

**ROS接口**:
- **服务**:
  - `/execute_trajectory` (ExecuteTrajectory.srv)
    - Request: waypoints[], velocity_scaling
    - Response: success, execution_time
  - `/execute_grasp` (ExecuteGrasp.srv)
    - Request: grasp_pose, pre_grasp_height, lift_height
    - Response: success, gripper_width
- **动作**:
  - `/execute_pick_place` (PickPlaceAction)
- **参数**:
  - `max_velocity_scaling` - 最大速度比例
  - `cartesian_step_size` - Cartesian步长

---

### 4. franka_task_planning (VLA上层控制)

**路径**: `/opt/ros_ws/src/franka_task_planning/`

```
franka_task_planning/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── vla_planner.launch
├── config/
│   └── vla_params.yaml
├── nodes/
│   ├── vla_adapter_node.py          # VLA适配器节点
│   └── task_coordinator_node.py     # 任务协调器
└── src/
    └── franka_task_planning/
        ├── __init__.py
        ├── vla_inference.py          # 从modules/vla迁移
        ├── language_parser.py
        └── task_planner.py
```

**ROS接口**:
- **服务**:
  - `/parse_instruction` (ParseInstruction.srv)
    - Request: language_instruction, scene_image
    - Response: target_object, action_type
  - `/rank_grasps` (RankGrasps.srv)
    - Request: grasp_candidates[], scene_context
    - Response: ranked_indices[], scores[]
- **参数**:
  - `model_checkpoint` - VLA模型路径
  - `temperature` - 采样温度
  - `use_lora` - LoRA微调开关

---

## 🔄 数据流

```
语言指令 → [franka_task_planning] → 目标选择
                                        ↓
相机图像 → [franka_perception] → 物体位置
                                        ↓
                            [franka_grasp_generation] → 抓取候选
                                        ↓
                            [franka_trajectory_planning] → 执行抓取
```

---

## 📋 迁移步骤

### 阶段1: 创建包骨架
- [x] 创建4个新ROS包的基本结构
- [x] 配置CMakeLists.txt和package.xml
- [x] 创建launch文件模板

### 阶段2: 代码迁移
- [ ] 迁移perception代码到franka_perception
- [ ] 迁移grasp generation代码到franka_grasp_generation
- [ ] 迁移trajectory planning代码到franka_trajectory_planning
- [ ] 迁移VLA代码到franka_task_planning

### 阶段3: 接口集成
- [ ] 定义ROS服务消息
- [ ] 创建统一launch文件
- [ ] 配置topic映射

### 阶段4: 测试验证
- [ ] 单元测试每个模块
- [ ] 集成测试完整流程
- [ ] 回归测试v4_demo功能

---

## 🎓 设计原则

1. **单一职责**: 每个包只负责一个核心功能
2. **松耦合**: 通过ROS接口通信，减少直接依赖
3. **可测试性**: 每个模块可独立测试
4. **向后兼容**: 保留原有的launch文件作为集成入口
5. **可扩展性**: 便于后续添加新功能（如学习型控制器）

---

## 📝 保留内容

**panda_grasp_planning** 保留为集成包：
- 保留launch/panda_grasp_complete.launch作为总入口
- 保留config/配置文件
- scripts/v4_demo.py重写为轻量级协调器
- 保留test_results/和文档

---

## ⚠️ 注意事项

1. **分步迁移**: 逐个模块迁移，每次完成后测试
2. **保留原代码**: 迁移过程中不删除原文件，标记为deprecated
3. **文档同步**: 每个新包必须包含README说明接口和用法
4. **依赖管理**: 注意循环依赖，perception和grasp_generation不应相互依赖

---

## 🎯 完成标准

- ✅ 4个新包编译通过
- ✅ roslaunch启动无错误
- ✅ v4_demo的100%成功率保持不变
- ✅ 代码行数: 每个包 < 500行（相比原来1262行的单文件）
- ✅ 每个包有独立README和测试脚本
