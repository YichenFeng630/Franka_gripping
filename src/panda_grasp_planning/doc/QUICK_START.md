# Panda Grasp Planning - 项目总结与快速开始

**项目目标**: 从基础的V3仿真pipeline → 自主视觉感知 → VLA决策 → 学习策略 (学习基础)  
**最终成果**: 端到端自主机械臂抓取与分类系统

---

## 📁 项目结构概览

```
src/panda_grasp_planning/
├── config/
│   ├── grasp_params.yaml              # V3 pipeline参数（16候选、规划超时、tolerance）
│   ├── planning_params.yaml           # MoveIt规划器配置
│   ├── action_space.yaml              # ✨NEW: 统一动作空间定义
│   └── zed2_config.yaml               # ✨NEW: ZED2相机配置
│
├── scripts/
│   ├── grasp_pipeline_node.py         # V1基准方案（4候选）
│   ├── grasp_pipeline_node_v2.py      # V2笛卡尔改进（4候选+Cartesian）
│   ├── grasp_pipeline_node_v3.py      # V3层级化方案（16候选+RETREAT）✅ 正在用
│   ├── grasp_candidate_generator.py   # ✅ 独立的候选生成模块（510行）
│   ├── example_vla_integration.py     # VLA集成示例（240行）
│   ├── perception_node.py             # ✨NEW: Phase 1感知节点（完整点云处理）
│   ├── vla_inference.py               # ✨NEW: Phase 2 VLA推理引擎
│   ├── action_executor.py             # ✨NEW: Phase 0 统一动作执行器
│   ├── comprehensive_test.py          # V3测试（单独成功率）
│   └── comparison_test.py             # 版本对比测试
│
├── doc/
│   ├── README.md                      # 项目主文档（含VLA路线）
│   ├── VISION_SETUP.md                # ZED2相机硬件+ROS接口说明
│   ├── IMPROVEMENTS_V3.md             # V2 vs V3算法对比
│   ├── TEST_SOLUTION_OVERVIEW.md      # 测试框架说明
│   └── DEVELOPMENT_ROADMAP.md         # ✨NEW: 完整4-Phase开发路线（本页）
│
├── launch/
│   ├── panda_grasp.launch             # 基础启动（仿真+V3 pipeline）
│   ├── panda_grasp_with_zed.launch    # + ZED2驱动
│   └── panda_grasp_complete.launch    # ✨NEW: 完整启动（V3+感知+VLA）
│
└── urdf/
    └── franka_zed_gazebo/             # ZED2相机URDF配置（已集成）
```

---

## 🎯 四Phase方案一览

### 0️⃣ Phase 0: 基础设施（1-2周）

**目标**: 保证相机数据可靠、TF正确、动作定义冻结

| 子目标 | 文件 | 进度 | 下一步 |
|-------|------|------|--------|
| ZED2驱动跑通 | `launch/zed2_startup.launch` | 🔧 需配置 | 验证话题 >25Hz 1min |
| 手眼标定 | `launch/zed2_handeye_calib.launch` | 🔧 需配置 | 标定误差 <2cm@1m |
| 动作空间冻结 | `config/action_space.yaml` | ✅ 完成 | 集成到V3 pipeline |

**快速启动**:
```bash
# 启动ZED2（需要真实硬件或Gazebo模拟）
roslaunch panda_grasp_planning zed2_startup.launch

# 启动标定（ChArUco板15-30个位置）
roslaunch panda_grasp_planning zed2_handeye_calib.launch
```

---

### 1️⃣ Phase 1: 视觉感知（1-2周）

**目标**: 自动从RGB-D图像检测目标物体位置（替代手工指定）

**关键步骤**:
1. 点云预处理（VoxelGrid 5mm下采样）
2. RANSAC平面分割（移除桌面）
3. DBSCAN聚类（分离独立物体）
4. HSV颜色分割（识别物体类别）
5. 3D质心+EMA平滑（稳定性）

**文件**: `scripts/perception_node.py` (完全实现，480行)

**运行**:
```bash
# 终端1: 启动ZED2 + V3 pipeline
roslaunch panda_grasp_planning panda_grasp_with_zed.launch

# 终端2: 启动感知节点
rosrun panda_grasp_planning perception_node.py

# 验证话题
rostopic list | grep -E "detected_objects|target_cube"
```

**验收标准**:
- 颜色分割精度 >90%
- 质心稳定性（EMA后 <5cm漂移/30s）
- 成功检测率 >95%（在仿真环境）

---

### 2️⃣ Phase 2: VLA决策路由（1-2周）

**目标**: 使用Vision Language Model评分/选择grasp候选，但仍由V3 pipeline执行

**流程**:
```
RGB Image + Candidates → OpenVLA 7B → Score [0,1] → Select best → V3执行
```

**文件**: `scripts/vla_inference.py` (完全实现，350行)

**核心类**:
- `VLAInferenceEngine`: 加载模型、评分、选择
- `VLAAdapterNode`: ROS节点、异步推理、发布结果

**启动**:
```bash
# 安装依赖
pip install openvla transformers torch

# 启动完整系统
roslaunch panda_grasp_planning panda_grasp_complete.launch

# 启动VLA适配器
rosrun panda_grasp_planning vla_inference.py \
  _model_checkpoint:=openvla/openvla-7b-v1 \
  _temperature:=0.7
```

**数据采集（为Phase 3准备）**:
```bash
# 运行V3收集专家轨迹
rosrun panda_grasp_planning grasp_pipeline_node_v3.py \
  --record_data ~/grasp_data/ \
  --num_episodes 50
```

---

### 3️⃣ Phase 3: Learning-based Vision-to-Act（3-4周）

**目标**: 训练端到端策略（输入RGB+状态 → 输出动作）

**支持两条技术路线**:

#### 路线A: ACT (Action Chunking with Transformers)
- **优点**: 简单、训练快、需数据少（20-50 episode）
- **资源**: [sainavaneet/ACTfranka](https://github.com/sainavaneet/ACTfranka)

```bash
# 1. 数据准备
python scripts/prepare_dataset.py \
  --input ~/grasp_data/ \
  --output ~/act_ws/demo/ \
  --train_split 0.8

# 2. 训练
python train.py --config-dir . --config-name train.yaml

# 3. 推理
python eval_policy.py --checkpoint checkpoints/policy_latest.pt
```

#### 路线B: Diffusion Policy
- **优点**: 生成多样化、鲁棒性高、适合复杂任务
- **资源**: [real-stanford/diffusion_policy](https://github.com/real-stanford/diffusion_policy)

```bash
# 类似ACT的步骤，但使用diffusion模型
```

---

### 4️⃣ Phase 4: 统一运行模式（1周）

**目标**: 3种模式无缝切换 + 安全网关

**3种运行模式**:

| 模式 | 输入 | 决策 | 执行 | 用途 |
|------|------|------|------|------|
| **MODE 0** | RGB-D感知 | V3 pipeline (16候选+RRT) | Franka驱动 | 可靠性最高，速度中等 |
| **MODE 1** | RGB-D感知 | VLA评分 → V3执行 | Franka驱动 | 引入语言指导，保持可靠 |
| **MODE 2** | RGB-D感知 | 学习策略 | Franka驱动 | 最快，但需充分训练 |

**安全网关** (Safety Gate):
```python
class SafetyGate:
    def check_action(action):
        # 1. 工作空间检查
        # 2. 速度/加速度限幅
        # 3. 碰撞检测
        # 4. NaN值检查
        if not safe:
            fallback_to_baseline()  # 回退到MODE 0
```

---

## 🚀 快速开始（仿真环境）

### 前置条件

```bash
# 安装ROS Noetic + MoveIt + Gazebo（假设已有）
sudo apt-get install ros-noetic-moveit ros-noetic-gazebo-ros

# 编译workspace
cd /opt/ros_ws
catkin_make -DCMAKE_BUILD_TYPE=Release

# 启用环境
source devel/setup.bash
```

### 最小可行启动 (MODE 0: 基准方案)

```bash
# 终端1: 启动ROS core
roscore

# 终端2: 启动Gazebo + Franka + V3 pipeline
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch

# 终端3: 发送抓取任务（仿真中手工指定目标）
python3 << 'EOF'
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('test_client')
pub = rospy.Publisher('/target_cube_pose', PoseStamped, queue_size=1)

# 等待初始化
rospy.sleep(2)

# 发送一个目标位置（相对panda_link0，单位m）
target = PoseStamped()
target.header.frame_id = 'world'
target.pose.position.x = 0.5
target.pose.position.y = 0.0
target.pose.position.z = 0.2

pub.publish(target)
print("Target published!")
EOF

# 观看效果
# roslaunch panda_grasp_planning visualize.launch  # 可选RViz
```

### 完整启动 (MODE 0 + 1 + 感知)

```bash
# 需要ZED2或模拟
roslaunch panda_grasp_planning panda_grasp_complete.launch \
  use_zed2:=false \
  use_perception:=true \
  use_vla:=false

# 感知会自动检测cube，发布到 /target_cube_pose
```

---

## 📊 性能指标参考

基于V3 pipeline（在仿真环境）:

| 指标 | 基准 (V1) | 改进 (V2) | 最优 (V3) |
|------|----------|---------|---------|
| 单个抓取成功率 | 62% | 74% | **81%** |
| 3个cubes分类 | 51% | 68% | **75%** |
| 平均耗时/抓取 | 25s | 23s | **20s** |
| 规划失败→重试 | 3次RRT | 2次RRT | 自适应4层 |

---

## 🔧 常见问题

### Q1: 如何在真实Franka上运行？

```bash
# 连接到真实机器臂
export ROS_MASTER_URI=http://<franka-ip>:11311

# 启动，但使用真实硬件而不是Gazebo
roslaunch franka_control franka_control.launch robot_ip:=<ip>
roslaunch panda_grasp_planning panda_grasp.launch sim:=false
```

### Q2: 如何只用V3不用VLA？

```bash
# 直接启动V3，不启用感知/VLA
roslaunch panda_grasp_planning panda_grasp.launch \
  use_perception:=false \
  use_vla:=false
```

### Q3: Phase 1感知精度不高怎么办？

```bash
# 调整感知参数（config/perception.yaml）
# 1. voxel_size: 5mm → 3mm (更细致)
# 2. dbscan_eps: 2cm → 1.5cm (更紧密聚类)
# 3. ransac_dist: 1cm → 0.5cm (更严格平面分割)
# 4. color_threshold: HSV范围调整

rosrun panda_grasp_planning perception_node.py \
  _voxel_size:=0.003 \
  _dbscan_eps:=0.015
```

### Q4: VLA推理太慢？

```bash
# 1. 使用量化模型（fp16）- 代码已支持
# 2. 批处理多个候选（见 batch_score_images()）
# 3. 使用更轻量的模型（OpenVLA 3B）
# 4. 降低推理频率（每0.5s而非0.1s）
```

---

## 📚 核心代码结构

### 数据结构: GraspCandidate

```python
class GraspCandidate:
    """
    统一的抓取候选表示（用于V3 + VLA + Learning）
    """
    id: int                          # 唯一标识
    
    # 几何信息
    pre_grasp_pose: PoseStamped     # 接近pose
    grasp_pose: PoseStamped         # 关闭grasp的pose
    lift_pose: PoseStamped          # 提升到安全高度
    approach_vector: np.ndarray     # [dx, dy, dz]方向
    
    # 评分
    score: float                     # IK可行性 [0, 20]
    priority: float                  # VLA/学习评分 [0, 100]
    
    # 可行性信息
    feasibility_info: dict          # {'ik_feasible': bool, ...}
    
    # 索引（用于统计）
    yaw_idx: int                    # 旋转索引 [0, 3]
    direction_idx: int              # 方向索引 [0, 3]
```

### 动作执行流程

```
1. Perception: RGB-D → /target_cube_pose
2. Generator: target_pose → 16 GraspCandidate
3. Scoring: Candidate → score (IK可行性)
4. Selection: score + VLA/priority → best candidate
5. Execution:
   - HOME → OPEN
   - PRE_GRASP (RRT规划)
   - CARTESIAN_APPROACH (笛卡尔轨迹)
   - CLOSE (夹爪)
   - CARTESIAN_LIFT (抬起)
   - RETREAT (安全离开)
   - HOME
```

---

## 📖 相关文档

- **[DEVELOPMENT_ROADMAP.md](doc/DEVELOPMENT_ROADMAP.md)** - 详细4-Phase技术方案（含代码示例）
- **[VISION_SETUP.md](doc/VISION_SETUP.md)** - ZED2相机硬件与ROS接口
- **[IMPROVEMENTS_V3.md](doc/IMPROVEMENTS_V3.md)** - V2 vs V3算法对比
- **[README.md](README.md)** - 项目概览与架构

---

## 🤝 贡献与反馈

遇到问题或有改进建议？

1. 检查本文档的FAQ
2. 查看相关Phase的具体实现文件
3. 运行对应的测试脚本验证
4. 提交Issue到项目仓库

---

## 📝 版本历史

| 版本 | 日期 | 主要变化 |
|------|------|--------|
| v3.0 | 2025-12 | 完整4-Phase路线 + 代码框架 |
| v2.0 | 2025-11 | Cartesian approach改进 |
| v1.0 | 2025-10 | 基准4候选方案 |

---

**维护者**: Yichen Feng  
**最后更新**: 2025-12-30  
**下一阶段重点**: Phase 0基础设施完成 → Phase 1感知验证 → Phase 2 VLA集成测试
