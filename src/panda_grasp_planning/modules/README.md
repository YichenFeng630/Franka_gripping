# Modules - 功能模块组织

本目录包含Panda Grasp Planning的核心功能模块，组织如下：

## 目录结构

```
modules/
├── candidate_generation/    # 候选生成模块
│   ├── grasp_candidate_generator.py
│   └── __init__.py
├── perception/              # 视觉感知模块  
│   ├── perception_node.py
│   └── __init__.py
├── vla/                     # Vision Language Action模块
│   ├── vla_inference.py
│   ├── example_vla_integration.py
│   └── __init__.py
├── action/                  # 动作执行模块
│   ├── action_executor.py
│   └── __init__.py
└── __init__.py
```

## 模块说明

### 1. candidate_generation (候选生成)

**文件**: `grasp_candidate_generator.py`  
**核心类**: `GraspCandidateGenerator`, `GraspCandidate`

生成结构化的grasp候选，包含几何信息、可行性评分和优先级。支持：
- 16候选生成 (4个yaw角 × 4个接近方向)
- IK可行性检测
- 优先级评分 [0-100]

**使用示例**:
```python
from modules.candidate_generation.grasp_candidate_generator import GraspCandidateGenerator

generator = GraspCandidateGenerator(move_group=move_group)
candidates = generator.generate(target_pose)
```

### 2. perception (视觉感知)

**文件**: `perception_node.py`  
**核心类**: `PerceptionNode`

从ZED2 RGB-D数据检测目标物体：
- 点云预处理 (VoxelGrid下采样)
- RANSAC平面分割
- DBSCAN聚类
- HSV颜色分割
- EMA平滑跟踪

**运行**:
```bash
rosrun panda_grasp_planning perception_node.py _voxel_size:=0.005
```

**输出话题**:
- `/target_cube_pose` - 检测到的目标位置

### 3. vla (Vision Language Action)

**文件**: `vla_inference.py`, `example_vla_integration.py`  
**核心类**: `VLAInferenceEngine`, `VLAAdapterNode`

使用OpenVLA 7B模型为grasp候选评分：
- 异步推理 (不阻塞ROS)
- LoRA微调支持
- 批处理能力
- 与grasp_pipeline_v3的无缝集成

**使用示例**:
```python
from modules.vla.vla_inference import VLAInferenceEngine

vla = VLAInferenceEngine(model_checkpoint='openvla/openvla-7b-v1')
selected, scores = vla.select_candidate(image, candidates, "grasp the cube")
```

**启动VLA适配器**:
```bash
rosrun panda_grasp_planning vla_inference.py _temperature:=0.7
```

### 4. action (动作执行)

**文件**: `action_executor.py`  
**核心类**: `ActionExecutor`

执行统一动作空间中定义的动作：
- 7维动作向量 [dx, dy, dz, droll, dpitch, dyaw, gripper]
- 工作空间检查
- 速度/加速度限幅
- Cartesian + RRT混合规划
- 安全网关

**使用示例**:
```python
from modules.action.action_executor import ActionExecutor

executor = ActionExecutor('config/action_space.yaml')
success, obs = executor.execute_action(np.array([0.02, 0, 0, 0, 0, 0, 1]))
```

## Python导入指南

### 从Pipeline脚本导入模块

由于模块在subpackage中，需要修改sys.path：

```python
import sys
import os

# 添加parent目录到path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 现在可以导入modules
from modules.candidate_generation.grasp_candidate_generator import GraspCandidateGenerator
from modules.perception.perception_node import PerceptionNode
from modules.vla.vla_inference import VLAInferenceEngine
from modules.action.action_executor import ActionExecutor
```

### 从其他模块导入

```python
# vla模块中使用candidate_generation
from modules.candidate_generation.grasp_candidate_generator import GraspCandidate
```

### ROS Launch启动

在launch文件中正确指定节点脚本：

```xml
<!-- 启动V3 Pipeline -->
<node name="grasp_pipeline_v3" pkg="panda_grasp_planning" type="grasp_pipeline_v3.py" output="screen"/>

<!-- 启动感知节点 -->
<node name="perception_node" pkg="panda_grasp_planning" type="perception_node.py" output="screen">
    <param name="voxel_size" value="0.005"/>
</node>

<!-- 启动VLA适配器 -->
<node name="vla_adapter" pkg="panda_grasp_planning" type="vla_inference.py" output="screen">
    <param name="temperature" value="0.7"/>
</node>

<!-- 启动动作执行器 -->
<node name="action_executor" pkg="panda_grasp_planning" type="action_executor.py" output="screen"/>
```

## 模块间通信

### 数据流

```
1. Perception (感知)
   输出: /target_cube_pose → GraspCandidateGenerator

2. Candidate Generation (候选生成)
   输入: /target_cube_pose
   输出: List[GraspCandidate] → VLA + Pipeline

3. VLA (决策)
   输入: RGB Image + Candidates
   输出: Scores → Pipeline选择

4. Action Executor (执行)
   输入: Selected Candidate
   输出: Robot motion
```

### 消息接口

| 模块 | 订阅话题 | 发布话题 |
|------|---------|---------|
| Perception | `/zed2/zed_node/point_cloud/cloud_registered`, `/zed2/zed_node/rgb/image_rect_color` | `/target_cube_pose` |
| VLA | `/target_cube_pose`, `/zed2/zed_node/rgb/image_rect_color` | `/vla_selected_candidate`, `/vla_candidate_scores` |
| Pipeline | `/target_cube_pose` | `/grasp_result`, `/pipeline_state` |

## 添加新模块

如果要添加新功能模块：

1. 在`modules/`下创建新子目录：
   ```bash
   mkdir -p modules/your_module
   touch modules/your_module/__init__.py
   ```

2. 添加你的模块文件：
   ```bash
   cp your_module.py modules/your_module/
   ```

3. 在modules/__init__.py中导出（可选）：
   ```python
   # modules/__init__.py
   from .your_module.your_module import YourClass
   ```

4. 导入使用：
   ```python
   from modules.your_module.your_module import YourClass
   ```

## 版本控制

每个模块应独立版本化。更新时：

1. 模块内部函数签名改变 → 模块patch版本 +0.1
2. 模块输出格式改变 → 模块minor版本 +0.1  
3. 不向后兼容 → 模块major版本 +1.0

记录在各模块的`__version__`变量或文件注释中。

## 文档

详细文档见：
- [DEVELOPMENT_ROADMAP.md](../doc/DEVELOPMENT_ROADMAP.md) - 完整技术路线
- [QUICK_START.md](../doc/QUICK_START.md) - 快速开始指南
- 各模块内的docstring

---

**维护者**: Yichen Feng  
**最后更新**: 2025-12-30
