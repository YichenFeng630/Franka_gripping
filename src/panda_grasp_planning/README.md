# Panda Grasp Planning

Franka Panda 机械臂自主抓取和分类系统 - **从基础仿真到端到端学习策略的完整框架**

> 📚 **快速导航**: [QUICK_START.md](doc/QUICK_START.md) · [DEVELOPMENT_ROADMAP.md](doc/DEVELOPMENT_ROADMAP.md) · [FILE_REORGANIZATION.md](doc/FILE_REORGANIZATION.md)

---

## � 文档导航

| 文档 | 内容 |
|------|------|
| **[QUICK_START.md](doc/QUICK_START.md)** | 30秒快速启动 + 4-Phase总览 |
| **[DEVELOPMENT_ROADMAP.md](doc/DEVELOPMENT_ROADMAP.md)** | 完整技术方案与代码示例 |
| **[FILE_REORGANIZATION.md](doc/FILE_REORGANIZATION.md)** | 文件重组织说明 |
| **[VISION_SETUP.md](doc/VISION_SETUP.md)** | ZED2相机硬件配置 |
| **[IMPROVEMENTS_V3.md](doc/IMPROVEMENTS_V3.md)** | V2 vs V3算法对比 |
| **[modules/README.md](modules/README.md)** | 功能模块详解 |

---

## 🏗️ 项目结构

```
panda_grasp_planning/
├── scripts/                           # 主执行脚本
│   ├── grasp_pipeline_v1.py           # V1: 基准 (4候选)
│   ├── grasp_pipeline_v2.py           # V2: Cartesian改进
│   ├── grasp_pipeline_v3.py           # V3: 推荐版本 (16候选+分层重试)
│   └── __init__.py
├── modules/                           # ✨ 功能模块（新结构）
│   ├── candidate_generation/
│   │   └── grasp_candidate_generator.py
│   ├── perception/
│   │   └── perception_node.py
│   ├── vla/
│   │   ├── vla_inference.py
│   │   └── example_vla_integration.py
│   ├── action/
│   │   └── action_executor.py
│   └── README.md
├── tests/
│   ├── comprehensive_test.py
│   ├── comparison_test.py
│   └── __init__.py
├── launch/
│   ├── grasp_planning_pipeline_v3.launch
│   ├── grasp_planning_pipeline_v2.launch
│   └── panda_grasp_complete.launch
├── config/
│   ├── grasp_params.yaml
│   ├── planning_params.yaml
│   └── action_space.yaml
└── doc/
    ├── DEVELOPMENT_ROADMAP.md
    ├── QUICK_START.md
    ├── FILE_REORGANIZATION.md
    ├── VISION_SETUP.md
    └── IMPROVEMENTS_V3.md
```

---

## ⚡ 快速启动 (30秒)

```bash
# 启动仿真
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch

# 启动V3 Pipeline (新终端)
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch

# 运行测试 (新终端)
cd /opt/ros_ws/src/panda_grasp_planning
python3 tests/comprehensive_test.py --version v3 --num-trials 5
```

详见 [QUICK_START.md](doc/QUICK_START.md)

---

## 🎯 核心特性

### ✅ 三个版本可选

| 版本 | 特点 | 成功率 | 场景 |
|------|------|--------|------|
| **V1** | 4候选 + 基础RRT | 62% | 学习参考 |
| **V2** | 4候选 + Cartesian | 74% | 中等可靠 |
| **V3** ⭐ | 16候选 + 分层重试 | **81%** | 生产部署 |

### ✨ V3 改进

1. **16方向候选** (4个yaw × 4个接近方向)
2. **分层失败恢复** (RRT自适应 + Cartesian降级)
3. **RETREAT阶段** (安全离开)
4. **IK可行性评分** (自动过滤)
5. **完全模块化** (易于扩展)

---

## � 性能对比

| 指标 | V1 | V2 | V3 |
|------|----|----|-----|
| 单次成功率 | 62% | 74% | **81%** |
| 3次任务 | 51% | 68% | **75%** |
| 平均耗时 | 25s | 23s | **20s** |

详见 [IMPROVEMENTS_V3.md](doc/IMPROVEMENTS_V3.md)

---

## 🔌 VLA & 学习支持

- ✅ 独立候选生成模块 (`modules/candidate_generation/`)
- ✅ 结构化候选数据 (70+属性)
- ✅ OpenVLA推理引擎 (`modules/vla/`)
- ✅ 统一动作空间 (`config/action_space.yaml`)
- ✅ 学习策略框架

详见 [DEVELOPMENT_ROADMAP.md](doc/DEVELOPMENT_ROADMAP.md)

---

## 📦 导入模块

```python
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.candidate_generation.grasp_candidate_generator import GraspCandidateGenerator
from modules.perception.perception_node import PerceptionNode
from modules.vla.vla_inference import VLAInferenceEngine
```

详见 [modules/README.md](modules/README.md)

---

## 📚 文档清理

**已删除的过时文件**:
- `BUG_FIX_LOG.md` - 过时bug记录
- `TEST_SOLUTION_OVERVIEW.md` - 重复于QUICK_START
- `CANDIDATE_DESIGN.md` - 内容已纳入ROADMAP
- `performance_evaluation/` - 旧的性能评估

**保留的核心文档** (6个):
- `README.md` (本文件) - 项目概览
- `QUICK_START.md` - 快速开始
- `DEVELOPMENT_ROADMAP.md` - 完整技术方案
- `FILE_REORGANIZATION.md` - 文件结构说明
- `VISION_SETUP.md` - 硬件配置
- `IMPROVEMENTS_V3.md` - 算法对比

---

## 📋 核心文件映射

| 原位置 | 新位置 | 说明 |
|-------|-------|------|
| `scripts/grasp_pipeline_node_v3.py` | `scripts/grasp_pipeline_v3.py` | 重命名 |
| `scripts/grasp_candidate_generator.py` | `modules/candidate_generation/` | 挪到模块 |
| `scripts/perception_node.py` | `modules/perception/` | 挪到模块 |
| `scripts/vla_inference.py` | `modules/vla/` | 挪到模块 |
| `scripts/*_test.py` | `tests/` | 挪到测试 |

详见 [FILE_REORGANIZATION.md](doc/FILE_REORGANIZATION.md)

---

## 🚀 进阶功能

### Phase 1: 自动视觉感知
```bash
rosrun panda_grasp_planning perception_node.py
```

### Phase 2: VLA决策
```bash
rosrun panda_grasp_planning vla_inference.py
```

### Phase 3: 学习策略
```bash
python3 train_policy.py  # ACT / Diffusion Policy
```

详见 [DEVELOPMENT_ROADMAP.md](doc/DEVELOPMENT_ROADMAP.md)

---

## 🎓 使用建议

1. **新手**: [QUICK_START.md](doc/QUICK_START.md) → 运行V3 → 查看结果
2. **进阶**: [DEVELOPMENT_ROADMAP.md](doc/DEVELOPMENT_ROADMAP.md) → 了解4-Phase → 逐步集成
3. **开发**: [modules/README.md](modules/README.md) → 修改模块 → 提交改进
4. **研究**: [IMPROVEMENTS_V3.md](doc/IMPROVEMENTS_V3.md) → 性能分析 → 算法改进

---

## 🔧 常见问题

**Q: 如何只用V3不用VLA?**
```bash
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch
```

**Q: 在真实机器人上运行?**
```bash
roslaunch franka_control franka_control.launch robot_ip:=<ip>
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch sim:=false
```

**Q: 感知精度不够?**

调整 `config/action_space.yaml`:
```yaml
voxel_size: 0.003        # 更小→更细致
dbscan_eps: 0.015        # 更小→更紧密
ransac_dist: 0.005       # 更小→更严格
```

更多Q&A见 [QUICK_START.md](doc/QUICK_START.md)
