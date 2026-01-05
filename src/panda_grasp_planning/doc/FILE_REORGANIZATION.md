# 文件重组织说明

**日期**: 2025-12-30  
**目的**: 将所有脚本按功能模块分类，提高代码组织和可维护性

---

## 📁 新的目录结构

### 前（旧）结构
```
scripts/
├── grasp_pipeline_node.py              (V1)
├── grasp_pipeline_node_v2.py           (V2)
├── grasp_pipeline_node_v3.py           (V3)
├── grasp_candidate_generator.py
├── example_vla_integration.py
├── perception_node.py
├── vla_inference.py
├── action_executor.py
├── comprehensive_test.py
└── comparison_test.py
```

### 后（新）结构
```
scripts/                       ← 主执行脚本
├── grasp_pipeline_v1.py
├── grasp_pipeline_v2.py
├── grasp_pipeline_v3.py
└── __init__.py

modules/                       ← 功能模块（新增）
├── candidate_generation/
│   ├── grasp_candidate_generator.py
│   └── __init__.py
├── perception/
│   ├── perception_node.py
│   └── __init__.py
├── vla/
│   ├── vla_inference.py
│   ├── example_vla_integration.py
│   └── __init__.py
├── action/
│   ├── action_executor.py
│   └── __init__.py
├── README.md                  ← 模块说明文档
└── __init__.py

tests/                         ← 测试脚本（新增）
├── comprehensive_test.py
├── comparison_test.py
└── __init__.py
```

---

## ✨ 重组织的优势

### 1. **模块化清晰**
- 相关功能集中在同一目录
- 易于理解各模块职责
- 便于独立开发和测试

### 2. **依赖管理更好**
- `modules/`中的模块可独立导入
- 减少脚本间的耦合
- 支持从其他项目重用模块

### 3. **易于扩展**
- 新增模块只需在`modules/`下创建新目录
- 不会污染`scripts/`目录
- 测试用例集中在`tests/`

### 4. **更好的包结构**
- 符合Python标准包组织方式
- 每个子模块有独立的`__init__.py`
- 支持`from modules.xxx import yyy`导入

---

## 📝 文件迁移映射

| 原位置 | 新位置 | 说明 |
|-------|-------|------|
| `scripts/grasp_pipeline_node.py` | `scripts/grasp_pipeline_v1.py` | 重命名以保持简洁 |
| `scripts/grasp_pipeline_node_v2.py` | `scripts/grasp_pipeline_v2.py` | 重命名 |
| `scripts/grasp_pipeline_node_v3.py` | `scripts/grasp_pipeline_v3.py` | 重命名 |
| `scripts/grasp_candidate_generator.py` | `modules/candidate_generation/grasp_candidate_generator.py` | 挪到模块 |
| `scripts/perception_node.py` | `modules/perception/perception_node.py` | 挪到模块 |
| `scripts/vla_inference.py` | `modules/vla/vla_inference.py` | 挪到模块 |
| `scripts/example_vla_integration.py` | `modules/vla/example_vla_integration.py` | 挪到模块 |
| `scripts/action_executor.py` | `modules/action/action_executor.py` | 挪到模块 |
| `scripts/comprehensive_test.py` | `tests/comprehensive_test.py` | 挪到tests |
| `scripts/comparison_test.py` | `tests/comparison_test.py` | 挪到tests |

---

## 🔧 导入语句更新

### V3 Pipeline中的更新

**旧代码**:
```python
from grasp_candidate_generator import GraspCandidateGenerator
```

**新代码**:
```python
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.candidate_generation.grasp_candidate_generator import GraspCandidateGenerator
```

### ROS Launch中的更新

**旧配置** (grasp_planning_pipeline_v3.launch):
```xml
<node type="grasp_pipeline_node_v3.py" ... />
```

**新配置**:
```xml
<node type="grasp_pipeline_v3.py" ... />
```

---

## 🚀 使用指南

### 运行V3 Pipeline

```bash
# 启动V3（自动加载新的模块结构）
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch
```

### 启动完整系统（含感知+VLA）

```bash
# 启动所有模块
roslaunch panda_grasp_planning panda_grasp_complete.launch \
  use_perception:=true \
  use_vla:=true \
  use_zed2:=true
```

### 运行测试

```bash
# 从任意目录运行测试
cd /opt/ros_ws
python3 src/panda_grasp_planning/tests/comprehensive_test.py --version v3

# 或使用ROS运行
rosrun panda_grasp_planning comprehensive_test.py
```

### 单独导入模块

```python
# 在你的Python脚本中
import sys
sys.path.insert(0, '/opt/ros_ws/src/panda_grasp_planning')

from modules.candidate_generation.grasp_candidate_generator import GraspCandidateGenerator
from modules.perception.perception_node import PerceptionNode
from modules.vla.vla_inference import VLAInferenceEngine
from modules.action.action_executor import ActionExecutor
```

---

## 📖 模块详细文档

详见 [modules/README.md](../modules/README.md)，其中包含：
- 各模块功能说明
- 导入和使用示例
- 模块间通信接口
- 添加新模块的方法

---

## ✅ 检查清单

重组织完成后的验证项目：

- [x] 所有文件已移动到新位置
- [x] `__init__.py`已添加到所有模块目录
- [x] V3 Pipeline导入已更新
- [x] Launch文件已更新
- [x] 文档已更新（README, QUICK_START等）
- [x] 新的完整launch文件已创建
- [ ] 在实际环境中测试运行（待确认）
- [ ] 更新CI/CD配置（如有）

---

## 💡 最佳实践

### 1. 添加新模块

```bash
# 1. 创建目录
mkdir -p modules/your_module

# 2. 添加 __init__.py
touch modules/your_module/__init__.py

# 3. 添加实现文件
cp your_code.py modules/your_module/

# 4. 导入使用
# from modules.your_module.your_code import YourClass
```

### 2. 模块间导入

```python
# 不要这样做（绝对路径）
import sys
sys.path.append('/opt/ros_ws/src/panda_grasp_planning/modules')

# 要这样做（相对路径）
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
```

### 3. ROS Node启动

```xml
<!-- 脚本应该在 scripts/ 目录 -->
<node type="grasp_pipeline_v3.py" pkg="panda_grasp_planning" name="pipeline_v3" />

<!-- 脚本会自动通过sys.path加载modules -->
```

---

## 🔍 向后兼容性

### 旧脚本会继续工作吗？

如果你有外部脚本导入这些模块：

```python
# 旧方式（现在会失败）
from grasp_candidate_generator import GraspCandidateGenerator
```

需要更新为：

```python
# 新方式
import sys
sys.path.insert(0, '/opt/ros_ws/src/panda_grasp_planning')
from modules.candidate_generation.grasp_candidate_generator import GraspCandidateGenerator
```

或者在你的项目setup.py中添加路径。

---

## 📊 代码统计

### 迁移前后的模块分布

**迁移前** (scripts/ 只有):
```
scripts/
  ├── 核心pipeline: 3文件 (~90KB)
  ├── 功能模块: 5文件 (~60KB)
  └── 测试: 2文件 (~23KB)
  总计: 10 Python文件
```

**迁移后** (明确分类):
```
scripts/      3文件  (~90KB)  - Pipeline核心
modules/      5文件  (~60KB)  - 功能模块
tests/        2文件  (~23KB)  - 测试用例
总计: 10 Python文件（逻辑组织更清晰）
```

---

## 📞 问题排查

### 导入失败？

```bash
# 检查目录结构
ls -la modules/*/

# 检查 __init__.py 是否存在
find . -name "__init__.py" | grep modules
```

### Launch文件找不到脚本？

```bash
# 检查脚本是否可执行
ls -la scripts/*.py

# 确保ROS_PACKAGE_PATH包含本包
echo $ROS_PACKAGE_PATH | grep panda_grasp_planning
```

### Python找不到modules？

```python
# 调试脚本
import sys
print("Current sys.path:", sys.path)

# 确保添加了parent目录
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
```

---

**维护者**: Yichen Feng  
**完成日期**: 2025-12-30  
**下一步**: 在实际环境中全面测试所有启动场景
