# franka_task_planning

Franka机器人任务规划模块 - 基于VLA的高层决策和语言指令解析

## 功能概述

该包负责机器人的高层任务规划和决策，包括：
- Vision-Language-Action (VLA) 模型推理
- 自然语言指令解析
- 物体选择和优先级排序
- 抓取候选评分和排序
- 多物体任务协调
- 颜色到bin位置的映射（sorting logic）

## 核心功能

### 1. 语言指令解析
- 输入：自然语言（如"pick the red cube"）
- 输出：目标物体属性、动作类型
- 支持颜色、大小、位置关系等描述

### 2. VLA推理
- 集成OpenVLA-7B模型
- 场景理解和语义推理
- 候选抓取评分
- 支持LoRA微调

### 3. 任务协调
- 多物体pick-and-place序列
- 颜色分类到bin
- 任务队列管理
- 错误恢复策略

## ROS接口

### 服务（Services）

#### /parse_language_instruction
解析自然语言指令
```
Request:
  string instruction            # "pick the red cube and place it in bin A"
  sensor_msgs/Image scene_image # 当前场景图像（可选）
  
Response:
  string target_color           # 目标颜色（red, blue, green, yellow）
  string action_type            # 动作类型（pick, place, sort, etc.）
  string target_location        # 目标位置（如bin名称）
  float32 confidence            # 置信度 [0-1]
  bool success
  string message
```

#### /rank_grasp_candidates
对抓取候选进行VLA评分和排序
```
Request:
  GraspCandidate[] candidates   # 来自grasp_generation
  sensor_msgs/Image scene_image
  string task_description       # 任务描述
  
Response:
  int32[] ranked_indices        # 排序后的索引
  float32[] scores              # VLA评分 [0-1]
  bool success
  string message
```

#### /get_bin_location
获取颜色对应的bin位置
```
Request:
  string color                  # red, blue, green, yellow
  
Response:
  geometry_msgs/Pose bin_pose
  bool success
```

### 话题（Topics）

#### 发布（Publish）
- `/task_status` (std_msgs/String) - 任务执行状态
- `/current_instruction` (std_msgs/String) - 当前指令
- `/vla_debug_image` (sensor_msgs/Image) - VLA处理的调试图像

#### 订阅（Subscribe）
- `/language_instruction` (std_msgs/String) - 接收语言指令
- `/scene_image` (sensor_msgs/Image) - 场景图像（来自ZED2）

### 参数（Parameters）

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `~model_checkpoint` | string | "openvla/openvla-7b-v1" | VLA模型路径 |
| `~use_lora` | bool | false | 是否使用LoRA微调 |
| `~lora_path` | string | "" | LoRA权重路径 |
| `~temperature` | float | 0.7 | 采样温度 |
| `~confidence_threshold` | float | 0.6 | 最低置信度阈值 |
| `~enable_vla` | bool | true | 启用VLA推理 |
| `~bin_positions` | dict | - | bin位置配置 |

## 使用方法

### 1. 基础模式（无VLA）
仅使用sorting logic，按颜色分类
```bash
roslaunch franka_task_planning task_coordinator.launch enable_vla:=false
```

### 2. VLA模式
使用语言模型进行决策
```bash
roslaunch franka_task_planning vla_planner.launch model_checkpoint:=openvla/openvla-7b-v1
```

### 3. 发送语言指令
```python
import rospy
from franka_task_planning.srv import ParseLanguageInstruction, ParseLanguageInstructionRequest

rospy.wait_for_service('/parse_language_instruction')
parse_instruction = rospy.ServiceProxy('/parse_language_instruction', ParseLanguageInstruction)

req = ParseLanguageInstructionRequest()
req.instruction = "pick the red cube"
req.scene_image = current_image  # 来自ZED2

resp = parse_instruction(req)
if resp.success:
    print(f"Target: {resp.target_color}, Action: {resp.action_type}")
    print(f"Confidence: {resp.confidence:.2f}")
```

### 4. VLA候选排序
```python
from franka_task_planning.srv import RankGraspCandidates, RankGraspCandidatesRequest

req = RankGraspCandidatesRequest()
req.candidates = grasp_candidates  # 来自grasp_generation
req.scene_image = current_image
req.task_description = "pick the object carefully"

resp = rank_candidates(req)
best_candidate_idx = resp.ranked_indices[0]
best_candidate = req.candidates[best_candidate_idx]
```

## Sorting Logic配置

### Bin位置定义
在`config/bin_positions.yaml`中配置：
```yaml
bins:
  red:
    position: [0.6, -0.3, 0.2]
    orientation: [0, 0, 0, 1]  # quaternion
  blue:
    position: [0.6, -0.1, 0.2]
    orientation: [0, 0, 0, 1]
  green:
    position: [0.6, 0.1, 0.2]
    orientation: [0, 0, 0, 1]
  yellow:
    position: [0.6, 0.3, 0.2]
    orientation: [0, 0, 0, 1]
```

### 颜色映射
```python
# 自动从配置加载
color_to_bin = {
    'red': 'bin_red',
    'blue': 'bin_blue',
    'green': 'bin_green',
    'yellow': 'bin_yellow'
}
```

## VLA模型说明

### 支持的模型
1. **OpenVLA-7B** (默认)
   - 7B参数的视觉-语言-动作模型
   - 支持物体识别和任务理解
   - 内存需求：~16GB GPU

2. **自定义LoRA微调**
   - 基于特定任务微调
   - 更快的推理速度
   - 更高的准确率

### 推理流程
```
输入: 语言指令 + 场景图像
  ↓
Tokenization + Image Encoding
  ↓
Transformer处理
  ↓
输出: 目标物体、动作类型、置信度
```

### 性能优化
- 使用BF16精度（降低内存）
- 批处理推理（多个候选）
- 缓存注意力机制
- Flash Attention 2

## 依赖

- ROS Noetic
- Python 3.8+
- PyTorch >= 2.0
- Transformers >= 4.30
- OpenVLA (可选)
- PIL/OpenCV

### 安装VLA依赖
```bash
pip install torch torchvision transformers
pip install openvla  # 可选：如果使用OpenVLA模型
```

## 文件结构

```
franka_task_planning/
├── nodes/
│   ├── vla_adapter_node.py           # VLA适配器节点
│   ├── task_coordinator_node.py      # 任务协调器
│   └── language_parser_node.py       # 语言解析器
├── src/franka_task_planning/
│   ├── __init__.py
│   ├── vla_inference.py              # VLA推理引擎
│   ├── language_parser.py            # NLP处理
│   ├── sorting_logic.py              # 颜色分类逻辑
│   └── task_state_machine.py        # 任务状态机
├── srv/
│   ├── ParseLanguageInstruction.srv
│   ├── RankGraspCandidates.srv
│   └── GetBinLocation.srv
├── launch/
│   ├── vla_planner.launch
│   ├── task_coordinator.launch
│   └── sorting_only.launch
├── config/
│   ├── vla_params.yaml
│   └── bin_positions.yaml
└── README.md
```

## 示例任务

### 1. 简单颜色分类
```
Instruction: "sort the cubes by color"
→ 检测所有物体
→ 依次抓取每个cube
→ 根据颜色放入对应bin
```

### 2. 选择性抓取
```
Instruction: "pick the red cube"
→ 过滤非红色物体
→ 选择红色物体
→ 生成抓取姿态
→ 执行抓取
```

### 3. 复杂指令
```
Instruction: "pick the red cube and place it next to the blue one"
→ 解析两个目标：red cube（抓取）、blue cube（参考位置）
→ 定位两个物体
→ 计算相对位置
→ 执行pick-and-place
```

## 性能指标

| 指标 | 值 |
|------|-----|
| 语言解析延迟 | < 500ms |
| VLA推理延迟 | < 2秒（GPU）|
| 候选排序时间 | < 1秒（8候选）|
| 指令识别准确率 | > 90% |

## 故障排除

### 问题1: VLA模型加载失败
- 检查模型路径是否正确
- 确认GPU内存充足（建议 > 16GB）
- 尝试使用CPU推理（添加`device:=cpu`）

### 问题2: 语言指令无响应
- 检查服务是否启动：`rosservice list | grep parse`
- 确认输入格式正确
- 查看节点日志：`rosnode log vla_adapter_node`

### 问题3: 候选排序不合理
- 调整`temperature`参数（降低可提高确定性）
- 检查场景图像质量
- 考虑微调VLA模型

## 开发者

- **维护者**: Yichen Feng
- **创建日期**: 2026-01-07
- **版本**: 1.0.0

## License

BSD 3-Clause

## 参考

- VLA推理引擎: `panda_grasp_planning/modules/vla/vla_inference.py`
- Sorting状态机: `panda_grasp_planning/modules/sorting/sorting_state_machine.py`
- OpenVLA论文: [arXiv:2406.09246](https://arxiv.org/abs/2406.09246)

## 未来扩展

- [ ] 支持更复杂的空间关系推理
- [ ] 多步骤任务规划
- [ ] 动态障碍物避让
- [ ] 人机协作场景
- [ ] 强化学习fine-tuning
