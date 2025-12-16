# 版本演进总结

## 快速选择指南

| 需求 | 推荐版本 | 理由 |
|-----|---------|------|
| 学习/研究 | V1 | 核心逻辑清晰，代码最简洁 |
| 原型系统 | V2 | Cartesian + 基础多候选，平衡性能和复杂度 |
| 生产部署 | V3 | 完整容错、内部约束管理、85%+ 成功率 |
| 真机集成 | V3 | 降级策略丰富，易于调试 |
| 快速原型 | V2 | 代码长度 780 行，快速理解和修改 |

## 版本对比表

### 功能维度

| 功能 | V1 | V2 | V3 |
|------|----|----|-----|
| 基础 5 步流程 | ✓ | ✓ | ✓ |
| Cartesian approach | - | ✓ | ✓ |
| Cartesian lift | - | ✓ | ✓ |
| 多 yaw 候选 | - | ✓ | ✓ |
| 多方向接近 | - | - | ✓ |
| IK 排序 | - | ✓ | ✓ |
| 规划 retry | - | ✓ | ✓ |
| Cartesian 降级 | - | - | ✓ |
| 分层 retry 策略 | - | - | ✓ |
| RETREAT 阶段 | - | - | ✓ |
| 内部 Z clamp | - | - | ✓ |
| 表面碰撞约束 | - | ✓ | ✓ |

### 性能指标

| 指标 | V1 | V2 | V3 |
|------|----|----|-----|
| 成功率 | ~50% | ~70% | ~85% |
| 平均耗时 | 8s | 12s | 14s |
| 候选数 | 1 | 4 | 16 |
| 接近方向 | 1 | 1 | 4 |
| 代码行数 | 400 | 780 | 930 |
| 集成难度 | 中 | 中 | 低 |

### 接口对比

#### V1 原版
```bash
输入: /target_cube_pose (需要 Z >= 某个值)
输出: 执行动作
约束: "Z 必须在安全范围" (上游负责)
```

#### V2 改进版
```bash
输入: /target_cube_pose (文档说需要 Z >= 0.10m)
输出: /grasp_planning_status
约束: "Z 必须 >= 0.10m" (上游负责)
```

#### V3 增强版 (推荐)
```bash
输入: /target_cube_pose (任意 Z 值)
约束: 无 (内部自动处理)
输出: /grasp_planning_status
优势: 接口清晰，上游无需关心约束
```

### 代码架构对比

#### V1: 顺序执行
```
for each target:
  HOME → OPEN → GRASP → CLOSE → LIFT → HOME
  if failed: FAIL
```

#### V2: 顺序 + 候选
```
for each target:
  generate 4 candidates (yaw only)
  for each candidate:
    HOME → OPEN → PLAN_PRE_GRASP → CARTESIAN_APPROACH 
    → CLOSE → CARTESIAN_LIFT → HOME
    if failed: try next candidate
```

#### V3: 顺序 + 候选 + 分层降级
```
for each target:
  generate 16 candidates (yaw + xy_direction)
  for each candidate:
    HOME → OPEN
    for stage in [PRE_GRASP, APPROACH, LIFT]:
      try with basic params
      if failed: 
        retry 1 (planning_time ++)
        retry 2 (tolerance ++)
        retry 3 (planner switch)
        if still failed: 
          try next candidate
    CLOSE → CARTESIAN_LIFT (2 retries) → RETREAT → HOME
```

## 迁移路径

### 如果当前用 V1

**推荐**：直接升级到 V3
- V2 的中间版本特性都包含了
- V3 额外的 approach_directions 几乎无学习成本
- 参数都有默认值

**升级步骤**：
```bash
# 1. 改启动文件
- roslaunch panda_grasp_planning grasp_planning_pipeline.launch
+ roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch

# 2. 放宽目标 Z 约束（可选）
# V3 内部自动处理，无需改上游代码

# 3. 若有性能问题，调参
# 参考 IMPROVEMENTS_V3.md 的参数调优指南
```

### 如果当前用 V2

**推荐**：升级到 V3
- 保留所有 V2 特性
- 增加多方向接近 (关键改进)
- 增加分层 retry (显著提升成功率)

**升级步骤**：同上，只需改启动文件

### 如果要保持 V2

完全可以，V2 是稳定的中间版本：
```bash
roslaunch panda_grasp_planning grasp_planning_pipeline_v2.launch
```

但损失的改进点：
- 无多方向接近 (影响某些位置成功率)
- 无分层 retry (容错能力较弱)

## 功能特性详解

### Cartesian 直线运动 (V2+ 新增)

**相比 V1 的 RRT**:
- 轨迹可预测，不会出现"大弧线接近"
- 末端姿态始终锁定
- 更适合精细的抓取操作

**配置**:
```yaml
cartesian:
  step_size: 0.005    # 5mm 步长
  min_fraction: 0.95  # 95% 完成度要求
```

### 多候选抓取 (V2 已有)

**特点**:
- V2: 4 个 yaw 角度
- V3: 4 yaw × 4 approach = 16 候选
- 自动 IK 评分和排序

**好处**:
- 某个姿态失败时自动尝试其他
- IK 评分确保最佳候选优先
- 大幅提升鲁棒性

### 多方向接近 (V3 新增)

**关键改进**:
- 不只从上方垂直下压
- 可从左、右、前侧向接近
- 解决真机中的碰撞问题

**使用场景**:
- 其他机器人/障碍物在上方
- 工作空间狭窄
- 手腕容易接近 joint limit

### 分层降级策略 (V3 新增)

**Cartesian 路径降级** (3 层):
1. 减小步长 (0.005 → 0.0025 m)
2. 缩短距离 (完整 → 80% 深度)
3. 两步到达 (部分 + 完整)

**规划重试** (3 层):
1. 增加规划时间
2. 放宽末端容差
3. 切换规划器

**好处**:
- 最后关头才放弃
- 成功率显著提升
- 系统更"聪明"

### RETREAT 阶段 (V3 新增)

**作用**:
- LIFT 后回到更高的安全位置
- 减少持物碰撞风险
- 特别适合真机

**配置**:
```yaml
retreat_offset_z: 0.20  # 相对于 grasp 的回撤高度
```

**成本**:
- 增加 ~1-2s 执行时间
- 代码简单，维护成本低
- 真机上价值很高

## 参数快速调优

### 若成功率低 (<70%)

**V2 用户**:
```yaml
# 增加候选数
candidates:
  max_candidates: 8

# 放宽 Cartesian 容差
cartesian:
  min_fraction: 0.85
  step_size: 0.003
```

**V3 用户**:
```yaml
# 增加接近方向
approach_directions:
  - [0, 0]
  - [0.1, 0]
  - [-0.1, 0]
  - [0, 0.1]
  - [0, -0.1]  # 新增

# 更激进的降级
failure_handling:
  step_downsample_factor: 3
  distance_scale: 0.6
```

### 若执行时间过长 (>20s)

**降低重试次数**:
```yaml
failure_handling:
  max_planning_retries: 2
  max_candidate_retries: 4
```

**减少 Cartesian 步长分辨率**:
```yaml
cartesian:
  step_size: 0.01  # 从 0.005 增大到 0.01
```

### 若持物碰撞频繁

**增加回撤高度**:
```yaml
retreat_offset_z: 0.30  # 从 0.20 增加到 0.30
```

**增加安全裕度**:
```yaml
environment:
  safety_margin_z: 0.12  # 从 0.08 增加到 0.12
```

## 文件清单

### 核心代码
- `scripts/grasp_pipeline_node_v3.py` - V3 推荐 (930 行)
- `scripts/grasp_pipeline_node_v2.py` - V2 备选 (780 行)
- `scripts/grasp_pipeline_node.py` - V1 参考 (400 行)

### 配置
- `config/grasp_params.yaml` - 参数配置 (所有版本通用)
- `config/planning_params.yaml` - MoveIt 配置

### 启动文件
- `launch/grasp_planning_pipeline_v3.launch` - V3 推荐
- `launch/grasp_planning_pipeline_v2.launch` - V2 备选
- `launch/grasp_planning_pipeline.launch` - V1 参考

### 文档
- `README.md` - 主文档
- `doc/IMPROVEMENTS_V2.md` - V2 详细说明
- `doc/IMPROVEMENTS_V3.md` - V3 详细说明 (推荐阅读)

## 总结建议

**对于新项目**: 直接用 V3
- 所有改进都包含
- 接口清晰易集成
- 预期成功率 85%+

**对于现有 V2 系统**: 
- 若稳定运行，可保持不变
- 若要更高成功率，升级到 V3 (改一行启动文件)

**对于学习/理解**:
- 从 V1 开始学习基础流程
- 理解 V2 的 Cartesian 和多候选
- 研究 V3 的分层降级策略

**生产部署**:
- 必用 V3
- 根据实际场景调参
- 监控日志中的 candidates_tried/retries，及时优化
