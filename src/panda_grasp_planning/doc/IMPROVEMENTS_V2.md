# V2 改进说明

## 核心改进

1. PRE_GRASP + CARTESIAN_APPROACH 分离 - 分离规划阶段，提高轨迹可预测性

2. CARTESIAN_LIFT 直线上抬 - 使用 Cartesian 路径确保上抬稳定

3. 多候选抓取姿态 - 生成 4 个 yaw 候选，按 IK 可行性排序

4. 分段规划策略 - HOME 和 LIFT 使用 RRT，approach 使用 Cartesian

5. Retry 容错逻辑 - 失败时自动增加 planning_time、切换 planner、尝试下一候选

6. 桌面碰撞约束 - 自动添加桌面为碰撞对象，Z 轴自动 clamp

## 执行流程

HOME → PRE_GRASP (RRT) → OPEN → CARTESIAN_APPROACH (直线) → CLOSE → CARTESIAN_LIFT (直线) → HOME (RRT)

## 参数配置

关键参数在 config/grasp_params.yaml:

- pre_grasp_height: 0.15 (安全高度)
- step_size: 0.005 (5mm 步长)
- min_fraction: 0.95 (Cartesian 路径完成度要求)
- max_planning_retries: 3 (规划重试次数)
- max_candidate_retries: 4 (候选尝试次数)

## 性能指标

- 成功率: 约 70%
- 平均耗时: 12 秒
- 候选数: 4 个 yaw 角
- 代码行数: 783 行

## 修复日志

修复 compute_cartesian_path 参数类型错误:
```python
# 修复前 (错误)
compute_cartesian_path(waypoints, step_size, 0.0)

# 修复后 (正确)
compute_cartesian_path(waypoints, step_size, avoid_collisions=True)
```

这是 MoveIt Noetic 的 Python-C++ 绑定要求第 3 个参数必须是 bool 类型。
