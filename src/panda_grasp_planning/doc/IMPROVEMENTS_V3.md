# V3 改进说明

## 核心改进

1. 目标位姿归一化 - 内部自动处理 Z 轴 clamp，上游无需关心

2. 多方向接近策略 - yaw (4 个) x 接近方向 (4 个) = 16 个候选

3. 分层规划重试 - 失败时尝试提高 planning_time、放宽 tolerance、切换 planner

4. Cartesian 智能降级 - 失败时自动降低步长或缩短距离

5. RETREAT 阶段 - 持物后先到安全高度再返回 HOME，避免碰撞

6. 内部约束管理 - 自动处理碰撞检查和安全约束

## 执行流程

HOME → OPEN → PRE_GRASP (RRT) → CARTESIAN_APPROACH (直线) → CLOSE → CARTESIAN_LIFT (直线) → RETREAT → HOME

## 关键特性

多方向接近: 从 4 个方向尝试接近 (上、左、右、前)

分层降级策略:
- 层级 1: 正常参数
- 层级 2: 增加 planning_time
- 层级 3: 放宽 goal tolerance
- 层级 4: 切换 planner

## 参数配置

关键参数在 config/grasp_params.yaml:

- pre_grasp_offset_z: 0.15 (相对接近高度)
- approach_directions: [[0,0], [0.1,0], [-0.1,0], [0,0.1]] (接近方向)
- step_size: 0.005 (5mm 步长)
- max_candidates: 8 (最多尝试候选数)

## 性能指标

- 成功率: 约 85%
- 平均耗时: 14 秒
- 候选数: 16 个 (yaw + 方向组合)
- 代码行数: 809 行

## 修复日志

修复 compute_cartesian_path 参数类型错误:
```python
# 修复前 (错误)
compute_cartesian_path(waypoints, step_size, 0.0)

# 修复后 (正确)
compute_cartesian_path(waypoints, step_size, avoid_collisions=True)
```

修复异常处理缺陷:
- 为每个管道步骤添加 try-except 包装
- 失败时显式调用 handle_failure()
- 确保状态机正确返回 IDLE

修复重试机制:
- plan_and_execute_joints 添加 max_retries=3
- 自动重试失败的执行命令
- 提高首次初始化的可靠性
