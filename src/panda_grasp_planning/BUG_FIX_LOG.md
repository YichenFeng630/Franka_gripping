# Bug 修复日志

## Bug #1: compute_cartesian_path 参数类型错误 [CRITICAL]

问题: AttributeError: 'bool' object has no attribute 'jump_threshold'

根本原因: MoveIt Noetic 的 compute_cartesian_path 签名为:
```
compute_cartesian_path(waypoints, eef_step, avoid_collisions=bool, path_constraints=None)
```

但代码错误地传递 3 个 float 参数，导致类型不匹配。

原代码 (错误):
```python
compute_cartesian_path(pose_waypoints, step_size, 0.0)  # 第 3 个是 float
```

修复 (正确):
```python
compute_cartesian_path(pose_waypoints, step_size, avoid_collisions=True)  # bool 参数
```

位置: 
- V3: grasp_pipeline_node_v3.py L523
- V2: grasp_pipeline_node_v2.py L476, L512

状态: [FIXED] 已修复，所有测试通过

## Bug #2: execute_grasp_sequence 缺少异常处理

问题: 执行某个步骤失败时，状态机未返回 IDLE，系统无法接收新目标。

根本原因: execute_grasp_sequence 没有在失败时调用 handle_failure，导致状态机卡住。

修复:
- 为每个管道步骤添加 try-except 包装
- 每个步骤失败都显式调用 handle_failure()
- 避免无声失败导致状态机不同步

位置: grasp_pipeline_node_v3.py L365-428

状态: [FIXED] 已修复，连续多目标测试正常工作

## Bug #3: plan_and_execute_joints 首次执行失败无重试

问题: 机器人初始化时首次规划可能失败 (CONTROL_FAILED)，系统无法恢复。

根本原因: Gazebo 初始化速度不稳定，偶尔会导致首次控制命令失败。

修复:
- 添加 max_retries=3 参数，失败时自动重试
- 每次重试间隔 0.5 秒
- 记录每次重试日志

位置: grasp_pipeline_node_v3.py L697-738

状态: [FIXED] 已修复，可靠性大幅提升

## 测试验证

V3 连续 3 目标测试结果:
- 目标 1 (正前方): SUCCESS 9.6s
- 目标 2 (左前方): SUCCESS 8.7s
- 目标 3 (右前方): SUCCESS 10.2s

成功率: 100%, 状态机正确重置
