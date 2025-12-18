# Panda Grasp Planning

Franka Panda 机器人自动抓取路径规划，基于 MoveIt + Cartesian 直线运动 + 多候选策略。

## 版本选择

| 版本 | 特性 | 推荐场景 |
|-----|------|---------|
| V1 | 基础 5 步流程 | 学习研究 |
| V2 | Cartesian + 4 候选 yaw + 多层重试 | 中等可靠性 |
| V3 | 16 候选（yaw+方向）+ 分层降级 + retreat | 生产部署 |

## V3 执行流程

1. HOME - 初始安全位置
2. OPEN - 打开夹爪
3. PRE_GRASP - RRT 规划到目标上方（支持 xy 方向偏移）
4. CARTESIAN_APPROACH - 直线下压到抓取点（支持 3 层降级）
5. CLOSE - 闭合夹爪
6. CARTESIAN_LIFT - 直线上抬（支持 2 层降级）
7. RETREAT - 回到安全高度（避免持物碰撞）
8. HOME - 返回初始位置

## 核心改进点

1. 目标位姿归一化 - 内部自动处理 Z 轴 clamp，避免上游误操作

2. 多方向接近策略 - yaw (4 个) x 接近方向 (4 个) = 16 个候选

3. 分层规划重试 - 失败后尝试提高 planning_time、放宽 goal tolerance、切换 planner

4. Cartesian 智能降级 - 失败时自动降低步长或缩短距离

5. RETREAT 阶段 - 持物后先到安全高度再返回 HOME

6. 内部约束管理 - 自动添加桌面碰撞约束，Z 轴自动 clamp

## 快速开始

启动 Gazebo 仿真:
```bash
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=true
```

启动 V3 管道:
```bash
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch
```

发送测试目标:
```bash
python3 scripts/comprehensive_test.py --version v3 --num-trials 5
```

对比 V2/V3 性能:
```bash
python3 scripts/comparison_test.py --num-trials 10
```

## 文件清单

- scripts/grasp_pipeline_node_v3.py - V3 版本 (809 行)
- scripts/grasp_pipeline_node_v2.py - V2 版本 (783 行)
- scripts/grasp_pipeline_node.py - V1 版本
- scripts/comprehensive_test.py - 多版本测试框架
- scripts/comparison_test.py - V2/V3 对比测试
- launch/grasp_planning_pipeline_v3.launch - V3 启动文件
- config/grasp_params.yaml - 参数配置
- doc/IMPROVEMENTS_V2.md - V2 改进说明
- doc/IMPROVEMENTS_V3.md - V3 改进说明
- doc/VERSION_COMPARISON.md - 版本对比
- BUG_FIX_LOG.md - 修复日志

## 关键参数

```yaml
grasp:
  pre_grasp_offset_z: 0.15          # 接近高度（相对偏移）
  approach_directions:              # 接近方向
    - [0.0, 0.0]   # 从上方
    - [0.1, 0.0]   # 从 +X
    - [-0.1, 0.0]  # 从 -X
    - [0.0, 0.1]   # 从 +Y

failure_handling:
  max_planning_retries: 3           # 单候选内重试次数
  max_candidate_retries: 8          # 最多尝试候选数
  step_downsample_factor: 2         # 降级步长倍数

environment:
  table_height: 0.0                 # 桌面高度
  safety_margin_z: 0.08             # 安全裕度
  cube_half_size: 0.015             # 用于自动 clamp
```

## 输出

- `test_results/phase4_grasp_results.csv` - V3 性能数据

字段: trial, timestamp, target_x/y/z, success, total_time, candidates_tried, retries, failure_reason

## 接口

**输入**: `/target_cube_pose` (PoseStamped)
- Frame: panda_link0
- Semantic: cube center (任意 z 值都可以)
- 系统内部自动 clamp z

**输出**: `/grasp_planning_status` (String)
- IDLE, GENERATING_CANDIDATES, PLANNING_PRE_GRASP, CARTESIAN_APPROACH, ...
- SUCCESS, FAILED

**执行结果**: 完整抓取动作 (HOME → GRASP → LIFT → RETREAT → HOME)

## 版本对比

| 特性 | V1 | V2 | V3 |
|-----|----|----|-----|
| Cartesian approach | - | + | + |
| 候选数 | 1 | 4 | 16 |
| 接近方向 | 1 | 1 | 4 |
| 分层 retry | - | - | + |
| Retreat 阶段 | - | - | + |
| 桌面约束 | - | + | + |
| 内部归一化 | - | - | + |
| 预期成功率 | 50% | 70% | 85%+ |

## 推荐用法

### 集成方的工作量

1. 提供 `/target_cube_pose`
   ```python
   pose = PoseStamped()
   pose.header.frame_id = 'panda_link0'
   pose.pose.position = detected_cube_center  # 任意 z
   pub.publish(pose)
   ```

2. 监听 `/grasp_planning_status`
   ```python
   def status_cb(msg):
       if msg.data == 'SUCCESS':
           print("抓取成功")
   ```

3. 如需特殊场景调整：
   - 修改 `config/grasp_params.yaml`
   - 重启 pipeline node

### 调试建议

```bash
# 查看详细日志
roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch
# 观察：candidates_tried, cartesian_attempts, retries

# 测试多个目标
for i in {1..5}; do
  z=$((12 + i * 5))  # 12-30 cm 范围
  rostopic pub /target_cube_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'panda_link0'}, 
      pose: {position: {x: 0.5, y: 0, z: 0.$z}, 
             orientation: {w: 1}}}"
  sleep 5
done

# 分析结果
cat test_results/phase4_grasp_results.csv
```
