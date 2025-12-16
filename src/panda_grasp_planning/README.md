# Panda Grasp Planning

Franka Panda 机器人自动抓取路径规划，基于 MoveIt + RRTConnect。

## 抓取流程

1. HOME - 初始位置
2. OPEN - 打开夹爪
3. GRASP - 下降到目标
4. CLOSE - 夹住物体
5. LIFT - 上抬并返回

## 快速开始

```bash
# 启动仿真
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=true

# 启动抓取节点
roslaunch panda_grasp_planning grasp_planning_pipeline.launch

# 自动测试
python3 src/panda_grasp_planning/scripts/auto_test_phase2.py
```

## 主要文件

- `scripts/grasp_pipeline_node.py` - 抓取执行
- `scripts/auto_test_phase2.py` - 自动化测试
- `config/grasp_params.yaml` - 参数配置

## 输出

- `test_results/auto_test_[timestamp].log` - 测试日志
- `test_results/phase2_grasp_results.csv` - 性能数据

## 接口

输入: `/target_cube_pose` (PoseStamped, frame: panda_link0)  
输出: 执行完整抓取动作

## 参数

```yaml
grasp:
  grasp_offset_z: 0.02
  lift_offset_z: 0.15
gripper:
  open_width: 0.08
  close_width: 0.02
  max_effort: 50.0
```

注意: 目标 Z >= 0.10m
