#!/usr/bin/env python3
"""
轨迹规划测试工具 - 完整的轨迹规划评估和数据统计
Trajectory Planning Test Tool - Comprehensive trajectory planning evaluation with statistics

功能：
1. 自动发送多个目标位置
2. 记录每个轨迹规划的详细数据（成功率、规划时间、路径长度、重试次数等）
3. 生成完整的测试报告（CSV + 可视化分析）
4. 支持多个版本的管道节点（V1, V2, V3）

Features:
1. Automatically send multiple target positions
2. Record detailed trajectory planning metrics (success rate, planning time, path length, retries)
3. Generate comprehensive test reports (CSV + analysis)
4. Support multiple pipeline versions (V1, V2, V3)

Usage:
    roslaunch franka_zed_gazebo moveit_gazebo_panda.launch gazebo_gui:=true
    roslaunch panda_grasp_planning grasp_planning_pipeline_v3.launch
    python3 test_trajectory_planning.py --version v3 --num-trials 10 --output-dir test_results

Output Files:
    - trajectory_test_[timestamp].csv - 原始数据
    - trajectory_analysis_[timestamp].txt - 统计分析
    - trajectory_plots_[timestamp].json - 可视化数据
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import os
from datetime import datetime
import sys
import csv
import json
import argparse
from collections import defaultdict
import math
import threading


class TrajectoryTestMetrics:
    """轨迹规划测试指标收集器"""
    
    def __init__(self):
        self.trial_data = {
            'trial_id': None,
            'target_x': None,
            'target_y': None,
            'target_z': None,
            'timestamp': None,
            'success': False,
            'failure_reason': '',
            'total_time': 0.0,
            'planning_time': 0.0,
            'execution_time': 0.0,
            'candidates_generated': 0,
            'candidates_tried': 0,
            'planning_retries': 0,
            'cartesian_retries': 0,
            'total_retries': 0,
            'final_status': '',
            'phases': {}  # 各阶段的详细信息
        }
        
    def to_dict(self):
        return self.trial_data.copy()
    
    def reset(self):
        self.__init__()


class TrajectoryTestNode:
    """轨迹规划测试节点"""
    
    def __init__(self, version='v3', num_trials=5, output_dir='test_results'):
        rospy.init_node('trajectory_test_node', anonymous=True)
        
        self.version = version
        self.num_trials = num_trials
        self.output_dir = os.path.expanduser(f"/opt/ros_ws/src/panda_grasp_planning/{output_dir}")
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 时间戳
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(self.output_dir, f"trajectory_test_{self.timestamp}.csv")
        self.analysis_file = os.path.join(self.output_dir, f"trajectory_analysis_{self.timestamp}.txt")
        self.plots_file = os.path.join(self.output_dir, f"trajectory_plots_{self.timestamp}.json")
        
        # 发布者和订阅者
        self.pose_pub = rospy.Publisher('/target_cube_pose', PoseStamped, queue_size=10)
        self.status_sub = rospy.Subscriber('/grasp_planning_status', String, self.status_callback)
        
        # 数据存储
        self.all_results = []
        self.current_trial_metrics = TrajectoryTestMetrics()
        self.last_status = 'IDLE'
        self.test_running = False
        
        # CSV写入器
        self.csv_writer = None
        self.csv_file_handle = None
        
        # 测试目标定义（多个不同的位置和高度）
        self.test_targets = self.generate_test_targets()
        
        rospy.loginfo(f"[TRAJECTORY TEST] Version: {self.version}")
        rospy.loginfo(f"[TRAJECTORY TEST] Num Trials: {self.num_trials}")
        rospy.loginfo(f"[TRAJECTORY TEST] Output Dir: {self.output_dir}")
        rospy.loginfo(f"[TRAJECTORY TEST] CSV: {self.csv_file}")
        
        # 初始化CSV
        self.init_csv()
        
        rospy.sleep(2.0)  # 等待发布者和订阅者连接
    
    def generate_test_targets(self):
        """生成测试目标位置
        
        测试场景：
        1. 不同的XY位置（正前方、左侧、右侧、对角线）
        2. 不同的Z高度（低、中、高）
        3. 边界情况
        """
        targets = []
        
        # 基础位置和高度
        xy_positions = [
            (0.5, 0.0, "正前方"),
            (0.45, 0.1, "前方左侧"),
            (0.45, -0.1, "前方右侧"),
            (0.4, 0.2, "左侧远"),
            (0.4, -0.2, "右侧远"),
            (0.55, 0.05, "右前方"),
        ]
        
        z_heights = [
            (0.10, "低 (0.10m)"),
            (0.12, "中 (0.12m)"),
            (0.15, "高 (0.15m)"),
        ]
        
        # 组合生成目标
        target_id = 0
        for x, y, xy_desc in xy_positions[:min(3, self.num_trials)]:  # 限制XY位置数
            for z, z_desc in z_heights[:min(3, self.num_trials)]:
                if target_id < self.num_trials:
                    targets.append({
                        'x': x, 'y': y, 'z': z,
                        'description': f"{xy_desc} + {z_desc}",
                        'trial_id': target_id
                    })
                    target_id += 1
                else:
                    break
        
        # 如果试验数不够，重复目标
        while len(targets) < self.num_trials:
            targets.append(targets[len(targets) % len(targets)])
            targets[-1]['trial_id'] = len(targets) - 1
        
        return targets[:self.num_trials]
    
    def init_csv(self):
        """初始化CSV文件"""
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        
        fieldnames = [
            'trial_id', 'target_x', 'target_y', 'target_z', 'description',
            'timestamp', 'success', 'failure_reason',
            'total_time', 'planning_time', 'execution_time',
            'candidates_generated', 'candidates_tried',
            'planning_retries', 'cartesian_retries', 'total_retries',
            'final_status'
        ]
        
        self.csv_writer = csv.DictWriter(self.csv_file_handle, fieldnames=fieldnames)
        self.csv_writer.writeheader()
        self.csv_file_handle.flush()
    
    def status_callback(self, msg):
        """状态回调 - 从管道节点接收状态更新"""
        status = msg.data
        self.last_status = status
        
        # 根据状态更新指标
        if status == 'SUCCESS':
            self.current_trial_metrics.trial_data['success'] = True
            self.current_trial_metrics.trial_data['final_status'] = 'SUCCESS'
        elif 'FAILED' in status or 'FAILED' == status:
            self.current_trial_metrics.trial_data['success'] = False
            self.current_trial_metrics.trial_data['final_status'] = 'FAILED'
            if ':' in status:
                self.current_trial_metrics.trial_data['failure_reason'] = status.split(':', 1)[1].strip()
    
    def create_target_pose(self, x, y, z):
        """创建目标位姿消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'panda_link0'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose
    
    def run_single_trial(self, trial_idx, target):
        """运行单个试验"""
        self.current_trial_metrics.reset()
        
        trial_id = trial_idx
        x, y, z = target['x'], target['y'], target['z']
        description = target['description']
        
        rospy.loginfo(f"\n{'='*70}")
        rospy.loginfo(f"[TRIAL {trial_id+1}/{self.num_trials}] {description}")
        rospy.loginfo(f"  Target: ({x:.3f}, {y:.3f}, {z:.3f})")
        rospy.loginfo(f"{'='*70}")
        
        # 记录试验数据
        self.current_trial_metrics.trial_data['trial_id'] = trial_id
        self.current_trial_metrics.trial_data['target_x'] = x
        self.current_trial_metrics.trial_data['target_y'] = y
        self.current_trial_metrics.trial_data['target_z'] = z
        self.current_trial_metrics.trial_data['timestamp'] = datetime.now().isoformat()
        
        # 发送目标并等待完成
        trial_start_time = time.time()
        self.last_status = 'IDLE'
        
        # 发送目标位姿
        pose = self.create_target_pose(x, y, z)
        rospy.loginfo(f"[TRAJECTORY] Publishing target pose...")
        self.pose_pub.publish(pose)
        
        # 等待规划和执行完成（最多60秒）
        timeout = 120.0  # 增加超时时间
        deadline = time.time() + timeout
        
        last_status_seen = 'IDLE'
        while time.time() < deadline:
            current_time = time.time() - trial_start_time
            
            # 检查状态变化
            if self.last_status != last_status_seen:
                rospy.loginfo(f"[TRAJECTORY] [{current_time:.1f}s] Status: {self.last_status}")
                last_status_seen = self.last_status
            
            # 检查完成条件
            if 'SUCCESS' in self.last_status or 'FAILED' in self.last_status:
                break
            
            rospy.sleep(0.5)
        
        trial_end_time = time.time()
        total_time = trial_end_time - trial_start_time
        
        self.current_trial_metrics.trial_data['total_time'] = total_time
        
        # 记录结果
        result = self.current_trial_metrics.to_dict()
        result['description'] = description
        
        self.all_results.append(result)
        
        # 写入CSV
        self.write_result_to_csv(result)
        
        # 打印结果摘要
        status_str = "✓ SUCCESS" if result['success'] else "✗ FAILED"
        rospy.loginfo(f"[TRIAL {trial_id+1}] {status_str}")
        rospy.loginfo(f"  Total Time: {total_time:.2f}s")
        rospy.loginfo(f"  Final Status: {result['final_status']}")
        if result['failure_reason']:
            rospy.loginfo(f"  Failure Reason: {result['failure_reason']}")
        
        # 试验间隔
        if trial_idx < self.num_trials - 1:
            rospy.loginfo(f"[TRAJECTORY] Waiting 3 seconds before next trial...")
            rospy.sleep(3.0)
    
    def write_result_to_csv(self, result):
        """将结果写入CSV"""
        if self.csv_writer:
            self.csv_writer.writerow(result)
            self.csv_file_handle.flush()
    
    def run_all_trials(self):
        """运行所有试验"""
        rospy.loginfo(f"\n\n{'#'*70}")
        rospy.loginfo(f"# TRAJECTORY PLANNING TEST - VERSION {self.version.upper()}")
        rospy.loginfo(f"# Total Trials: {self.num_trials}")
        rospy.loginfo(f"# Timestamp: {self.timestamp}")
        rospy.loginfo(f"{'#'*70}\n")
        
        try:
            for trial_idx, target in enumerate(self.test_targets):
                self.run_single_trial(trial_idx, target)
        
        except KeyboardInterrupt:
            rospy.loginfo("\n[TRAJECTORY] Test interrupted by user")
        
        finally:
            self.close_csv()
            self.generate_analysis_report()
    
    def close_csv(self):
        """关闭CSV文件"""
        if self.csv_file_handle:
            self.csv_file_handle.close()
    
    def generate_analysis_report(self):
        """生成分析报告"""
        if not self.all_results:
            rospy.logwarn("[TRAJECTORY] No results to analyze")
            return
        
        # 计算统计数据
        total_trials = len(self.all_results)
        successful_trials = sum(1 for r in self.all_results if r['success'])
        failed_trials = total_trials - successful_trials
        success_rate = (successful_trials / total_trials * 100) if total_trials > 0 else 0
        
        # 时间统计
        total_times = [r['total_time'] for r in self.all_results if r['total_time'] > 0]
        if total_times:
            avg_time = sum(total_times) / len(total_times)
            min_time = min(total_times)
            max_time = max(total_times)
            median_time = sorted(total_times)[len(total_times)//2]
        else:
            avg_time = min_time = max_time = median_time = 0
        
        # 重试统计
        total_retries = sum(r['total_retries'] for r in self.all_results)
        avg_retries = total_retries / total_trials if total_trials > 0 else 0
        
        # 生成报告
        report_lines = []
        report_lines.append("="*70)
        report_lines.append(f"TRAJECTORY PLANNING TEST ANALYSIS REPORT")
        report_lines.append(f"Version: {self.version.upper()}")
        report_lines.append(f"Timestamp: {self.timestamp}")
        report_lines.append("="*70)
        report_lines.append("")
        
        # 概览
        report_lines.append("OVERVIEW")
        report_lines.append("-" * 70)
        report_lines.append(f"  Total Trials:          {total_trials}")
        report_lines.append(f"  Successful Trials:     {successful_trials}")
        report_lines.append(f"  Failed Trials:         {failed_trials}")
        report_lines.append(f"  Success Rate:          {success_rate:.1f}%")
        report_lines.append("")
        
        # 时间统计
        report_lines.append("TIMING STATISTICS (seconds)")
        report_lines.append("-" * 70)
        report_lines.append(f"  Average Total Time:    {avg_time:.2f}s")
        report_lines.append(f"  Min Total Time:        {min_time:.2f}s")
        report_lines.append(f"  Max Total Time:        {max_time:.2f}s")
        report_lines.append(f"  Median Total Time:     {median_time:.2f}s")
        report_lines.append("")
        
        # 重试统计
        report_lines.append("RETRY STATISTICS")
        report_lines.append("-" * 70)
        report_lines.append(f"  Total Retries:         {total_retries}")
        report_lines.append(f"  Average Retries:       {avg_retries:.2f}")
        report_lines.append("")
        
        # 失败原因分析
        failure_reasons = defaultdict(int)
        for r in self.all_results:
            if r['failure_reason']:
                failure_reasons[r['failure_reason']] += 1
        
        if failure_reasons:
            report_lines.append("FAILURE REASONS")
            report_lines.append("-" * 70)
            for reason, count in sorted(failure_reasons.items(), key=lambda x: x[1], reverse=True):
                report_lines.append(f"  {reason}: {count}")
            report_lines.append("")
        
        # 详细结果
        report_lines.append("DETAILED RESULTS")
        report_lines.append("-" * 70)
        for result in self.all_results:
            status = "✓ PASS" if result['success'] else "✗ FAIL"
            report_lines.append(
                f"Trial {result['trial_id']:2d}: {status} | "
                f"({result['target_x']:.2f}, {result['target_y']:.2f}, {result['target_z']:.2f}) | "
                f"Time: {result['total_time']:6.2f}s | "
                f"Retries: {result['total_retries']}"
            )
        report_lines.append("")
        
        # 文件信息
        report_lines.append("OUTPUT FILES")
        report_lines.append("-" * 70)
        report_lines.append(f"  CSV Data:     {self.csv_file}")
        report_lines.append(f"  Analysis:     {self.analysis_file}")
        report_lines.append(f"  Plots Data:   {self.plots_file}")
        
        # 写入文件
        report_text = "\n".join(report_lines)
        with open(self.analysis_file, 'w') as f:
            f.write(report_text)
        
        # 打印到控制台
        rospy.loginfo("\n" + report_text)
        
        # 生成可视化数据
        self.generate_plot_data()
    
    def generate_plot_data(self):
        """生成可视化用的JSON数据"""
        plot_data = {
            'timestamp': self.timestamp,
            'version': self.version,
            'total_trials': len(self.all_results),
            'successful_trials': sum(1 for r in self.all_results if r['success']),
            'success_rate': sum(1 for r in self.all_results if r['success']) / len(self.all_results) * 100 if self.all_results else 0,
            'timing': {
                'total_times': [r['total_time'] for r in self.all_results],
                'planning_times': [r['planning_time'] for r in self.all_results],
                'execution_times': [r['execution_time'] for r in self.all_results],
            },
            'trials': self.all_results
        }
        
        with open(self.plots_file, 'w') as f:
            json.dump(plot_data, f, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description='Trajectory Planning Test Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_trajectory_planning.py --version v3 --num-trials 10
  python3 test_trajectory_planning.py --version v2 --num-trials 5 --output-dir my_results
        """
    )
    
    parser.add_argument('--version', type=str, default='v3',
                       choices=['v1', 'v2', 'v3'],
                       help='Pipeline version to test (default: v3)')
    parser.add_argument('--num-trials', type=int, default=5,
                       help='Number of test trials (default: 5)')
    parser.add_argument('--output-dir', type=str, default='test_results',
                       help='Output directory for results (default: test_results)')
    
    args = parser.parse_args()
    
    try:
        test_node = TrajectoryTestNode(
            version=args.version,
            num_trials=args.num_trials,
            output_dir=args.output_dir
        )
        
        test_node.run_all_trials()
        
        rospy.loginfo("\n[TRAJECTORY] Test completed. Results saved to:")
        rospy.loginfo(f"  - {test_node.csv_file}")
        rospy.loginfo(f"  - {test_node.analysis_file}")
        rospy.loginfo(f"  - {test_node.plots_file}")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("[TRAJECTORY] ROS interrupted")
    except Exception as e:
        rospy.logerr(f"[TRAJECTORY] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
