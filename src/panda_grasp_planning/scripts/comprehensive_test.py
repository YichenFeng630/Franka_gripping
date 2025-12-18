#!/usr/bin/env python3
"""
综合性能测试脚本 - V2/V3对比评估
Comprehensive Performance Test - V2/V3 Comparison

功能:
1. 支持多版本测试 (V1, V2, V3)
2. 可配置的测试目标和试验数
3. 完整的性能指标收集 (成功率, 耗时, 重试次数等)
4. 生成详细的性能报告和对比分析
5. 支持CSV导出和JSON可视化数据

使用:
    python3 comprehensive_test.py --version v3 --num-trials 5
    python3 comprehensive_test.py --version v2 --num-trials 10 --output-dir results
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import sys
import csv
import json
import argparse
from datetime import datetime
import os


class PerformanceMetrics:
    """性能指标收集器"""
    def __init__(self):
        self.trials = []
        self.start_time = None
        self.end_time = None
    
    def add_trial(self, trial_data):
        """添加一个试验的数据"""
        self.trials.append(trial_data)
    
    def calculate_statistics(self):
        """计算统计指标"""
        if not self.trials:
            return None
        
        total = len(self.trials)
        successful = sum(1 for t in self.trials if t['success'])
        failed = total - successful
        success_rate = (successful / total * 100) if total > 0 else 0
        
        # 时间统计
        times = [t['elapsed_time'] for t in self.trials if t['elapsed_time'] > 0]
        min_time = min(times) if times else 0
        max_time = max(times) if times else 0
        avg_time = sum(times) / len(times) if times else 0
        median_time = sorted(times)[len(times)//2] if times else 0
        
        return {
            'total_trials': total,
            'successful_trials': successful,
            'failed_trials': failed,
            'success_rate': success_rate,
            'min_time': min_time,
            'max_time': max_time,
            'avg_time': avg_time,
            'median_time': median_time,
            'total_duration': self.end_time - self.start_time if self.end_time and self.start_time else 0
        }


class ComprehensiveGraspTester:
    """综合抓取测试器"""
    
    def __init__(self, version='v3', num_trials=5, output_dir='test_results'):
        """初始化测试器
        
        Args:
            version: 管道版本 ('v1', 'v2', 'v3')
            num_trials: 试验次数
            output_dir: 输出目录
        """
        rospy.init_node(f'comprehensive_test_{version}', anonymous=True)
        
        self.version = version.upper()
        self.num_trials = num_trials
        self.output_dir = os.path.expanduser(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(self.output_dir, f"test_{self.version}_{self.timestamp}.csv")
        self.report_file = os.path.join(self.output_dir, f"report_{self.version}_{self.timestamp}.txt")
        self.json_file = os.path.join(self.output_dir, f"data_{self.version}_{self.timestamp}.json")
        
        # 发布者/订阅者
        self.pub = rospy.Publisher('/target_cube_pose', PoseStamped, queue_size=1)
        self.current_status = None
        self.all_statuses = []
        
        rospy.Subscriber('/grasp_planning_status', String, self._status_callback)
        
        # 性能指标
        self.metrics = PerformanceMetrics()
        
        # 测试目标
        self.test_targets = self._generate_test_targets()
        
        rospy.sleep(2.0)
    
    def _status_callback(self, msg):
        """状态回调"""
        self.current_status = msg.data
        self.all_statuses.append(msg.data)
    
    def _generate_test_targets(self):
        """生成测试目标"""
        base_targets = [
            (0.5, 0.0, 0.10, "正前方"),
            (0.45, 0.1, 0.12, "左前方"),
            (0.55, -0.05, 0.11, "右前方"),
            (0.50, 0.15, 0.09, "左偏"),
            (0.50, -0.15, 0.13, "右偏"),
        ]
        
        # If num_trials <= 5, return only the requested number
        if self.num_trials <= len(base_targets):
            return base_targets[:self.num_trials]
        
        # Otherwise, repeat the base targets and add random variations
        targets = []
        for i in range(self.num_trials):
            base_idx = i % len(base_targets)
            x, y, z, desc = base_targets[base_idx]
            targets.append((x, y, z, f"{desc} #{i+1}"))
        
        return targets
    
    def run_single_trial(self, trial_idx, target):
        """运行单个试验"""
        x, y, z, desc = target
        
        print(f"\n{'='*70}")
        print(f"试验 {trial_idx+1}/{self.num_trials}: {desc}")
        print(f"{'='*70}")
        print(f"发送目标: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # 重置
        self.current_status = None
        self.all_statuses = []
        
        # 发送目标
        pose = PoseStamped()
        pose.header.frame_id = 'panda_link0'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        start_time = time.time()
        self.pub.publish(pose)
        
        # 等待完成
        final_status = None
        last_status = None
        
        while time.time() - start_time < 120:
            if self.current_status and self.current_status != last_status:
                print(f"  状态: {self.current_status}")
                last_status = self.current_status
                
                # 检查完成条件
                if self.current_status == 'IDLE' and len(self.all_statuses) > 5:
                    if 'SUCCESS' in self.all_statuses or 'FAILED' in self.all_statuses:
                        final_status = 'SUCCESS' if 'SUCCESS' in self.all_statuses else 'FAILED'
                        break
                
                if 'SUCCESS' in self.current_status or 'FAILED' in self.current_status:
                    final_status = self.current_status
                    break
            
            rospy.sleep(0.3)
        
        elapsed = time.time() - start_time
        success = 'SUCCESS' in final_status if final_status else False
        
        # 记录结果
        trial_data = {
            'trial_id': trial_idx + 1,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'target_x': x,
            'target_y': y,
            'target_z': z,
            'target_desc': desc,
            'success': success,
            'final_status': final_status if final_status else 'TIMEOUT',
            'elapsed_time': elapsed,
            'status_count': len(self.all_statuses),
            'status_sequence': ','.join(self.all_statuses)
        }
        
        self.metrics.add_trial(trial_data)
        
        # 打印结果
        status_str = "✓ 成功" if success else "✗ 失败"
        print(f"\n{status_str} - {final_status if final_status else 'TIMEOUT'} ({elapsed:.1f}s)")
        
        return success
    
    def run_all_trials(self):
        """运行所有试验"""
        print("\n" + "="*70)
        print(f"综合抓取测试 - 版本 {self.version}")
        print(f"试验次数: {self.num_trials}")
        print(f"时间戳: {self.timestamp}")
        print("="*70)
        
        self.metrics.start_time = time.time()
        
        for idx, target in enumerate(self.test_targets):
            self.run_single_trial(idx, target)
            
            # 下一个试验前等待
            if idx < len(self.test_targets) - 1:
                print(f"\n等待3秒后发送下一个目标...")
                rospy.sleep(3.0)
        
        self.metrics.end_time = time.time()
        
        # 生成报告
        self.generate_report()
    
    def generate_report(self):
        """生成性能报告"""
        stats = self.metrics.calculate_statistics()
        
        if not stats:
            rospy.logwarn("没有试验数据可分析")
            return
        
        # 写入CSV
        self._write_csv()
        
        # 写入文本报告
        self._write_text_report(stats)
        
        # 写入JSON数据
        self._write_json_data(stats)
        
        # 打印摘要
        self._print_summary(stats)
    
    def _write_csv(self):
        """写入CSV文件"""
        if not self.metrics.trials:
            return
        
        fieldnames = list(self.metrics.trials[0].keys())
        
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.metrics.trials)
        
        rospy.loginfo(f"CSV已保存: {self.csv_file}")
    
    def _write_text_report(self, stats):
        """写入文本报告"""
        lines = []
        lines.append("="*70)
        lines.append(f"GRASP PIPELINE {self.version} - 性能评估报告")
        lines.append("="*70)
        lines.append(f"时间戳: {self.timestamp}")
        lines.append("")
        
        # 概览
        lines.append("【总体概览】")
        lines.append("-" * 70)
        lines.append(f"  总试验数:           {stats['total_trials']}")
        lines.append(f"  成功试验:           {stats['successful_trials']}")
        lines.append(f"  失败试验:           {stats['failed_trials']}")
        lines.append(f"  成功率:             {stats['success_rate']:.1f}%")
        lines.append(f"  总耗时:             {stats['total_duration']:.1f}s")
        lines.append("")
        
        # 时间统计
        lines.append("【时间统计】(单位: 秒)")
        lines.append("-" * 70)
        lines.append(f"  最小耗时:           {stats['min_time']:.2f}s")
        lines.append(f"  最大耗时:           {stats['max_time']:.2f}s")
        lines.append(f"  平均耗时:           {stats['avg_time']:.2f}s")
        lines.append(f"  中位数耗时:         {stats['median_time']:.2f}s")
        lines.append("")
        
        # 详细试验结果
        lines.append("【详细试验结果】")
        lines.append("-" * 70)
        for trial in self.metrics.trials:
            status_icon = "✓" if trial['success'] else "✗"
            lines.append(f"  {status_icon} 试验{trial['trial_id']:2d}: "
                        f"{trial['target_desc']:6s} - {trial['final_status']:10s} "
                        f"({trial['elapsed_time']:6.2f}s)")
        lines.append("")
        
        # 建议
        lines.append("【性能分析】")
        lines.append("-" * 70)
        if stats['success_rate'] >= 90:
            lines.append("  ✓ 性能优秀: 成功率 >= 90%")
        elif stats['success_rate'] >= 70:
            lines.append("  ⚠ 性能良好: 成功率 >= 70%")
        else:
            lines.append("  ✗ 性能需改进: 成功率 < 70%")
        
        if stats['avg_time'] < 10:
            lines.append("  ✓ 执行速度快: 平均耗时 < 10s")
        elif stats['avg_time'] < 15:
            lines.append("  ⚠ 执行速度一般: 平均耗时 < 15s")
        else:
            lines.append("  ✗ 执行速度慢: 平均耗时 >= 15s")
        
        lines.append("="*70)
        
        report_text = "\n".join(lines)
        
        with open(self.report_file, 'w') as f:
            f.write(report_text)
        
        rospy.loginfo(f"报告已保存: {self.report_file}")
    
    def _write_json_data(self, stats):
        """写入JSON数据"""
        data = {
            'version': self.version,
            'timestamp': self.timestamp,
            'statistics': stats,
            'trials': self.metrics.trials
        }
        
        with open(self.json_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        rospy.loginfo(f"JSON数据已保存: {self.json_file}")
    
    def _print_summary(self, stats):
        """打印摘要"""
        print("\n" + "="*70)
        print("【测试摘要】")
        print("="*70)
        print(f"版本:                 {self.version}")
        print(f"总试验数:             {stats['total_trials']}")
        print(f"成功率:               {stats['success_rate']:.1f}%")
        print(f"平均耗时:             {stats['avg_time']:.2f}s")
        print(f"总耗时:               {stats['total_duration']:.1f}s")
        print("="*70)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='综合抓取性能测试')
    parser.add_argument('--version', choices=['v1', 'v2', 'v3'], default='v3',
                       help='管道版本 (default: v3)')
    parser.add_argument('--num-trials', type=int, default=5,
                       help='试验次数 (default: 5)')
    parser.add_argument('--output-dir', default='test_results',
                       help='输出目录 (default: test_results)')
    
    args = parser.parse_args()
    
    try:
        tester = ComprehensiveGraspTester(
            version=args.version,
            num_trials=args.num_trials,
            output_dir=args.output_dir
        )
        tester.run_all_trials()
        
        print("\n✓ 测试完成!")
        return 0
    
    except rospy.ROSInterruptException:
        return 1
    except Exception as e:
        rospy.logerr(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
