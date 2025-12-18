#!/usr/bin/env python3
"""
V2/V3对比测试脚本
V2 vs V3 Comparison Test

功能:
1. 依次测试V2和V3版本
2. 生成对比分析报告
3. 性能指标可视化数据

使用:
    python3 comparison_test.py --num-trials 10
"""

import subprocess
import sys
import json
import os
from datetime import datetime


class ComparisonAnalyzer:
    """对比分析器"""
    
    def __init__(self, results_dir='test_results'):
        self.results_dir = results_dir
        self.v2_data = None
        self.v3_data = None
    
    def load_results(self, version, timestamp):
        """加载测试结果"""
        json_file = os.path.join(self.results_dir, f"data_{version}_{timestamp}.json")
        
        if not os.path.exists(json_file):
            print(f"✗ 找不到结果文件: {json_file}")
            return None
        
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            return data
        except Exception as e:
            print(f"✗ 加载结果失败: {e}")
            return None
    
    def generate_comparison_report(self, v2_data, v3_data):
        """生成对比报告"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = os.path.join(self.results_dir, f"comparison_{timestamp}.txt")
        
        lines = []
        lines.append("="*80)
        lines.append("PANDA GRASP PIPELINE - V2 vs V3 性能对比分析")
        lines.append("="*80)
        lines.append(f"测试时间: {timestamp}")
        lines.append("")
        
        # 提取统计数据
        v2_stats = v2_data['statistics']
        v3_stats = v3_data['statistics']
        
        # 对比表格
        lines.append("【性能指标对比】")
        lines.append("-" * 80)
        lines.append(f"{'指标':<20} {'V2':<20} {'V3':<20} {'差异':<15}")
        lines.append("-" * 80)
        
        # 成功率
        v2_sr = v2_stats['success_rate']
        v3_sr = v3_stats['success_rate']
        diff_sr = v3_sr - v2_sr
        lines.append(f"{'成功率(%)':<20} {v2_sr:>18.1f} {v3_sr:>18.1f} {diff_sr:>+13.1f}")
        
        # 平均耗时
        v2_avg = v2_stats['avg_time']
        v3_avg = v3_stats['avg_time']
        diff_avg = v3_avg - v2_avg
        lines.append(f"{'平均耗时(s)':<20} {v2_avg:>18.2f} {v3_avg:>18.2f} {diff_avg:>+13.2f}")
        
        # 中位数耗时
        v2_med = v2_stats['median_time']
        v3_med = v3_stats['median_time']
        diff_med = v3_med - v2_med
        lines.append(f"{'中位数耗时(s)':<20} {v2_med:>18.2f} {v3_med:>18.2f} {diff_med:>+13.2f}")
        
        # 最小耗时
        v2_min = v2_stats['min_time']
        v3_min = v3_stats['min_time']
        diff_min = v3_min - v2_min
        lines.append(f"{'最小耗时(s)':<20} {v2_min:>18.2f} {v3_min:>18.2f} {diff_min:>+13.2f}")
        
        # 最大耗时
        v2_max = v2_stats['max_time']
        v3_max = v3_stats['max_time']
        diff_max = v3_max - v2_max
        lines.append(f"{'最大耗时(s)':<20} {v2_max:>18.2f} {v3_max:>18.2f} {diff_max:>+13.2f}")
        
        # 试验统计
        lines.append(f"{'成功试验数':<20} {v2_stats['successful_trials']:>18} {v3_stats['successful_trials']:>18}")
        lines.append(f"{'失败试验数':<20} {v2_stats['failed_trials']:>18} {v3_stats['failed_trials']:>18}")
        lines.append("")
        
        # 详细分析
        lines.append("【详细分析】")
        lines.append("-" * 80)
        
        # 成功率分析
        if v3_sr > v2_sr:
            lines.append(f"✓ 成功率提升: V3 ({v3_sr:.1f}%) > V2 ({v2_sr:.1f}%), 提升 {diff_sr:.1f}%")
        elif v3_sr < v2_sr:
            lines.append(f"✗ 成功率下降: V3 ({v3_sr:.1f}%) < V2 ({v2_sr:.1f}%), 下降 {abs(diff_sr):.1f}%")
        else:
            lines.append(f"= 成功率持平: V3 = V2 = {v3_sr:.1f}%")
        
        # 速度分析
        if v3_avg < v2_avg:
            speedup = (v2_avg - v3_avg) / v2_avg * 100
            lines.append(f"✓ 执行速度提升: V3 ({v3_avg:.2f}s) < V2 ({v2_avg:.2f}s), 快 {speedup:.1f}%")
        elif v3_avg > v2_avg:
            slowdown = (v3_avg - v2_avg) / v2_avg * 100
            lines.append(f"✗ 执行速度下降: V3 ({v3_avg:.2f}s) > V2 ({v2_avg:.2f}s), 慢 {slowdown:.1f}%")
        else:
            lines.append(f"= 执行速度持平: V3 = V2 = {v3_avg:.2f}s")
        
        # 稳定性分析
        v2_var = v2_stats['max_time'] - v2_stats['min_time']
        v3_var = v3_stats['max_time'] - v3_stats['min_time']
        if v3_var < v2_var:
            lines.append(f"✓ 稳定性提升: V3耗时范围 {v3_var:.2f}s < V2 {v2_var:.2f}s")
        else:
            lines.append(f"✗ 稳定性下降: V3耗时范围 {v3_var:.2f}s > V2 {v2_var:.2f}s")
        
        lines.append("")
        
        # 结论
        lines.append("【综合评价】")
        lines.append("-" * 80)
        
        # 计算综合得分
        success_weight = 0.4
        speed_weight = 0.4
        stability_weight = 0.2
        
        v3_success_score = (v3_sr / 100) * success_weight * 100
        v3_speed_score = (1 - min(v3_avg / 20, 1)) * speed_weight * 100  # 假设20s为满分
        v3_stability_score = (1 - min(v3_var / 10, 1)) * stability_weight * 100  # 假设10s范围为满分
        
        v3_overall = v3_success_score + v3_speed_score + v3_stability_score
        
        lines.append(f"V3综合评分: {v3_overall:.1f}/100")
        lines.append(f"  - 成功率得分: {v3_success_score:.1f}/40")
        lines.append(f"  - 速度得分: {v3_speed_score:.1f}/40")
        lines.append(f"  - 稳定性得分: {v3_stability_score:.1f}/20")
        
        if v3_overall >= 80:
            lines.append(f"\n✓ 性能优秀: V3达到生产级别质量")
        elif v3_overall >= 60:
            lines.append(f"\n⚠ 性能良好: V3可用于实际应用, 但需要进一步优化")
        else:
            lines.append(f"\n✗ 性能待改进: V3还需要显著改进")
        
        lines.append("="*80)
        
        report_text = "\n".join(lines)
        
        with open(report_file, 'w') as f:
            f.write(report_text)
        
        print(f"\n✓ 对比报告已保存: {report_file}")
        print("\n" + report_text)


def run_tests(num_trials, output_dir):
    """运行V2和V3测试"""
    
    print("="*70)
    print("V2/V3 对比测试")
    print("="*70)
    
    timestamps = {}
    
    # 测试V2
    print("\n【第1阶段】测试V2管道...")
    print("-"*70)
    result_v2 = subprocess.run(
        [sys.executable, 'comprehensive_test.py',
         '--version', 'v2',
         '--num-trials', str(num_trials),
         '--output-dir', output_dir],
        capture_output=False
    )
    
    if result_v2.returncode != 0:
        print("✗ V2测试失败")
        return False
    
    # 等待一下，让管道切换
    print("\n等待管道切换...")
    import time
    time.sleep(5)
    
    # 测试V3
    print("\n【第2阶段】测试V3管道...")
    print("-"*70)
    result_v3 = subprocess.run(
        [sys.executable, 'comprehensive_test.py',
         '--version', 'v3',
         '--num-trials', str(num_trials),
         '--output-dir', output_dir],
        capture_output=False
    )
    
    if result_v3.returncode != 0:
        print("✗ V3测试失败")
        return False
    
    return True


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='V2/V3对比测试')
    parser.add_argument('--num-trials', type=int, default=5,
                       help='每个版本的试验次数 (default: 5)')
    parser.add_argument('--output-dir', default='test_results',
                       help='输出目录 (default: test_results)')
    parser.add_argument('--analyze-only', action='store_true',
                       help='仅分析已有的测试结果，不运行新测试')
    
    args = parser.parse_args()
    
    os.makedirs(args.output_dir, exist_ok=True)
    
    if not args.analyze_only:
        # 运行测试
        if not run_tests(args.num_trials, args.output_dir):
            return 1
    
    # 生成对比报告
    print("\n【第3阶段】生成对比报告...")
    print("-"*70)
    
    analyzer = ComparisonAnalyzer(results_dir=args.output_dir)
    
    # 查找最新的V2和V3结果
    v2_files = [f for f in os.listdir(args.output_dir) if f.startswith('data_V2_')]
    v3_files = [f for f in os.listdir(args.output_dir) if f.startswith('data_V3_')]
    
    if not v2_files or not v3_files:
        print("✗ 找不到测试结果文件")
        return 1
    
    # 获取最新的文件
    v2_file = sorted(v2_files)[-1]
    v3_file = sorted(v3_files)[-1]
    
    v2_timestamp = v2_file.replace('data_V2_', '').replace('.json', '')
    v3_timestamp = v3_file.replace('data_V3_', '').replace('.json', '')
    
    v2_data = analyzer.load_results('V2', v2_timestamp)
    v3_data = analyzer.load_results('V3', v3_timestamp)
    
    if v2_data and v3_data:
        analyzer.generate_comparison_report(v2_data, v3_data)
        print("\n✓ 对比分析完成!")
        return 0
    else:
        print("✗ 加载测试结果失败")
        return 1


if __name__ == '__main__':
    sys.exit(main())
