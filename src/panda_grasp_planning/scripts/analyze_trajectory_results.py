#!/usr/bin/env python3
"""
轨迹规划测试数据分析和可视化工具
Trajectory Planning Test Data Analysis & Visualization Tool

功能：
1. 读取轨迹规划测试的CSV数据
2. 生成详细的统计分析
3. 创建可视化报告（文本表格和JSON）
4. 对比不同版本的性能
5. 生成建议和问题诊断

Features:
1. Read trajectory planning test CSV data
2. Generate detailed statistical analysis
3. Create visualization reports (text tables and JSON)
4. Compare performance across versions
5. Generate recommendations and diagnostics

Usage:
    # 分析单个文件
    python3 analyze_trajectory_results.py trajectory_test_20240101_120000.csv
    
    # 对比多个版本
    python3 analyze_trajectory_results.py trajectory_test_v2_*.csv trajectory_test_v3_*.csv
    
    # 导出为详细报告
    python3 analyze_trajectory_results.py trajectory_test_*.csv --detailed --output report.txt
"""

import csv
import sys
import json
import os
import argparse
from datetime import datetime
from collections import defaultdict, Counter
import statistics
import glob


class TrajectoryAnalyzer:
    """轨迹规划测试分析器"""
    
    def __init__(self, csv_files):
        self.csv_files = csv_files if isinstance(csv_files, list) else [csv_files]
        self.results = []
        self.load_data()
    
    def load_data(self):
        """加载CSV数据"""
        for csv_file in self.csv_files:
            if not os.path.exists(csv_file):
                print(f"✗ File not found: {csv_file}")
                continue
            
            print(f"✓ Loading: {csv_file}")
            try:
                with open(csv_file, 'r') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        # 类型转换
                        row['trial_id'] = int(row['trial_id'])
                        row['target_x'] = float(row['target_x'])
                        row['target_y'] = float(row['target_y'])
                        row['target_z'] = float(row['target_z'])
                        row['success'] = row['success'].lower() == 'true'
                        row['total_time'] = float(row['total_time'])
                        row['planning_time'] = float(row['planning_time'])
                        row['execution_time'] = float(row['execution_time'])
                        row['candidates_generated'] = int(row.get('candidates_generated', 0) or 0)
                        row['candidates_tried'] = int(row.get('candidates_tried', 0) or 0)
                        row['planning_retries'] = int(row.get('planning_retries', 0) or 0)
                        row['cartesian_retries'] = int(row.get('cartesian_retries', 0) or 0)
                        row['total_retries'] = int(row.get('total_retries', 0) or 0)
                        row['csv_file'] = csv_file
                        
                        self.results.append(row)
            
            except Exception as e:
                print(f"✗ Error loading {csv_file}: {e}")
        
        print(f"✓ Loaded {len(self.results)} records from {len(self.csv_files)} file(s)\n")
    
    def compute_statistics(self):
        """计算统计数据"""
        if not self.results:
            return {}
        
        total = len(self.results)
        successful = sum(1 for r in self.results if r['success'])
        failed = total - successful
        
        success_rate = (successful / total * 100) if total > 0 else 0
        
        # 时间统计
        total_times = [r['total_time'] for r in self.results if r['total_time'] > 0]
        planning_times = [r['planning_time'] for r in self.results if r['planning_time'] > 0]
        execution_times = [r['execution_time'] for r in self.results if r['execution_time'] > 0]
        
        stats = {
            'total_trials': total,
            'successful_trials': successful,
            'failed_trials': failed,
            'success_rate': success_rate,
            'timing': {
                'total_time': {
                    'min': min(total_times) if total_times else 0,
                    'max': max(total_times) if total_times else 0,
                    'mean': statistics.mean(total_times) if total_times else 0,
                    'median': statistics.median(total_times) if total_times else 0,
                    'stdev': statistics.stdev(total_times) if len(total_times) > 1 else 0,
                },
                'planning_time': {
                    'min': min(planning_times) if planning_times else 0,
                    'max': max(planning_times) if planning_times else 0,
                    'mean': statistics.mean(planning_times) if planning_times else 0,
                },
                'execution_time': {
                    'min': min(execution_times) if execution_times else 0,
                    'max': max(execution_times) if execution_times else 0,
                    'mean': statistics.mean(execution_times) if execution_times else 0,
                }
            },
            'retries': {
                'total_retries': sum(r['total_retries'] for r in self.results),
                'avg_retries': statistics.mean([r['total_retries'] for r in self.results]) if self.results else 0,
                'max_retries': max([r['total_retries'] for r in self.results]) if self.results else 0,
                'planning_retries': sum(r['planning_retries'] for r in self.results),
                'cartesian_retries': sum(r['cartesian_retries'] for r in self.results),
            },
            'candidates': {
                'avg_generated': statistics.mean([r['candidates_generated'] for r in self.results]) if self.results else 0,
                'avg_tried': statistics.mean([r['candidates_tried'] for r in self.results]) if self.results else 0,
                'max_tried': max([r['candidates_tried'] for r in self.results]) if self.results else 0,
            }
        }
        
        return stats
    
    def analyze_failures(self):
        """分析失败原因"""
        failures = defaultdict(int)
        for r in self.results:
            if not r['success'] and r['failure_reason']:
                failures[r['failure_reason']] += 1
        
        return dict(sorted(failures.items(), key=lambda x: x[1], reverse=True))
    
    def generate_text_report(self, detailed=False):
        """生成文本报告"""
        stats = self.compute_statistics()
        failures = self.analyze_failures()
        
        lines = []
        lines.append("=" * 80)
        lines.append("TRAJECTORY PLANNING TEST ANALYSIS REPORT")
        lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"Data Files: {len(self.csv_files)}")
        lines.append("=" * 80)
        lines.append("")
        
        # 概览部分
        lines.append("EXECUTIVE SUMMARY")
        lines.append("-" * 80)
        lines.append(f"Total Trials:              {stats['total_trials']:>10}")
        lines.append(f"✓ Successful:              {stats['successful_trials']:>10}")
        lines.append(f"✗ Failed:                  {stats['failed_trials']:>10}")
        lines.append(f"Success Rate:              {stats['success_rate']:>9.1f}%")
        lines.append("")
        
        # 时间统计
        lines.append("TIMING STATISTICS (seconds)")
        lines.append("-" * 80)
        timing = stats['timing']
        
        lines.append("Total Execution Time:")
        lines.append(f"  Mean:                    {timing['total_time']['mean']:>10.2f}s")
        lines.append(f"  Median:                  {timing['total_time']['median']:>10.2f}s")
        lines.append(f"  Min:                     {timing['total_time']['min']:>10.2f}s")
        lines.append(f"  Max:                     {timing['total_time']['max']:>10.2f}s")
        lines.append(f"  Std Dev:                 {timing['total_time']['stdev']:>10.2f}s")
        
        lines.append("")
        lines.append("Planning Time:")
        lines.append(f"  Mean:                    {timing['planning_time']['mean']:>10.2f}s")
        
        lines.append("")
        lines.append("Execution Time:")
        lines.append(f"  Mean:                    {timing['execution_time']['mean']:>10.2f}s")
        
        lines.append("")
        
        # 重试统计
        lines.append("RETRY STATISTICS")
        lines.append("-" * 80)
        retries = stats['retries']
        lines.append(f"Total Retries:             {retries['total_retries']:>10}")
        lines.append(f"Average Retries/Trial:     {retries['avg_retries']:>10.2f}")
        lines.append(f"Max Retries:               {retries['max_retries']:>10}")
        lines.append(f"Planning Retries:          {retries['planning_retries']:>10}")
        lines.append(f"Cartesian Retries:         {retries['cartesian_retries']:>10}")
        lines.append("")
        
        # 候选统计
        lines.append("CANDIDATE STATISTICS")
        lines.append("-" * 80)
        candidates = stats['candidates']
        lines.append(f"Avg Candidates Generated:  {candidates['avg_generated']:>10.1f}")
        lines.append(f"Avg Candidates Tried:      {candidates['avg_tried']:>10.1f}")
        lines.append(f"Max Candidates Tried:      {candidates['max_tried']:>10.0f}")
        lines.append("")
        
        # 失败原因
        if failures:
            lines.append("FAILURE ANALYSIS")
            lines.append("-" * 80)
            for reason, count in failures.items():
                percentage = (count / stats['failed_trials'] * 100) if stats['failed_trials'] > 0 else 0
                lines.append(f"  {reason:40s} {count:>3} ({percentage:>5.1f}%)")
            lines.append("")
        
        # 详细结果
        if detailed:
            lines.append("DETAILED TRIAL RESULTS")
            lines.append("-" * 80)
            lines.append(f"{'Trial':<6} {'Status':<8} {'X':>8} {'Y':>8} {'Z':>8} "
                        f"{'Time':>8} {'Retries':>8} {'Reason':<30}")
            lines.append("-" * 80)
            
            for r in sorted(self.results, key=lambda x: x['trial_id']):
                status = "✓ PASS" if r['success'] else "✗ FAIL"
                reason = r['failure_reason'][:28] if r['failure_reason'] else ""
                lines.append(
                    f"{r['trial_id']:<6} {status:<8} "
                    f"{r['target_x']:>8.3f} {r['target_y']:>8.3f} {r['target_z']:>8.3f} "
                    f"{r['total_time']:>8.2f} {r['total_retries']:>8} {reason:<30}"
                )
            lines.append("")
        
        # 建议
        lines.append("RECOMMENDATIONS")
        lines.append("-" * 80)
        
        recommendations = []
        
        if stats['success_rate'] < 70:
            recommendations.append("⚠ Success rate below 70%. Consider:")
            recommendations.append("  - Increasing planning_time parameter")
            recommendations.append("  - Adjusting pre_grasp_offset_z")
            recommendations.append("  - Checking collision environment setup")
        elif stats['success_rate'] < 85:
            recommendations.append("◆ Success rate 70-85%. Performance acceptable but can be improved.")
        else:
            recommendations.append("✓ Success rate above 85%. Performance is good.")
        
        if retries['avg_retries'] > 3:
            recommendations.append(f"⚠ Average {retries['avg_retries']:.1f} retries per trial (target: <2)")
            recommendations.append("  - Review IK solution quality")
            recommendations.append("  - Check collision detection parameters")
        
        if candidates['avg_tried'] > 6:
            recommendations.append(f"⚠ Average {candidates['avg_tried']:.1f} candidates tried (target: <4)")
            recommendations.append("  - Improve candidate generation heuristics")
            recommendations.append("  - Review scoring function")
        
        if not recommendations:
            recommendations.append("✓ System performance is optimal")
        
        for rec in recommendations:
            lines.append(f"  {rec}")
        
        lines.append("")
        lines.append("=" * 80)
        
        return "\n".join(lines)
    
    def generate_json_report(self):
        """生成JSON报告"""
        stats = self.compute_statistics()
        failures = self.analyze_failures()
        
        return {
            'timestamp': datetime.now().isoformat(),
            'input_files': self.csv_files,
            'statistics': stats,
            'failures': failures,
            'trials': self.results
        }
    
    def print_summary_table(self):
        """打印摘要表格"""
        stats = self.compute_statistics()
        
        print("\n" + "=" * 80)
        print("QUICK SUMMARY")
        print("=" * 80)
        print(f"{'Metric':<30} {'Value':<20} {'Status':<20}")
        print("-" * 80)
        
        success_color = "✓ GOOD" if stats['success_rate'] >= 85 else "⚠ FAIR" if stats['success_rate'] >= 70 else "✗ POOR"
        print(f"{'Success Rate':<30} {stats['success_rate']:>6.1f}%{'':<12} {success_color:<20}")
        
        time_status = "✓ GOOD" if stats['timing']['total_time']['mean'] < 10 else "⚠ FAIR" if stats['timing']['total_time']['mean'] < 20 else "✗ SLOW"
        print(f"{'Avg Total Time':<30} {stats['timing']['total_time']['mean']:>6.2f}s{'':<12} {time_status:<20}")
        
        retry_status = "✓ GOOD" if stats['retries']['avg_retries'] < 1.5 else "⚠ FAIR" if stats['retries']['avg_retries'] < 3 else "✗ HIGH"
        print(f"{'Avg Retries':<30} {stats['retries']['avg_retries']:>6.2f}{'':<13} {retry_status:<20}")
        
        candidate_status = "✓ GOOD" if stats['candidates']['avg_tried'] < 3 else "⚠ FAIR" if stats['candidates']['avg_tried'] < 5 else "✗ MANY"
        print(f"{'Avg Candidates Tried':<30} {stats['candidates']['avg_tried']:>6.2f}{'':<13} {candidate_status:<20}")
        
        print("=" * 80 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description='Trajectory Planning Test Data Analyzer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 分析单个文件
  python3 analyze_trajectory_results.py trajectory_test_20240101_120000.csv
  
  # 使用通配符分析多个文件
  python3 analyze_trajectory_results.py trajectory_test_*.csv
  
  # 生成详细报告并保存
  python3 analyze_trajectory_results.py trajectory_test_*.csv --detailed --output report.txt
  
  # 导出JSON格式
  python3 analyze_trajectory_results.py trajectory_test_*.csv --json --output report.json
        """
    )
    
    parser.add_argument('csv_files', nargs='+', 
                       help='CSV file(s) to analyze (supports wildcards)')
    parser.add_argument('--detailed', action='store_true',
                       help='Include detailed trial-by-trial results')
    parser.add_argument('--output', type=str,
                       help='Output file for report')
    parser.add_argument('--json', action='store_true',
                       help='Export as JSON format')
    
    args = parser.parse_args()
    
    # 展开通配符
    expanded_files = []
    for pattern in args.csv_files:
        matches = glob.glob(pattern)
        if matches:
            expanded_files.extend(matches)
        else:
            expanded_files.append(pattern)
    
    if not expanded_files:
        print("✗ No CSV files found")
        sys.exit(1)
    
    try:
        analyzer = TrajectoryAnalyzer(expanded_files)
        
        if args.json:
            # JSON格式
            report = analyzer.generate_json_report()
            json_str = json.dumps(report, indent=2)
            
            if args.output:
                with open(args.output, 'w') as f:
                    f.write(json_str)
                print(f"✓ JSON report saved to: {args.output}")
            else:
                print(json_str)
        else:
            # 文本格式
            report = analyzer.generate_text_report(detailed=args.detailed)
            
            if args.output:
                with open(args.output, 'w') as f:
                    f.write(report)
                print(f"✓ Text report saved to: {args.output}")
            else:
                print(report)
            
            # 打印摘要表格
            analyzer.print_summary_table()
    
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
