#!/usr/bin/env python3
"""
仿真环境参数优化工具
在不需要实物的情况下，通过扫描参数空间找到最优检测参数
"""

import sys
import json
import rospy
import numpy as np
import subprocess
import time
from itertools import product
from gazebo_msgs.srv import GetModelState, GetWorldProperties


class ParameterOptimizer:
    """自动参数优化器"""
    
    def __init__(self):
        self.results = []
        self.best_params = None
        self.best_error = float('inf')
        
        # 定义参数搜索空间
        self.param_space = {
            'voxel_size': [0.003, 0.005, 0.007, 0.01],  # 3-10mm
            'dbscan_eps': [0.015, 0.02, 0.025, 0.03],    # 15-30mm
            'dbscan_min_samples': [30, 50, 70],
            'ransac_dist': [0.008, 0.01, 0.015, 0.02],   # 8-20mm
        }
        
        # 可选：Z轴偏差补正
        self.z_offset_candidates = [-0.03, -0.02, -0.01, 0, 0.01, 0.02]  # ±30mm
    
    def run_single_test(self, params):
        """运行单次精度测试"""
        try:
            # 更新ROS参数
            for param_name, param_value in params.items():
                rospy.set_param(f'/perception_node/{param_name}', param_value)
            
            # 给节点时间重新加载参数
            time.sleep(0.5)
            
            # 运行精度测试
            result = subprocess.run(
                ['python3', 
                 'src/franka_perception/scripts/test_accuracy.py'],
                cwd='/opt/ros_ws',
                capture_output=True,
                timeout=15,
                text=True
            )
            
            # 解析结果
            output = result.stdout
            if '平均误差:' in output:
                # 提取平均误差
                lines = output.split('\n')
                for line in lines:
                    if '平均误差:' in line:
                        avg_error_mm = float(line.split(':')[1].split('mm')[0].strip())
                        return avg_error_mm
            
            return None
            
        except Exception as e:
            rospy.logwarn(f"Test failed: {e}")
            return None
    
    def optimize(self):
        """执行参数搜索"""
        param_names = list(self.param_space.keys())
        param_values_list = [self.param_space[name] for name in param_names]
        
        total_combinations = np.prod([len(v) for v in param_values_list])
        
        print(f"\n{'='*80}")
        print(f"🔍 开始参数优化")
        print(f"{'='*80}")
        print(f"参数空间: {self.param_space}")
        print(f"总组合数: {total_combinations}")
        print(f"{'='*80}\n")
        
        combination_count = 0
        for param_tuple in product(*param_values_list):
            combination_count += 1
            
            # 构建参数字典
            params = dict(zip(param_names, param_tuple))
            
            print(f"[{combination_count}/{total_combinations}] 测试参数:")
            for pname, pvalue in params.items():
                print(f"  {pname}: {pvalue}")
            
            # 运行测试
            avg_error = self.run_single_test(params)
            
            if avg_error is not None:
                print(f"  📊 平均误差: {avg_error:.1f}mm")
                
                self.results.append({
                    'params': params,
                    'error': avg_error
                })
                
                # 更新最优参数
                if avg_error < self.best_error:
                    self.best_error = avg_error
                    self.best_params = params.copy()
                    print(f"  ✅ 新的最优参数! 误差: {self.best_error:.1f}mm")
            else:
                print(f"  ❌ 测试失败")
            
            print()
    
    def print_results(self):
        """打印优化结果"""
        if not self.results:
            print("没有有效的测试结果")
            return
        
        # 按误差排序
        sorted_results = sorted(self.results, key=lambda x: x['error'])
        
        print(f"\n{'='*80}")
        print("📈 优化结果 (TOP 10)")
        print(f"{'='*80}")
        
        for i, result in enumerate(sorted_results[:10], 1):
            params = result['params']
            error = result['error']
            print(f"\n{i}. 误差: {error:.1f}mm")
            for pname, pvalue in params.items():
                print(f"   {pname}: {pvalue}")
        
        # 保存最优参数
        if self.best_params:
            print(f"\n{'='*80}")
            print(f"🎯 最优参数")
            print(f"{'='*80}")
            print(f"最小误差: {self.best_error:.1f}mm")
            print("\n更新 detection_params.yaml:")
            for pname, pvalue in self.best_params.items():
                print(f"  {pname}: {pvalue}")
            
            # 保存到文件
            output_file = '/tmp/best_params.json'
            with open(output_file, 'w') as f:
                json.dump({
                    'best_error': self.best_error,
                    'params': self.best_params
                }, f, indent=2)
            print(f"\n已保存到: {output_file}")


def main():
    rospy.init_node('parameter_optimizer', log_level=rospy.INFO)
    
    optimizer = ParameterOptimizer()
    
    try:
        optimizer.optimize()
        optimizer.print_results()
    except KeyboardInterrupt:
        print("\n\n⏸️  优化被中断")
        optimizer.print_results()


if __name__ == '__main__':
    main()
