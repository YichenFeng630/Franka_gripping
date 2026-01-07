#!/usr/bin/env python3
"""
Z轴偏差补正工具
针对Z轴系统性偏差的快速修复方案
"""

import sys
import json
import rospy
import numpy as np
import subprocess
import time


class ZAxisCalibrator:
    """Z轴快速标定工具"""
    
    def __init__(self):
        self.measurements = []
        self.z_offset = 0
    
    def measure_z_error(self, num_measurements=5):
        """测量Z轴误差"""
        print(f"\n📏 开始Z轴误差测量 (测试{num_measurements}次)")
        print("="*60)
        
        z_errors = []
        
        for i in range(num_measurements):
            print(f"\n[{i+1}/{num_measurements}] 运行精度测试...")
            
            try:
                result = subprocess.run(
                    ['python3', 
                     'src/franka_perception/scripts/test_accuracy.py'],
                    cwd='/opt/ros_ws',
                    capture_output=True,
                    timeout=15,
                    text=True
                )
                
                output = result.stdout
                
                # 提取Z轴误差
                z_axis_errors = []
                lines = output.split('\n')
                for line in lines:
                    if 'XYZ误差:' in line:
                        # 提取格式: XYZ误差: [x, y, z]mm
                        try:
                            parts = line.split('[')[1].split(']')[0].split(',')
                            z_error = float(parts[2].strip().replace('mm', ''))
                            z_axis_errors.append(z_error)
                        except:
                            pass
                
                if z_axis_errors:
                    avg_z = np.mean(z_axis_errors)
                    z_errors.append(avg_z)
                    print(f"  Z轴误差: {avg_z:.1f}mm")
                else:
                    print(f"  ❌ 无法解析Z轴误差")
                
                time.sleep(1)
                
            except Exception as e:
                print(f"  ❌ 测试失败: {e}")
        
        if z_errors:
            avg_z_error = np.mean(z_errors)
            std_z_error = np.std(z_errors)
            
            print(f"\n{'='*60}")
            print(f"📊 Z轴误差统计:")
            print(f"  平均误差: {avg_z_error:.1f}mm")
            print(f"  标准差: {std_z_error:.1f}mm")
            print(f"  最大误差: {np.max(z_errors):.1f}mm")
            print(f"  最小误差: {np.min(z_errors):.1f}mm")
            print(f"{'='*60}")
            
            # 计算补正值
            # 如果Z轴误差为+25mm，说明检测点比真值高25mm
            # 需要在Z方向减小25mm来补正
            self.z_offset = -avg_z_error / 1000.0  # 转换为米
            
            print(f"\n🔧 建议的Z轴补正值: {self.z_offset*1000:.1f}mm")
            print(f"   (在perception_node中添加此偏差)")
            
            return self.z_offset
        
        return None
    
    def validate_correction(self):
        """验证补正效果"""
        print(f"\n✅ 应用Z轴补正后，重新运行精度测试...")
        # 这里可以修改节点参数并重新测试
        pass


class ZOffsetInjector:
    """在点云处理中注入Z轴偏差"""
    
    @staticmethod
    def add_z_correction(offset_mm):
        """
        在perception_node.py中添加Z轴补正
        offset_mm: 补正值（毫米）
        """
        offset_m = offset_mm / 1000.0
        
        code_snippet = f"""
    # Z-axis calibration correction (added by calibrator)
    z_correction = {offset_m}
    
    # 在ICP配准后应用
    def apply_z_correction(pose_3d):
        pose_3d[2] += z_correction
        return pose_3d
"""
        return code_snippet


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Z轴快速标定工具')
    parser.add_argument('--measure', action='store_true', help='测量Z轴误差')
    parser.add_argument('--num-tests', type=int, default=5, help='测试次数')
    parser.add_argument('--offset', type=float, help='手动设置Z轴补正值(mm)')
    
    args = parser.parse_args()
    
    rospy.init_node('z_axis_calibrator', log_level=rospy.INFO)
    
    if args.measure:
        calibrator = ZAxisCalibrator()
        z_offset = calibrator.measure_z_error(args.num_tests)
        
        if z_offset:
            print(f"\n💡 使用方案:")
            print(f"   1. 在perception_node.py中的on_cloud()函数内")
            print(f"   2. 在ICP配准后，发布位置前")
            print(f"   3. 添加: detected_position[2] += {z_offset:.4f}")
            
    elif args.offset:
        print(f"Z轴补正值: {args.offset}mm = {args.offset/1000:.4f}m")
    
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
