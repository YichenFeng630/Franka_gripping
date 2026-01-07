#!/usr/bin/env python3
"""
Simple Cube Detection Accuracy Test
测试点云定位精度，与Gazebo真值对比
"""

import sys
import json
import rospy
import numpy as np
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState, GetWorldProperties


def get_gazebo_cubes():
    """获取Gazebo中所有cube的位置"""
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=3.0)
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=3.0)
        
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        
        world = get_world_properties()
        cubes = {}
        
        for model_name in world.model_names:
            if model_name.startswith('cube_'):
                try:
                    state = get_model_state(model_name, '')
                    if state.success:
                        pos = state.pose.position
                        cubes[model_name] = np.array([pos.x, pos.y, pos.z])
                except Exception as e:
                    print(f"Warning: Failed to get {model_name}: {e}")
                    
        return cubes
    except Exception as e:
        print(f"Error getting Gazebo cubes: {e}")
        return {}


def compare_detection(gazebo_cubes, detected_objects):
    """对比检测结果与Gazebo真值"""
    if not detected_objects:
        print("❌ No objects detected!")
        return
    
    print(f"\n📊 检测结果对比:")
    print("=" * 80)
    print(f"Gazebo真值: {len(gazebo_cubes)} cubes")
    print(f"检测结果: {len(detected_objects)} cubes")
    print("-" * 80)
    
    # 对于每个检测到的cube，找最近的Gazebo cube
    gazebo_positions = np.array(list(gazebo_cubes.values()))
    errors = []
    
    for i, obj in enumerate(detected_objects):
        det_pos = np.array(obj['position'])
        
        # 计算与所有Gazebo cube的距离
        distances = np.linalg.norm(gazebo_positions - det_pos, axis=1)
        nearest_idx = np.argmin(distances)
        nearest_name = list(gazebo_cubes.keys())[nearest_idx]
        nearest_pos = gazebo_positions[nearest_idx]
        error = distances[nearest_idx]
        
        errors.append(error)
        
        # 计算XYZ分量误差
        xyz_error = det_pos - nearest_pos
        
        print(f"\nCube {i}:")
        print(f"  检测位置: [{det_pos[0]:.3f}, {det_pos[1]:.3f}, {det_pos[2]:.3f}]")
        print(f"  最近真值: {nearest_name}")
        print(f"  真值位置: [{nearest_pos[0]:.3f}, {nearest_pos[1]:.3f}, {nearest_pos[2]:.3f}]")
        print(f"  总误差: {error*1000:.1f}mm")
        print(f"  XYZ误差: [{xyz_error[0]*1000:.1f}, {xyz_error[1]*1000:.1f}, {xyz_error[2]*1000:.1f}]mm")
        if 'fitness' in obj:
            print(f"  ICP fitness: {obj['fitness']:.3f}")
    
    print("\n" + "=" * 80)
    print(f"📈 统计结果:")
    print(f"  平均误差: {np.mean(errors)*1000:.1f}mm")
    print(f"  最大误差: {np.max(errors)*1000:.1f}mm")
    print(f"  最小误差: {np.min(errors)*1000:.1f}mm")
    print(f"  标准差: {np.std(errors)*1000:.1f}mm")
    print("=" * 80)
    
    # 判断精度等级
    avg_error_mm = np.mean(errors) * 1000
    if avg_error_mm < 5:
        print("✅ 定位精度: 优秀 (<5mm)")
    elif avg_error_mm < 10:
        print("✅ 定位精度: 良好 (5-10mm)")
    elif avg_error_mm < 20:
        print("⚠️  定位精度: 一般 (10-20mm)")
    else:
        print("❌ 定位精度: 较差 (>20mm)")


def main():
    rospy.init_node('test_accuracy', anonymous=True)
    
    print("\n" + "=" * 80)
    print("🎯 Cube Detection Accuracy Test (Open3D + ICP)")
    print("=" * 80)
    
    # 获取Gazebo真值
    print("\n📍 读取Gazebo真值...")
    gazebo_cubes = get_gazebo_cubes()
    
    if not gazebo_cubes:
        print("❌ 无法获取Gazebo cube数据!")
        print("请确保Gazebo正在运行且已生成cubes")
        return
    
    print(f"✓ 找到 {len(gazebo_cubes)} 个cubes:")
    for name, pos in gazebo_cubes.items():
        print(f"  {name:20s}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
    
    # 等待perception检测结果
    print("\n⏳ 等待perception检测结果...")
    print("(确保perception node正在运行: roslaunch franka_perception sim_perception.launch)")
    
    detected_objects = None
    
    def callback(msg):
        nonlocal detected_objects
        try:
            data = json.loads(msg.data)
            detected_objects = data.get('objects', [])
        except Exception as e:
            rospy.logwarn(f"Failed to parse detected_objects: {e}")
    
    sub = rospy.Subscriber('/detected_objects', String, callback)
    
    # 等待检测结果
    timeout = rospy.Time.now() + rospy.Duration(10.0)
    rate = rospy.Rate(10)
    
    while detected_objects is None and rospy.Time.now() < timeout and not rospy.is_shutdown():
        rate.sleep()
    
    if detected_objects is None:
        print("\n❌ 10秒内未收到检测结果!")
        print("请检查:")
        print("  1. perception node是否正在运行")
        print("  2. /detected_objects topic是否有数据: rostopic echo /detected_objects")
        return
    
    print(f"✓ 收到检测结果")
    
    # 对比结果
    compare_detection(gazebo_cubes, detected_objects)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n中断测试")
