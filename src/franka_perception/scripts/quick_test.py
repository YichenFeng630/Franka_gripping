#!/usr/bin/env python3
"""
Quick Perception Test - Compare one detection with Gazebo ground truth

Usage:
    # Test red cube
    rosrun franka_perception quick_test.py
    
    # Test specific color
    rosrun franka_perception quick_test.py blue
"""

import sys
import json
import rospy
import numpy as np
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState, GetWorldProperties


def get_gazebo_cubes():
    """Get all cubes from Gazebo."""
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=3.0)
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=3.0)
        
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        
        world = get_world_properties()
        cubes = {}
        
        color_map = {
            'red': 'RED',
            'blue': 'BLUE', 
            'green': 'GREEN',
            'yellow': 'YELLOW'
        }
        
        for model_name in world.model_names:
            if model_name.startswith('cube_'):
                try:
                    state = get_model_state(model_name, '')
                    if state.success:
                        for color_key, color_gazebo in color_map.items():
                            if color_gazebo in model_name.upper():
                                pos = state.pose.position
                                cubes[model_name] = {
                                    'color': color_key,
                                    'position': np.array([pos.x, pos.y, pos.z]),
                                    'name': model_name
                                }
                                break
                except Exception as e:
                    print(f"Warning: Failed to get {model_name}: {e}")
                    
        return cubes
    except Exception as e:
        print(f"Error getting Gazebo cubes: {e}")
        return {}


def main():
    rospy.init_node('quick_perception_test', anonymous=True)
    
    target_color = sys.argv[1] if len(sys.argv) > 1 else 'red'
    
    print("=" * 70)
    print("Quick Perception Accuracy Test")
    print("=" * 70)
    print(f"Target color: {target_color}")
    print("")
    
    # Get Gazebo ground truth
    print("Reading Gazebo ground truth...")
    gazebo_cubes = get_gazebo_cubes()
    
    if not gazebo_cubes:
        print("❌ No cubes found in Gazebo!")
        print("Make sure Gazebo is running with cubes spawned.")
        return
    
    print(f"Found {len(gazebo_cubes)} cubes in Gazebo:")
    for name, data in gazebo_cubes.items():
        pos = data['position']
        print(f"  {name:20s} ({data['color']:6s}): "
              f"x={pos[0]:6.3f}, y={pos[1]:6.3f}, z={pos[2]:6.3f}")
    print("")
    
    # Wait for perception data
    print("Waiting for perception detection...")
    detected_objects = None
    
    def callback(msg):
        nonlocal detected_objects
        try:
            detected_objects = json.loads(msg.data)
        except:
            pass
    
    sub = rospy.Subscriber('/detected_objects', String, callback)
    
    # Wait up to 10 seconds for detection
    timeout = rospy.Time.now() + rospy.Duration(10.0)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        if detected_objects is not None:
            break
        rate.sleep()
    
    if detected_objects is None:
        print("❌ No detection received from perception node!")
        print("Make sure perception node is running:")
        print("  roslaunch franka_perception sim_perception.launch")
        return
    
    # Display detected objects
    print("Perception detected objects:")
    objects = detected_objects.get('objects', [])
    
    if not objects:
        print("  ⚠ No objects detected!")
        return
    
    for i, obj in enumerate(objects):
        pos = obj['position']
        print(f"  Object {i+1} ({obj['color']:6s}): "
              f"x={pos[0]:6.3f}, y={pos[1]:6.3f}, z={pos[2]:6.3f} "
              f"(conf: {obj['confidence']:.2f})")
    print("")
    
    # Find target color
    target_detected = None
    for obj in objects:
        if obj['color'] == target_color:
            target_detected = obj
            break
    
    if target_detected is None:
        print(f"❌ No {target_color} cube detected by perception!")
        return
    
    # Find matching Gazebo cube
    detected_pos = np.array(target_detected['position'])
    
    target_gazebo_cubes = {n: d for n, d in gazebo_cubes.items() 
                           if d['color'] == target_color}
    
    if not target_gazebo_cubes:
        print(f"❌ No {target_color} cube found in Gazebo!")
        return
    
    # Find closest match
    closest_cube = None
    min_distance = float('inf')
    
    for name, data in target_gazebo_cubes.items():
        distance = np.linalg.norm(detected_pos - data['position'])
        if distance < min_distance:
            min_distance = distance
            closest_cube = (name, data)
    
    if closest_cube is None:
        print("❌ Could not match detected object to Gazebo cube!")
        return
    
    # Calculate error
    cube_name, cube_data = closest_cube
    gazebo_pos = cube_data['position']
    error = detected_pos - gazebo_pos
    error_magnitude = np.linalg.norm(error)
    error_xy = np.linalg.norm(error[:2])
    
    print("=" * 70)
    print(f"Comparison for {target_color.upper()} cube")
    print("=" * 70)
    print(f"Matched Gazebo cube: {cube_name}")
    print("")
    print(f"Gazebo position:     x={gazebo_pos[0]:7.4f}, y={gazebo_pos[1]:7.4f}, z={gazebo_pos[2]:7.4f}")
    print(f"Detected position:   x={detected_pos[0]:7.4f}, y={detected_pos[1]:7.4f}, z={detected_pos[2]:7.4f}")
    print("")
    print("Position Error:")
    print(f"  ΔX: {error[0]*1000:+7.2f} mm")
    print(f"  ΔY: {error[1]*1000:+7.2f} mm")
    print(f"  ΔZ: {error[2]*1000:+7.2f} mm")
    print("")
    print(f"  2D Error (XY): {error_xy*1000:6.2f} mm")
    print(f"  3D Error:      {error_magnitude*1000:6.2f} mm")
    print("=" * 70)
    
    # Assessment
    print("")
    if error_xy * 1000 < 10:
        print("✓ EXCELLENT: 2D accuracy < 10mm")
    elif error_xy * 1000 < 20:
        print("✓ GOOD: 2D accuracy < 20mm")
    else:
        print("⚠ NEEDS IMPROVEMENT: 2D error > 20mm")
    
    if error_magnitude * 1000 < 20:
        print("✓ EXCELLENT: 3D accuracy < 20mm")
    elif error_magnitude * 1000 < 30:
        print("✓ GOOD: 3D accuracy < 30mm")
    else:
        print("⚠ NEEDS IMPROVEMENT: 3D error > 30mm")
    print("")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
