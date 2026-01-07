#!/usr/bin/env python3
"""
Test Perception Accuracy by comparing detected positions with Gazebo ground truth.

Usage:
    rosrun franka_perception test_perception_accuracy.py
    
    OR specify target color:
    rosrun franka_perception test_perception_accuracy.py red
"""

import sys
import json
import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState, GetWorldProperties


class PerceptionAccuracyTest:
    """Test perception accuracy by comparing with Gazebo ground truth."""
    
    def __init__(self, target_color='red'):
        rospy.init_node('perception_accuracy_test')
        
        self.target_color = target_color
        self.detected_objects = None
        self.target_pose = None
        
        # Subscribe to perception outputs
        rospy.Subscriber('/detected_objects', String, self.on_detected_objects)
        rospy.Subscriber('/object_pose', PoseStamped, self.on_target_pose)
        
        # Gazebo services
        rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=5.0)
        
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        
        rospy.loginfo(f"Perception Accuracy Test initialized. Target color: {target_color}")
        
    def on_detected_objects(self, msg):
        """Store latest detected objects."""
        try:
            self.detected_objects = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"Failed to parse detected_objects: {e}")
            
    def on_target_pose(self, msg):
        """Store latest target pose."""
        self.target_pose = msg
        
    def get_gazebo_cubes(self):
        """Get all cube models from Gazebo."""
        try:
            world = self.get_world_properties()
            cubes = {}
            
            color_map = {
                'red': 'RED',
                'blue': 'BLUE',
                'green': 'GREEN',
                'yellow': 'YELLOW'
            }
            
            for model_name in world.model_names:
                if model_name.startswith('cube_'):
                    # Get model state
                    try:
                        state = self.get_model_state(model_name, '')
                        if state.success:
                            # Extract color from name (e.g., cube_RED_1 -> red)
                            for color_key, color_gazebo in color_map.items():
                                if color_gazebo in model_name.upper():
                                    pos = state.pose.position
                                    cubes[model_name] = {
                                        'color': color_key,
                                        'position': [pos.x, pos.y, pos.z],
                                        'name': model_name
                                    }
                                    break
                    except Exception as e:
                        rospy.logwarn(f"Failed to get state for {model_name}: {e}")
                        
            return cubes
        except Exception as e:
            rospy.logerr(f"Failed to get Gazebo cubes: {e}")
            return {}
            
    def find_matching_cube(self, detected_pos, target_color, gazebo_cubes):
        """Find the closest Gazebo cube of matching color."""
        closest_cube = None
        min_distance = float('inf')
        
        for cube_name, cube_data in gazebo_cubes.items():
            if cube_data['color'] == target_color:
                gazebo_pos = np.array(cube_data['position'])
                detected_pos_arr = np.array(detected_pos)
                distance = np.linalg.norm(gazebo_pos - detected_pos_arr)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_cube = cube_data
                    
        return closest_cube, min_distance
        
    def calculate_error(self, detected_pos, gazebo_pos):
        """Calculate positional error."""
        detected = np.array(detected_pos)
        gazebo = np.array(gazebo_pos)
        
        error = detected - gazebo
        error_magnitude = np.linalg.norm(error)
        
        return {
            'error_x': error[0],
            'error_y': error[1],
            'error_z': error[2],
            'error_magnitude': error_magnitude,
            'error_xy': np.linalg.norm(error[:2])  # 2D error
        }
        
    def run_test(self, duration=10.0):
        """Run accuracy test for specified duration."""
        rospy.loginfo("=" * 70)
        rospy.loginfo("Starting Perception Accuracy Test")
        rospy.loginfo("=" * 70)
        rospy.loginfo(f"Target color: {self.target_color}")
        rospy.loginfo(f"Test duration: {duration}s")
        rospy.loginfo("Waiting for perception data...")
        rospy.loginfo("")
        
        # Wait for first detection
        timeout = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(1)  # 1 Hz
        
        detection_count = 0
        errors = []
        
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.detected_objects is not None:
                # Get Gazebo ground truth
                gazebo_cubes = self.get_gazebo_cubes()
                
                if not gazebo_cubes:
                    rospy.logwarn("No cubes found in Gazebo!")
                    rate.sleep()
                    continue
                    
                # Display all Gazebo cubes
                rospy.loginfo("-" * 70)
                rospy.loginfo(f"[{rospy.Time.now().to_sec():.1f}s] Gazebo Ground Truth:")
                for cube_name, cube_data in gazebo_cubes.items():
                    pos = cube_data['position']
                    rospy.loginfo(f"  {cube_name:20s} ({cube_data['color']:6s}): "
                                f"x={pos[0]:6.3f}, y={pos[1]:6.3f}, z={pos[2]:6.3f}")
                
                # Display detected objects
                rospy.loginfo("")
                rospy.loginfo("Perception Detected Objects:")
                objects = self.detected_objects.get('objects', [])
                
                if not objects:
                    rospy.logwarn("  No objects detected by perception!")
                else:
                    for i, obj in enumerate(objects):
                        pos = obj['position']
                        rospy.loginfo(f"  Object {i+1:2d} ({obj['color']:6s}): "
                                    f"x={pos[0]:6.3f}, y={pos[1]:6.3f}, z={pos[2]:6.3f} "
                                    f"(confidence: {obj['confidence']:.2f})")
                        
                        # Compare with Gazebo
                        if obj['color'] == self.target_color:
                            cube, distance = self.find_matching_cube(
                                pos, self.target_color, gazebo_cubes)
                            
                            if cube:
                                error = self.calculate_error(pos, cube['position'])
                                errors.append(error)
                                detection_count += 1
                                
                                rospy.loginfo("")
                                rospy.loginfo(f"  ✓ Matched with {cube['name']}")
                                rospy.loginfo(f"    Position Error:")
                                rospy.loginfo(f"      ΔX: {error['error_x']*1000:+7.2f} mm")
                                rospy.loginfo(f"      ΔY: {error['error_y']*1000:+7.2f} mm")
                                rospy.loginfo(f"      ΔZ: {error['error_z']*1000:+7.2f} mm")
                                rospy.loginfo(f"      2D Error (XY): {error['error_xy']*1000:6.2f} mm")
                                rospy.loginfo(f"      3D Error:      {error['error_magnitude']*1000:6.2f} mm")
                
            else:
                rospy.loginfo(f"Waiting for detection data... ({(timeout - rospy.Time.now()).to_sec():.0f}s remaining)")
                
            rate.sleep()
            
        # Calculate statistics
        rospy.loginfo("")
        rospy.loginfo("=" * 70)
        rospy.loginfo("Test Results Summary")
        rospy.loginfo("=" * 70)
        
        if detection_count == 0:
            rospy.logwarn("No detections recorded during test period!")
            return
            
        errors_array = np.array([[e['error_x'], e['error_y'], e['error_z']] for e in errors])
        error_magnitudes = np.array([e['error_magnitude'] for e in errors])
        error_xy = np.array([e['error_xy'] for e in errors])
        
        rospy.loginfo(f"Total detections: {detection_count}")
        rospy.loginfo("")
        rospy.loginfo("Position Error Statistics (mm):")
        rospy.loginfo(f"  X Error:  Mean = {np.mean(errors_array[:, 0])*1000:+7.2f}, "
                     f"Std = {np.std(errors_array[:, 0])*1000:6.2f}, "
                     f"Max = {np.max(np.abs(errors_array[:, 0]))*1000:6.2f}")
        rospy.loginfo(f"  Y Error:  Mean = {np.mean(errors_array[:, 1])*1000:+7.2f}, "
                     f"Std = {np.std(errors_array[:, 1])*1000:6.2f}, "
                     f"Max = {np.max(np.abs(errors_array[:, 1]))*1000:6.2f}")
        rospy.loginfo(f"  Z Error:  Mean = {np.mean(errors_array[:, 2])*1000:+7.2f}, "
                     f"Std = {np.std(errors_array[:, 2])*1000:6.2f}, "
                     f"Max = {np.max(np.abs(errors_array[:, 2]))*1000:6.2f}")
        rospy.loginfo("")
        rospy.loginfo(f"  2D (XY) Error:  Mean = {np.mean(error_xy)*1000:6.2f} mm, "
                     f"Std = {np.std(error_xy)*1000:6.2f} mm")
        rospy.loginfo(f"  3D Error:       Mean = {np.mean(error_magnitudes)*1000:6.2f} mm, "
                     f"Std = {np.std(error_magnitudes)*1000:6.2f} mm")
        rospy.loginfo("=" * 70)
        
        # Performance assessment
        mean_2d_error_mm = np.mean(error_xy) * 1000
        mean_3d_error_mm = np.mean(error_magnitudes) * 1000
        
        rospy.loginfo("")
        rospy.loginfo("Performance Assessment:")
        if mean_2d_error_mm < 10:
            rospy.loginfo("  ✓ EXCELLENT: 2D positioning accuracy < 10mm")
        elif mean_2d_error_mm < 20:
            rospy.loginfo("  ✓ GOOD: 2D positioning accuracy < 20mm")
        else:
            rospy.logwarn("  ⚠ NEEDS IMPROVEMENT: 2D error > 20mm")
            
        if mean_3d_error_mm < 20:
            rospy.loginfo("  ✓ EXCELLENT: 3D positioning accuracy < 20mm")
        elif mean_3d_error_mm < 30:
            rospy.loginfo("  ✓ GOOD: 3D positioning accuracy < 30mm")
        else:
            rospy.logwarn("  ⚠ NEEDS IMPROVEMENT: 3D error > 30mm")


def main():
    try:
        # Get target color from command line
        target_color = sys.argv[1] if len(sys.argv) > 1 else 'red'
        
        # Create test instance
        test = PerceptionAccuracyTest(target_color)
        
        # Run test
        rospy.sleep(2.0)  # Wait for connections
        test.run_test(duration=10.0)
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
