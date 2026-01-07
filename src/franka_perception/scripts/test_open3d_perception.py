#!/usr/bin/env python3
"""
Quick test for Open3D-based perception
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sys

class PerceptionTester:
    def __init__(self):
        self.detected_objects = None
        self.object_pose = None
        self.status = None
        self.segmented_pc = None
        self.num_cubes = None
        
        rospy.Subscriber('/detected_objects', String, self.on_objects)
        rospy.Subscriber('/object_pose', PoseStamped, self.on_pose)
        rospy.Subscriber('/detection_status', String, self.on_status)
        rospy.Subscriber('/segmented_pc', PointCloud2, self.on_segmented_pc)
        rospy.Subscriber('/num_cubes', String, self.on_num_cubes)
    
    def on_objects(self, msg):
        self.detected_objects = msg.data
        print(f"\n✓ Detected Objects: {msg.data[:100]}...")
    
    def on_pose(self, msg):
        self.object_pose = msg
        print(f"✓ Object Pose: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")
    
    def on_status(self, msg):
        self.status = msg.data
        print(f"✓ Status: {msg.data}")
    
    def on_segmented_pc(self, msg):
        self.segmented_pc = msg
        print(f"✓ Segmented PC received: {len(msg.data)} bytes")
    
    def on_num_cubes(self, msg):
        self.num_cubes = msg.data
        print(f"✓ Number of cubes: {msg.data}")

def main():
    rospy.init_node('perception_tester')
    
    print("="*60)
    print("Testing Open3D-based Perception System")
    print("="*60)
    
    tester = PerceptionTester()
    
    print("\nWaiting for perception data (10 seconds)...")
    rospy.sleep(10.0)
    
    print("\n" + "="*60)
    print("Test Results:")
    print("="*60)
    
    if tester.detected_objects:
        print("✓ Detected objects topic working")
    else:
        print("✗ No detected objects received")
    
    if tester.object_pose:
        print("✓ Object pose topic working")
    else:
        print("✗ No object pose received")
    
    if tester.status:
        print("✓ Status topic working")
    else:
        print("✗ No status received")
    
    if tester.segmented_pc:
        print("✓ Segmented point cloud topic working")
    else:
        print("✗ No segmented PC received")
    
    if tester.num_cubes:
        print(f"✓ Cube counting working: {tester.num_cubes} cubes")
    else:
        print("✗ No cube count received")
    
    print("="*60)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
