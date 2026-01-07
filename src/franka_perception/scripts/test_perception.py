#!/usr/bin/env python3
"""
Test script for franka_perception package

Usage:
    # Test in simulation
    python3 test_perception.py --sim
    
    # Test with real robot
    python3 test_perception.py --real
"""

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class PerceptionTester:
    """Simple test class for perception module"""
    
    def __init__(self):
        rospy.init_node('perception_tester', anonymous=True)
        
        self.detections = []
        self.poses = []
        self.status_msgs = []
        
        # Subscribe to perception outputs
        rospy.Subscriber('/detected_objects', String, self.on_objects)
        rospy.Subscriber('/object_pose', PoseStamped, self.on_pose)
        rospy.Subscriber('/detection_status', String, self.on_status)
        
        rospy.loginfo("Perception tester initialized")
        rospy.loginfo("Waiting for detection messages...")
    
    def on_objects(self, msg):
        """Callback for detected objects"""
        self.detections.append(msg.data)
        rospy.loginfo(f"✓ Received detection: {msg.data[:100]}...")
    
    def on_pose(self, msg):
        """Callback for object pose"""
        self.poses.append(msg)
        rospy.loginfo(f"✓ Received pose: x={msg.pose.position.x:.3f}, "
                     f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}")
    
    def on_status(self, msg):
        """Callback for detection status"""
        self.status_msgs.append(msg.data)
        rospy.loginfo(f"✓ Status: {msg.data}")
    
    def run_test(self, duration=10.0):
        """Run test for specified duration"""
        rospy.loginfo(f"Running test for {duration} seconds...")
        rospy.sleep(duration)
        
        # Print summary
        rospy.loginfo("="*60)
        rospy.loginfo("TEST SUMMARY")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Detections received: {len(self.detections)}")
        rospy.loginfo(f"Poses received: {len(self.poses)}")
        rospy.loginfo(f"Status messages: {len(self.status_msgs)}")
        
        if len(self.detections) > 0:
            rospy.loginfo("✓ Detection working")
        else:
            rospy.logwarn("✗ No detections received")
        
        if len(self.poses) > 0:
            rospy.loginfo("✓ Pose estimation working")
        else:
            rospy.logwarn("✗ No poses received")
        
        rospy.loginfo("="*60)
        
        return len(self.detections) > 0 and len(self.poses) > 0


def main():
    """Main test function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Test franka_perception')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Test duration in seconds')
    parser.add_argument('--sim', action='store_true',
                       help='Test in simulation mode')
    parser.add_argument('--real', action='store_true',
                       help='Test with real robot')
    
    args = parser.parse_args()
    
    try:
        tester = PerceptionTester()
        success = tester.run_test(args.duration)
        
        if success:
            rospy.loginfo("✓ Test PASSED")
            sys.exit(0)
        else:
            rospy.logwarn("✗ Test FAILED")
            sys.exit(1)
    
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Test error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
