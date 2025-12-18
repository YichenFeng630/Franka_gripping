#!/usr/bin/env python3
"""
Simple V3 test script - send multiple targets sequentially
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import sys

class SimpleV3Tester:
    def __init__(self, num_tests=3):
        rospy.init_node('v3_simple_test', anonymous=True)
        
        self.pub = rospy.Publisher('/target_cube_pose', PoseStamped, queue_size=1)
        self.status = None
        self.status_sub = rospy.Subscriber('/grasp_planning_status', String, self.status_callback)
        
        self.num_tests = num_tests
        self.results = []
        
        # Wait for publisher/subscriber setup
        rospy.sleep(2.0)
    
    def status_callback(self, msg):
        """Update status"""
        self.status = msg.data
    
    def send_target(self, x, y, z):
        """Send target pose"""
        pose = PoseStamped()
        pose.header.frame_id = 'panda_link0'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        self.pub.publish(pose)
        rospy.loginfo(f"[TEST] Sent target: ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def wait_for_completion(self, timeout=120):
        """Wait for SUCCESS or FAILED status"""
        start = time.time()
        last_status = None
        
        while time.time() - start < timeout:
            if self.status and self.status != last_status:
                rospy.loginfo(f"[STATUS] {self.status}")
                last_status = self.status
                
                if 'SUCCESS' in self.status or 'FAILED' in self.status:
                    elapsed = time.time() - start
                    return self.status, elapsed
            
            rospy.sleep(0.5)
        
        return None, timeout
    
    def run(self):
        """Run test sequence"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("V3 SIMPLE TEST - SEQUENTIAL TARGETS")
        rospy.loginfo(f"Total Tests: {self.num_tests}")
        rospy.loginfo("="*70 + "\n")
        
        targets = [
            (0.5, 0.0, 0.10, "Test 1: Center"),
            (0.45, 0.1, 0.12, "Test 2: Left"),
            (0.55, -0.05, 0.11, "Test 3: Right"),
        ][:self.num_tests]
        
        for idx, (x, y, z, desc) in enumerate(targets):
            rospy.loginfo(f"\n{'='*70}")
            rospy.loginfo(f"TEST {idx+1}/{self.num_tests}: {desc}")
            rospy.loginfo(f"{'='*70}")
            
            # Reset status
            self.status = None
            
            # Send target
            self.send_target(x, y, z)
            
            # Wait for completion
            final_status, elapsed = self.wait_for_completion()
            
            if final_status:
                success = 'SUCCESS' in final_status
                result = {
                    'idx': idx + 1,
                    'desc': desc,
                    'status': final_status,
                    'time': elapsed,
                    'success': success
                }
                self.results.append(result)
                
                status_str = "✓ PASS" if success else "✗ FAIL"
                rospy.loginfo(f"\n{status_str}: {final_status} ({elapsed:.1f}s)")
            else:
                rospy.logerr(f"\n✗ TIMEOUT after {elapsed:.1f}s")
                self.results.append({
                    'idx': idx + 1,
                    'desc': desc,
                    'status': 'TIMEOUT',
                    'time': elapsed,
                    'success': False
                })
            
            # Wait between tests
            if idx < len(targets) - 1:
                rospy.loginfo(f"\nWaiting 3 seconds before next test...")
                rospy.sleep(3.0)
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print test summary"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("TEST SUMMARY")
        rospy.loginfo("="*70)
        
        total = len(self.results)
        passed = sum(1 for r in self.results if r['success'])
        failed = total - passed
        
        rospy.loginfo(f"Total: {total}")
        rospy.loginfo(f"Passed: {passed}")
        rospy.loginfo(f"Failed: {failed}")
        rospy.loginfo(f"Success Rate: {100*passed//total}%\n")
        
        for r in self.results:
            status_str = "✓" if r['success'] else "✗"
            rospy.loginfo(f"  {status_str} Test {r['idx']}: {r['status']} ({r['time']:.1f}s)")
        
        rospy.loginfo("="*70)
        
        return passed == total

if __name__ == '__main__':
    try:
        tester = SimpleV3Tester(num_tests=3)
        success = tester.run()
        sys.exit(0 if success else 1)
    except rospy.ROSInterruptException:
        sys.exit(1)
