#!/usr/bin/env python3
"""
Test script for simplified grasp pipeline
Tests the direct joint-space planning approach
"""

import rospy
import sys
import os
import time
import argparse
from datetime import datetime
import moveit_commander
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import the simplified pipeline
from scripts.grasp_pipeline_simplified import SimplifiedGraspPipeline


def run_grasp_trials(num_trials=5):
    """Run multiple grasp trials"""
    
    rospy.loginfo("\n" + "="*70)
    rospy.loginfo(f"SIMPLIFIED GRASP PIPELINE TEST")
    rospy.loginfo(f"Trials: {num_trials}")
    rospy.loginfo("="*70)
    
    try:
        # Initialize pipeline
        pipeline = SimplifiedGraspPipeline()
        
        # Create cube center pose (as PoseStamped)
        cube_center = PoseStamped()
        cube_center.header.frame_id = "panda_link0"
        cube_center.header.stamp = rospy.Time.now()
        cube_center.pose.position.x = 0.3
        cube_center.pose.position.y = 0.0
        cube_center.pose.position.z = 0.02
        q = quaternion_from_euler(0, 0, 0)
        cube_center.pose.orientation.x = q[0]
        cube_center.pose.orientation.y = q[1]
        cube_center.pose.orientation.z = q[2]
        cube_center.pose.orientation.w = q[3]
        
        results = {
            'total': num_trials,
            'success': 0,
            'failed': 0,
            'trials': []
        }
        
        # Run trials
        for trial_num in range(1, num_trials + 1):
            rospy.loginfo(f"\n\nTRIAL {trial_num}/{num_trials}")
            rospy.loginfo("-" * 70)
            
            start_time = time.time()
            success = pipeline.execute_grasp(cube_center)
            trial_time = time.time() - start_time
            
            results['trials'].append({
                'trial': trial_num,
                'success': success,
                'time': trial_time
            })
            
            if success:
                results['success'] += 1
            else:
                results['failed'] += 1
            
            rospy.loginfo(f"Trial {trial_num} time: {trial_time:.2f}s, "
                         f"Result: {'SUCCESS' if success else 'FAILED'}")
            
            # Wait between trials
            if trial_num < num_trials:
                rospy.loginfo(f"Waiting 3s before next trial...")
                rospy.sleep(3.0)
        
        # Print summary
        print_summary(results)
        
        return results
        
    except Exception as e:
        rospy.logerr(f"Test error: {e}")
        return None


def print_summary(results):
    """Print test summary"""
    
    if results is None:
        rospy.logerr("No results to print")
        return
    
    success_rate = 100.0 * results['success'] / results['total']
    avg_time = sum(t['time'] for t in results['trials']) / results['total']
    
    rospy.loginfo("\n" + "="*70)
    rospy.loginfo("TEST SUMMARY")
    rospy.loginfo("="*70)
    rospy.loginfo(f"Total trials:    {results['total']}")
    rospy.loginfo(f"Successful:      {results['success']}")
    rospy.loginfo(f"Failed:          {results['failed']}")
    rospy.loginfo(f"Success rate:    {success_rate:.1f}%")
    rospy.loginfo(f"Avg time/trial:  {avg_time:.2f}s")
    rospy.loginfo("="*70 + "\n")
    
    # Trial details
    rospy.loginfo("Trial Details:")
    for trial in results['trials']:
        status = "✓ SUCCESS" if trial['success'] else "✗ FAILED"
        rospy.loginfo(f"  Trial {trial['trial']}: {status} ({trial['time']:.2f}s)")
    
    rospy.loginfo("\n" + "="*70)
    if success_rate >= 90:
        rospy.loginfo(f"✓✓✓ EXCELLENT! Success rate: {success_rate:.1f}% (≥90%) ✓✓✓")
    elif success_rate >= 70:
        rospy.loginfo(f"✓ GOOD! Success rate: {success_rate:.1f}%")
    else:
        rospy.loginfo(f"✗ NEEDS IMPROVEMENT. Success rate: {success_rate:.1f}%")
    rospy.loginfo("="*70 + "\n")


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Test simplified grasp pipeline")
    parser.add_argument("--trials", type=int, default=5,
                       help="Number of trials to run (default: 5)")
    args = parser.parse_args()
    
    # Initialize ROS node first
    rospy.init_node('simplified_grasp_test', anonymous=True)
    
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    
    try:
        results = run_grasp_trials(num_trials=args.trials)
    except KeyboardInterrupt:
        rospy.loginfo("\nTest interrupted by user")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
    finally:
        moveit_commander.roscpp_shutdown()
