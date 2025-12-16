#!/usr/bin/env python3
"""
Phase 3: RRT Benchmark Script
Evaluates RRTConnect planner performance across multiple scenarios.

This script:
1. Generates random target poses within the workspace
2. Plans trajectories from home to each target
3. Records success rate, planning time, and path metrics
4. Saves results to a CSV file for analysis

Usage:
    rosrun panda_grasp_planning benchmark_rrt.py --trials 100 --output results.csv
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import math
import time
import csv
import argparse
import numpy as np
from datetime import datetime
import os


class RRTBenchmark:
    def __init__(self, output_file='benchmark_results.csv'):
        """
        Initialize the benchmark system.
        
        Args:
            output_file: Path to save results CSV
        """
        # Initialize moveit_commander and rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('rrt_benchmark', anonymous=True)
        
        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        
        # Configure move group
        self.move_group.set_pose_reference_frame("panda_link0")
        self.move_group.set_planner_id("RRTConnect")
        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(10)
        
        # Workspace bounds (safety limits)
        self.workspace_bounds = {
            'x_min': 0.2,
            'x_max': 0.8,
            'y_min': -0.5,
            'y_max': 0.5,
            'z_min': 0.05,
            'z_max': 0.8
        }
        
        # Results storage
        self.results = []
        self.output_file = output_file
        
        rospy.loginfo("="*60)
        rospy.loginfo("RRT Benchmark System Initialized")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Planning group: panda_arm")
        rospy.loginfo(f"Planner: RRTConnect")
        rospy.loginfo(f"Output file: {output_file}")
        rospy.loginfo(f"Workspace bounds: X[{self.workspace_bounds['x_min']:.2f}, "
                     f"{self.workspace_bounds['x_max']:.2f}], "
                     f"Y[{self.workspace_bounds['y_min']:.2f}, "
                     f"{self.workspace_bounds['y_max']:.2f}], "
                     f"Z[{self.workspace_bounds['z_min']:.2f}, "
                     f"{self.workspace_bounds['z_max']:.2f}]")
        rospy.loginfo("="*60)
        
        rospy.sleep(2.0)
    
    def get_home_joint_values(self):
        """Define home configuration for Panda robot"""
        return [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]
    
    def move_to_home(self):
        """Move robot to home configuration"""
        rospy.loginfo("Moving to home configuration...")
        try:
            self.move_group.set_joint_value_target(self.get_home_joint_values())
            plan = self.move_group.plan()
            
            if isinstance(plan, tuple):
                success, trajectory, _, _ = plan
            else:
                success = plan
                trajectory = plan
            
            if success:
                self.move_group.execute(trajectory if isinstance(plan, tuple) else plan, wait=True)
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                rospy.loginfo("✓ Moved to home")
                return True
            else:
                rospy.logerr("Failed to plan to home")
                return False
        except Exception as e:
            rospy.logerr(f"Error moving to home: {e}")
            return False
    
    def generate_random_pose(self):
        """
        Generate a random target pose within workspace bounds.
        Gripper pointing down (standard grasp orientation).
        
        Returns:
            PoseStamped message
        """
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.header.stamp = rospy.Time.now()
        
        # Random position within bounds
        pose.pose.position.x = np.random.uniform(
            self.workspace_bounds['x_min'],
            self.workspace_bounds['x_max']
        )
        pose.pose.position.y = np.random.uniform(
            self.workspace_bounds['y_min'],
            self.workspace_bounds['y_max']
        )
        pose.pose.position.z = np.random.uniform(
            self.workspace_bounds['z_min'],
            self.workspace_bounds['z_max']
        )
        
        # Standard grasp orientation (gripper pointing down)
        # Add small random variations (±15 degrees)
        roll = math.pi + np.random.uniform(-math.pi/12, math.pi/12)
        pitch = 0.0 + np.random.uniform(-math.pi/12, math.pi/12)
        yaw = 0.0 + np.random.uniform(-math.pi/12, math.pi/12)
        
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
    
    def compute_path_length(self, trajectory):
        """
        Compute the arc length of a trajectory in joint space.
        
        Args:
            trajectory: RobotTrajectory message
        
        Returns:
            Total path length (sum of joint angle changes)
        """
        if isinstance(trajectory, moveit_msgs.msg.RobotTrajectory):
            points = trajectory.joint_trajectory.points
        else:
            # Handle tuple return from plan()
            return 0.0
        
        if len(points) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(points)):
            # Compute Euclidean distance in joint space
            delta = np.array(points[i].positions) - np.array(points[i-1].positions)
            total_length += np.linalg.norm(delta)
        
        return total_length
    
    def benchmark_single_trial(self, trial_num, target_pose):
        """
        Run a single benchmark trial.
        
        Args:
            trial_num: Trial number
            target_pose: Target pose to plan to
        
        Returns:
            Dictionary with trial results
        """
        result = {
            'trial': trial_num,
            'target_x': target_pose.pose.position.x,
            'target_y': target_pose.pose.position.y,
            'target_z': target_pose.pose.position.z,
            'success': False,
            'planning_time': 0.0,
            'path_length': 0.0,
            'num_waypoints': 0,
            'error_type': 'NONE'
        }
        
        try:
            # Set target
            self.move_group.set_pose_target(target_pose)
            
            # Plan with timing
            start_time = time.time()
            plan = self.move_group.plan()
            planning_time = time.time() - start_time
            
            # Handle both API versions
            if isinstance(plan, tuple):
                success, trajectory, _, error_code = plan
            else:
                success = bool(plan and hasattr(plan, 'joint_trajectory') and 
                              len(plan.joint_trajectory.points) > 0)
                trajectory = plan
                error_code = None
            
            result['planning_time'] = planning_time
            
            if success and (not isinstance(plan, tuple) or plan[0]):
                result['success'] = True
                result['path_length'] = self.compute_path_length(trajectory)
                
                if isinstance(trajectory, moveit_msgs.msg.RobotTrajectory):
                    result['num_waypoints'] = len(trajectory.joint_trajectory.points)
                
                rospy.loginfo(f"  ✓ Trial {trial_num}: SUCCESS "
                             f"(time: {planning_time:.3f}s, "
                             f"length: {result['path_length']:.3f})")
            else:
                result['success'] = False
                if error_code:
                    result['error_type'] = str(error_code)
                else:
                    result['error_type'] = 'PLANNING_FAILED'
                rospy.logwarn(f"  ✗ Trial {trial_num}: FAILED "
                             f"(time: {planning_time:.3f}s, "
                             f"error: {result['error_type']})")
            
            # Clear targets
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
        except Exception as e:
            result['success'] = False
            result['error_type'] = f'EXCEPTION: {str(e)[:50]}'
            rospy.logerr(f"  ✗ Trial {trial_num}: EXCEPTION: {e}")
        
        return result
    
    def run_benchmark(self, num_trials=100, return_home_interval=10):
        """
        Run complete benchmark suite.
        
        Args:
            num_trials: Number of random trials to run
            return_home_interval: Return to home every N trials
        """
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo(f"STARTING BENCHMARK: {num_trials} trials")
        rospy.loginfo("="*60 + "\n")
        
        # Move to home at start
        if not self.move_to_home():
            rospy.logerr("Failed to reach home position. Aborting benchmark.")
            return
        
        rospy.sleep(1.0)
        
        start_time = time.time()
        
        for trial in range(1, num_trials + 1):
            rospy.loginfo(f"\n[{trial}/{num_trials}] Generating random target...")
            
            # Generate random target
            target_pose = self.generate_random_pose()
            
            rospy.loginfo(f"  Target: [{target_pose.pose.position.x:.3f}, "
                         f"{target_pose.pose.position.y:.3f}, "
                         f"{target_pose.pose.position.z:.3f}]")
            
            # Run trial
            result = self.benchmark_single_trial(trial, target_pose)
            self.results.append(result)
            
            # Periodically return to home
            if trial % return_home_interval == 0:
                rospy.loginfo(f"\n>>> Returning to home (every {return_home_interval} trials)")
                self.move_to_home()
                rospy.sleep(1.0)
        
        total_time = time.time() - start_time
        
        # Print summary
        self.print_summary(total_time)
        
        # Save results
        self.save_results()
    
    def print_summary(self, total_time):
        """Print benchmark summary statistics"""
        if not self.results:
            rospy.logwarn("No results to summarize")
            return
        
        successful = [r for r in self.results if r['success']]
        failed = [r for r in self.results if not r['success']]
        
        success_rate = len(successful) / len(self.results) * 100
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("BENCHMARK SUMMARY")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Total trials: {len(self.results)}")
        rospy.loginfo(f"Successful: {len(successful)} ({success_rate:.1f}%)")
        rospy.loginfo(f"Failed: {len(failed)} ({100-success_rate:.1f}%)")
        rospy.loginfo(f"Total time: {total_time:.1f}s")
        rospy.loginfo(f"Avg time per trial: {total_time/len(self.results):.2f}s")
        
        if successful:
            planning_times = [r['planning_time'] for r in successful]
            path_lengths = [r['path_length'] for r in successful]
            waypoints = [r['num_waypoints'] for r in successful]
            
            rospy.loginfo("\n--- Successful Trials Statistics ---")
            rospy.loginfo(f"Planning time: {np.mean(planning_times):.3f}s "
                         f"(±{np.std(planning_times):.3f}s)")
            rospy.loginfo(f"  Min: {np.min(planning_times):.3f}s, "
                         f"Max: {np.max(planning_times):.3f}s")
            rospy.loginfo(f"Path length: {np.mean(path_lengths):.3f} rad "
                         f"(±{np.std(path_lengths):.3f})")
            rospy.loginfo(f"Waypoints: {np.mean(waypoints):.1f} "
                         f"(±{np.std(waypoints):.1f})")
        
        if failed:
            rospy.loginfo("\n--- Failure Analysis ---")
            error_types = {}
            for r in failed:
                error_type = r['error_type']
                error_types[error_type] = error_types.get(error_type, 0) + 1
            
            for error, count in sorted(error_types.items(), 
                                      key=lambda x: x[1], 
                                      reverse=True):
                rospy.loginfo(f"  {error}: {count} ({count/len(failed)*100:.1f}%)")
        
        rospy.loginfo("="*60 + "\n")
    
    def save_results(self):
        """Save results to CSV file"""
        if not self.results:
            rospy.logwarn("No results to save")
            return
        
        try:
            # Add timestamp to filename if not already present
            base_name, ext = os.path.splitext(self.output_file)
            if '_' not in base_name:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_file = f"{base_name}_{timestamp}{ext}"
            else:
                output_file = self.output_file
            
            with open(output_file, 'w', newline='') as csvfile:
                fieldnames = ['trial', 'target_x', 'target_y', 'target_z',
                            'success', 'planning_time', 'path_length',
                            'num_waypoints', 'error_type']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                writer.writeheader()
                for result in self.results:
                    writer.writerow(result)
            
            rospy.loginfo(f"✓ Results saved to: {output_file}")
            
        except Exception as e:
            rospy.logerr(f"Failed to save results: {e}")


def main():
    parser = argparse.ArgumentParser(description='Benchmark RRTConnect planner performance')
    parser.add_argument('--trials', type=int, default=100,
                       help='Number of random trials to run (default: 100)')
    parser.add_argument('--output', type=str, default='benchmark_results.csv',
                       help='Output CSV file (default: benchmark_results.csv)')
    parser.add_argument('--home-interval', type=int, default=10,
                       help='Return to home every N trials (default: 10)')
    
    # Parse only known args to avoid ROS args issues
    args, unknown = parser.parse_known_args()
    
    try:
        benchmark = RRTBenchmark(output_file=args.output)
        benchmark.run_benchmark(num_trials=args.trials, 
                               return_home_interval=args.home_interval)
        
        rospy.loginfo("\n✓ Benchmark completed successfully!")
        rospy.loginfo(f"Results saved to: {args.output}")
        rospy.loginfo("Run analyze_results.py to generate detailed analysis.\n")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Benchmark interrupted by user.")
    except Exception as e:
        rospy.logerr(f"Benchmark error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
