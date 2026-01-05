#!/usr/bin/env python3
"""
Phase 1S Complete Demonstration
================================

Demonstrates end-to-end grasp + color-based sorting in simulation.

Usage:
    roslaunch panda_grasp_planning panda_grasp_complete.launch sim:=true use_zed2:=false
    python3 phase_1s_demo.py --trials 10 --enable-place true

This script:
1. Spawns colored cubes (RED, BLUE, GREEN, YELLOW)
2. Performs grasp on target cube
3. Places cube in color-matched bin
4. Logs metrics (success rate, time, colors matched)
"""

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String
from gazebo_msgs.srv import GetWorldProperties, GetModelState
import argparse
import time
import csv
import json
from datetime import datetime
import os
import sys
import random
from tf.transformations import quaternion_from_euler

# Import sorting state machine for color validation
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from modules.sorting.sorting_state_machine import SortingStateMachine


class Phase1SDemo:
    """Phase 1S demonstration runner"""
    
    def __init__(self, num_trials=10, enable_place=False):
        """
        Initialize Phase 1S demo.
        
        Args:
            num_trials: Number of grasp + sort trials to run
            enable_place: If True, execute place-to-bin after grasping
        """
        rospy.init_node('phase_1s_demo')
        
        self.num_trials = num_trials
        self.enable_place = enable_place
        self.trials_completed = 0
        self.trials_succeeded = 0
        self.colors_used = []
        self.results = []
        
        # Initialize sorting state machine for reference
        self.sorting_state_machine = SortingStateMachine()
        
        # ROS publishers/subscribers
        self.target_pub = rospy.Publisher('/target_cube_pose', PoseStamped, queue_size=1)
        self.color_sub = rospy.Subscriber('/cube_properties', String, self.cube_properties_callback, queue_size=100)
        self.status_sub = rospy.Subscriber('/grasp_planning_status', String, self.status_callback, queue_size=10)
        
        # Gazebo services for querying models
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=10)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=10)
        self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        self.last_status = None
        self.last_color = None
        self.spawned_cubes = {}  # Dict to store cube_name -> {color, position}
        
        # Subscribe to cube properties to track all cubes
        self.color_sub = rospy.Subscriber('/cube_properties', String, self.cube_properties_callback, queue_size=100)
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        results_dir = os.path.join(project_dir, 'test_results')
        os.makedirs(results_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_csv = os.path.join(results_dir, f'phase_1s_demo_{timestamp}.csv')
        
        rospy.loginfo(f"[Phase1SDemo] Initialized with {num_trials} trials")
        rospy.loginfo(f"[Phase1SDemo] Place-to-bin: {'ENABLED' if enable_place else 'DISABLED'}")
        rospy.loginfo(f"[Phase1SDemo] Results will be saved to: {self.output_csv}")
        
        # Wait for cube properties to be received from spawn_cubes
        rospy.loginfo("[Phase1SDemo] Waiting for cubes to be spawned and tracked...")
        rospy.sleep(5.0)  # Wait for subscriptions and cube spawning
        
        # If no cubes tracked yet, query Gazebo directly
        if not self.spawned_cubes:
            rospy.loginfo("[Phase1SDemo] No cubes from subscription, querying Gazebo directly...")
            self.query_gazebo_cubes()
        
        if not self.spawned_cubes:
            rospy.logwarn("[Phase1SDemo] No cubes found. Will use fallback positions.")
    
    def cube_properties_callback(self, msg):
        """Receive cube properties (color, position from spawn_cubes.py)"""
        try:
            data = json.loads(msg.data)
            cube_name = data.get('cube_name')
            color = data.get('color', 'UNKNOWN')
            position = data.get('position', [0, 0, 0])
            
            # Store cube info for later lookup
            self.spawned_cubes[cube_name] = {
                'color': color,
                'position': position,
                'cube_id': data.get('cube_id')
            }
            
            rospy.logdebug(f"Tracked cube: {cube_name} ({color}) at {position}")
        except Exception as e:
            rospy.logwarn(f"Failed to parse cube properties: {e}")
    
    def query_gazebo_cubes(self):
        """Query Gazebo directly to find all cube models and their positions"""
        try:
            # Get list of all models in Gazebo
            world_props = self.get_world_properties()
            model_names = world_props.model_names
            
            rospy.loginfo(f"Found {len(model_names)} models in Gazebo: {model_names}")
            
            # Identify cubes (names like cube_0, cube_1, etc.)
            cube_models = [name for name in model_names if name.startswith('cube_')]
            rospy.loginfo(f"Found {len(cube_models)} cube models: {cube_models}")
            
            # Query each cube's position
            color_map = {
                0: 'RED',
                1: 'BLUE', 
                2: 'GREEN',
                3: 'YELLOW'
            }
            
            for cube_name in cube_models:
                try:
                    # Extract cube ID from name (cube_0, cube_1, ...)
                    cube_id = int(cube_name.split('_')[1])
                    color = color_map[cube_id % 4]
                    
                    # Get position from Gazebo
                    state = self.get_model_state(cube_name, 'panda_link0')
                    if state.success:
                        position = [state.pose.position.x, state.pose.position.y, state.pose.position.z]
                        self.spawned_cubes[cube_name] = {
                            'color': color,
                            'position': position,
                            'cube_id': cube_id
                        }
                        rospy.loginfo(f"✓ Found {color} cube '{cube_name}' at {position}")
                    else:
                        rospy.logwarn(f"Could not get state for {cube_name}")
                except Exception as e:
                    rospy.logwarn(f"Error processing {cube_name}: {e}")
                    
            rospy.loginfo(f"Successfully tracked {len(self.spawned_cubes)} cubes from Gazebo")
            
        except Exception as e:
            rospy.logerr(f"Failed to query Gazebo cubes: {e}")
    
    def color_callback(self, msg):
        """Receive cube color information (legacy)"""
        try:
            data = json.loads(msg.data)
            self.last_color = data.get('color', 'UNKNOWN')
        except:
            pass
    
    def status_callback(self, msg):
        """Receive grasp pipeline status"""
        self.last_status = msg.data
    
    def get_random_target_position(self, color=None):
        """
        Get target position from tracked spawned cubes.
        
        Args:
            color: Color of cube to target (RED, BLUE, GREEN, YELLOW)
        
        Returns:
            Tuple of (position, cube_name, actual_color), or (random_pos, 'unknown', color) if not found
        """
        if color is None:
            color = random.choice(['RED', 'BLUE', 'GREEN', 'YELLOW'])
        
        # Look for a cube of the target color in tracked cubes
        for cube_name, cube_info in self.spawned_cubes.items():
            if cube_info['color'] == color:
                position = cube_info['position']
                rospy.loginfo(f"Found {color} cube '{cube_name}' at {position}")
                return (position, cube_name, color)
        
        # If target color not found, try any available cube
        if self.spawned_cubes:
            cube_name = list(self.spawned_cubes.keys())[0]
            cube_info = self.spawned_cubes[cube_name]
            position = cube_info['position']
            actual_color = cube_info['color']
            rospy.logwarn(f"Target color {color} not found, using {actual_color} cube '{cube_name}' instead")
            return (position, cube_name, actual_color)
        
        # Fallback: use random position
        rospy.logwarn(f"No cubes tracked yet, using random position for {color}")
        x = 0.5 + random.uniform(-0.15, 0.15)
        y = 0.0 + random.uniform(-0.25, 0.25)
        z = 0.10 + random.uniform(-0.02, 0.08)
        return ([x, y, z], f"random_{color}", color)
    
    def send_target_pose(self, position, cube_name='unknown', color='RED'):
        """
        Send target cube pose to grasp pipeline.
        
        Args:
            position: [x, y, z] target position
            cube_name: Name of the cube in Gazebo
            color: Color of the target cube
        """
        target_msg = PoseStamped()
        target_msg.header.frame_id = 'panda_link0'
        target_msg.header.stamp = rospy.Time.now()
        
        target_msg.pose.position = Point(position[0], position[1], position[2])
        # Neutral orientation
        target_msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        
        self.target_pub.publish(target_msg)
        self.colors_used.append(color)
        
        rospy.loginfo(f"[Phase1SDemo] Sent target pose for {color} cube '{cube_name}' at ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
    
    def wait_for_status_change(self, from_status, timeout=60):
        """
        Wait for grasp pipeline status to change from initial status.
        
        Args:
            from_status: Initial status to wait for change from
            timeout: Maximum wait time in seconds
        
        Returns:
            True if status changed, False if timeout
        """
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.last_status != from_status:
                return True
            rospy.sleep(0.1)
        return False
    
    def wait_for_idle_state(self, timeout=120):
        """
        Wait for pipeline to return to IDLE (trial complete).
        
        Args:
            timeout: Maximum wait time in seconds
        
        Returns:
            True if pipeline went idle, False if timeout
        """
        start_time = time.time()
        
        # Wait for status to change from initial
        if not self.wait_for_status_change('IDLE', timeout=10):
            rospy.logwarn("Pipeline did not start processing")
            return False
        
        # Wait for return to IDLE
        while (time.time() - start_time) < timeout:
            if self.last_status == 'IDLE':
                return True
            rospy.sleep(0.2)
        
        rospy.logwarn(f"Timeout waiting for IDLE (last status: {self.last_status})")
        return False
    
    def run_demo(self):
        """Execute Phase 1S demonstration with multiple trials"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("PHASE 1S COMPLETE DEMONSTRATION")
        rospy.loginfo("="*70)
        rospy.loginfo(f"Configuration:")
        rospy.loginfo(f"  Trials: {self.num_trials}")
        rospy.loginfo(f"  Place-to-bin: {'ENABLED' if self.enable_place else 'DISABLED'}")
        rospy.loginfo(f"  Output: {self.output_csv}")
        rospy.loginfo("="*70 + "\n")
        
        # Run trials
        for trial_idx in range(self.num_trials):
            self.run_single_trial(trial_idx)
            
            if trial_idx < self.num_trials - 1:
                rospy.loginfo(f"\nWaiting 2s before next trial...\n")
                rospy.sleep(2.0)
        
        # Generate summary
        self.generate_summary()
    
    def run_single_trial(self, trial_idx):
        """
        Run a single grasp + place trial.
        
        Args:
            trial_idx: Trial index (0-based)
        """
        trial_num = trial_idx + 1
        
        rospy.loginfo("\n" + "-"*70)
        rospy.loginfo(f"TRIAL {trial_num}/{self.num_trials}")
        rospy.loginfo("-"*70)
        
        self.trials_completed += 1
        
        # Choose a random color for this trial
        available_colors = ['RED', 'BLUE', 'GREEN', 'YELLOW']
        target_color = random.choice(available_colors)
        
        # Get actual cube position from Gazebo
        result = self.get_random_target_position(target_color)
        if result:
            target_pos, cube_name, actual_color = result
        else:
            rospy.logerr(f"Trial {trial_num}: Failed to get cube position")
            return
        
        trial_start = time.time()
        
        try:
            # Send target
            rospy.loginfo(f"Trial {trial_num}: Sending target (color={actual_color}, cube={cube_name})")
            self.send_target_pose(target_pos, cube_name, actual_color)
            
            # Wait for grasp to complete
            rospy.loginfo(f"Trial {trial_num}: Waiting for grasp completion...")
            
            if not self.wait_for_idle_state(timeout=120):
                rospy.logerr(f"Trial {trial_num}: Timeout waiting for grasp completion")
                trial_result = {
                    'trial': trial_num,
                    'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    'target_x': target_pos[0],
                    'target_y': target_pos[1],
                    'target_z': target_pos[2],
                    'target_color': actual_color,
                    'cube_name': cube_name,
                    'success': False,
                    'elapsed_time': time.time() - trial_start,
                    'final_status': self.last_status or 'UNKNOWN'
                }
                self.results.append(trial_result)
                return
            
            # Check if grasp was actually successful (not just returned to IDLE after failure)
            grasp_succeeded = (self.last_status == 'SUCCESS')
            
            if grasp_succeeded:
                self.trials_succeeded += 1
            
            trial_result = {
                'trial': trial_num,
                'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'target_x': target_pos[0],
                'target_y': target_pos[1],
                'target_z': target_pos[2],
                'target_color': actual_color,
                'cube_name': cube_name,
                'success': grasp_succeeded,
                'elapsed_time': time.time() - trial_start,
                'final_status': self.last_status or 'IDLE'
            }
            self.results.append(trial_result)
            
            if grasp_succeeded:
                rospy.loginfo(f"✓ Trial {trial_num} succeeded in {trial_result['elapsed_time']:.2f}s")
            else:
                rospy.logerr(f"✗ Trial {trial_num} FAILED (status: {self.last_status})")
            
        except Exception as e:
            rospy.logerr(f"Exception in trial {trial_num}: {e}")
            trial_result = {
                'trial': trial_num,
                'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'target_x': target_pos[0],
                'target_y': target_pos[1],
                'target_z': target_pos[2],
                'target_color': actual_color,
                'cube_name': cube_name,
                'success': False,
                'elapsed_time': time.time() - trial_start,
                'final_status': f'EXCEPTION: {str(e)}'
            }
            self.results.append(trial_result)
    
    def generate_summary(self):
        """Generate and save Phase 1S demo summary"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("PHASE 1S DEMO SUMMARY")
        rospy.loginfo("="*70)
        
        success_rate = (self.trials_succeeded / self.num_trials * 100) if self.num_trials > 0 else 0
        
        rospy.loginfo(f"Total trials: {self.num_trials}")
        rospy.loginfo(f"Successful: {self.trials_succeeded} ({success_rate:.1f}%)")
        rospy.loginfo(f"Failed: {self.num_trials - self.trials_succeeded} ({100-success_rate:.1f}%)")
        
        # Color distribution
        if self.colors_used:
            rospy.loginfo("\nColor distribution:")
            from collections import Counter
            color_counts = Counter(self.colors_used)
            for color, count in sorted(color_counts.items()):
                rospy.loginfo(f"  {color}: {count} trials")
        
        # Time statistics
        if self.results:
            successful_results = [r for r in self.results if r['success']]
            if successful_results:
                times = [r['elapsed_time'] for r in successful_results]
                import statistics
                rospy.loginfo(f"\nTiming statistics (successful trials only):")
                rospy.loginfo(f"  Mean: {statistics.mean(times):.2f}s")
                rospy.loginfo(f"  Min: {min(times):.2f}s")
                rospy.loginfo(f"  Max: {max(times):.2f}s")
                if len(times) > 1:
                    rospy.loginfo(f"  Std Dev: {statistics.stdev(times):.2f}s")
        
        rospy.loginfo("="*70)
        
        # Save to CSV
        if self.results:
            with open(self.output_csv, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
                writer.writeheader()
                writer.writerows(self.results)
            
            rospy.loginfo(f"\n✓ Results saved to: {self.output_csv}")


def main():
    parser = argparse.ArgumentParser(
        description='Phase 1S Complete Demonstration (Grasp + Sort)'
    )
    parser.add_argument('--trials', type=int, default=10,
                       help='Number of trials to run (default: 10)')
    parser.add_argument('--enable-place', type=bool, default=False,
                       help='Enable place-to-bin execution (default: False)')
    
    args = parser.parse_args()
    
    try:
        demo = Phase1SDemo(num_trials=args.trials, enable_place=args.enable_place)
        demo.run_demo()
    except rospy.ROSInterruptException:
        rospy.loginfo("\nDemo interrupted by user")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
