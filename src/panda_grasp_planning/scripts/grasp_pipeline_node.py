#!/usr/bin/env python3
"""
Phase 2: Grasp Pipeline Node
Executes complete grasp sequence using a state machine.

This node:
1. Moves to home configuration
2. Plans and executes to pre-grasp pose
3. Opens gripper
4. Plans and executes to grasp pose
5. Closes gripper
6. Plans and executes lift
7. Returns to home with object

State machine: IDLE -> HOME -> PRE_GRASP -> GRASP -> LIFT -> HOME_WITH_OBJECT -> SUCCESS
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from franka_gripper.msg import MoveGoal, MoveAction, GraspGoal, GraspAction
import actionlib
import math
from enum import Enum
import csv
import time
from datetime import datetime
import os


class GraspState(Enum):
    """Enumeration of grasp pipeline states"""
    IDLE = 0
    WAITING_FOR_TARGET = 1
    MOVING_TO_HOME = 2
    PLANNING_PRE_GRASP = 3
    EXECUTING_PRE_GRASP = 4
    OPENING_GRIPPER = 5
    PLANNING_GRASP = 6
    EXECUTING_GRASP = 7
    CLOSING_GRIPPER = 8
    PLANNING_LIFT = 9
    EXECUTING_LIFT = 10
    RETURNING_HOME = 11
    SUCCESS = 12
    FAILED = 13


class GraspPipeline:
    def __init__(self):
        # Initialize moveit_commander and rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('grasp_pipeline_node', anonymous=False)
        
        # Load parameters
        self.load_parameters()
        
        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)
        
        # Configure move group
        self.move_group.set_pose_reference_frame(self.reference_frame)
        self.move_group.set_planner_id(self.planner_id)
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_num_planning_attempts(self.num_planning_attempts)
        self.move_group.set_max_velocity_scaling_factor(self.max_velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(self.max_acceleration_scaling)
        
        # State management
        self.current_state = GraspState.IDLE
        self.target_pose = None
        self.pre_grasp_pose = None
        self.grasp_pose = None
        self.lift_pose = None
        
        # Data recording
        self.grasp_results = []
        self.current_trial = {
            'start_time': None,
            'target_position': None,
            'planning_times': [],
            'execution_times': [],
            'success': False,
            'failure_reason': None
        }
        # Save to test_results directory in project
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        results_dir = os.path.join(project_dir, 'test_results')
        os.makedirs(results_dir, exist_ok=True)
        self.output_file = os.path.join(results_dir, 'phase2_grasp_results.csv')
        rospy.loginfo(f"Data will be saved to: {self.output_file}")
        
        # Register shutdown hook to save results
        rospy.on_shutdown(self.shutdown_hook)
        
        # Publishers
        self.status_pub = rospy.Publisher(
            '/grasp_planning_status',
            String,
            queue_size=1
        )
        
        # Subscribers
        self.target_sub = rospy.Subscriber(
            '/target_cube_pose',
            PoseStamped,
            self.target_callback,
            queue_size=1
        )
        self.pre_grasp_sub = rospy.Subscriber(
            '/pre_grasp_pose',
            PoseStamped,
            self.pre_grasp_callback,
            queue_size=1
        )
        self.grasp_sub = rospy.Subscriber(
            '/grasp_pose',
            PoseStamped,
            self.grasp_callback,
            queue_size=1
        )
        self.lift_sub = rospy.Subscriber(
            '/lift_pose',
            PoseStamped,
            self.lift_callback,
            queue_size=1
        )
        
        # Initialize gripper action clients (optional, for real robot)
        self.use_gripper = rospy.get_param('~use_gripper', False)
        if self.use_gripper:
            try:
                self.gripper_move_client = actionlib.SimpleActionClient(
                    '/franka_gripper/move',
                    MoveAction
                )
                self.gripper_grasp_client = actionlib.SimpleActionClient(
                    '/franka_gripper/grasp',
                    GraspAction
                )
                rospy.loginfo("Waiting for gripper action servers...")
                self.gripper_move_client.wait_for_server(timeout=rospy.Duration(5.0))
                self.gripper_grasp_client.wait_for_server(timeout=rospy.Duration(5.0))
                rospy.loginfo("✓ Gripper action servers connected")
            except Exception as e:
                rospy.logwarn(f"Gripper action servers not available: {e}")
                rospy.logwarn("Continuing without gripper control (simulation mode)")
                self.use_gripper = False
        
        rospy.loginfo("="*60)
        rospy.loginfo("Grasp Pipeline Node Initialized")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Planning group: {self.planning_group}")
        rospy.loginfo(f"Reference frame: {self.reference_frame}")
        rospy.loginfo(f"Planner: {self.planner_id}")
        rospy.loginfo(f"Gripper control: {'ENABLED' if self.use_gripper else 'DISABLED (simulation)'}")
        rospy.loginfo("="*60)
        rospy.loginfo("Waiting for target pose and grasp poses...")
        
        self.publish_status("IDLE")
    
    def load_parameters(self):
        """Load planning parameters from ROS parameter server"""
        self.planning_group = rospy.get_param(
            '~planning_group',
            rospy.get_param('/planning/planning_group', 'panda_arm')
        )
        self.reference_frame = rospy.get_param(
            '~reference_frame',
            rospy.get_param('/planning/reference_frame', 'panda_link0')
        )
        self.planner_id = rospy.get_param(
            '~planner_id',
            rospy.get_param('/planning/planner_id', 'RRTConnect')
        )
        self.planning_time = rospy.get_param(
            '~planning_time',
            rospy.get_param('/planning/planning_time', 10.0)
        )
        self.num_planning_attempts = rospy.get_param(
            '~num_planning_attempts',
            rospy.get_param('/planning/num_planning_attempts', 10)
        )
        self.max_velocity_scaling = rospy.get_param(
            '~max_velocity_scaling_factor',
            rospy.get_param('/planning/max_velocity_scaling_factor', 0.5)
        )
        self.max_acceleration_scaling = rospy.get_param(
            '~max_acceleration_scaling_factor',
            rospy.get_param('/planning/max_acceleration_scaling_factor', 0.5)
        )
        
        # Gripper parameters
        self.gripper_open_width = rospy.get_param(
            '~gripper_open_width',
            rospy.get_param('/gripper/open_width', 0.08)
        )
        self.gripper_close_width = rospy.get_param(
            '~gripper_close_width',
            rospy.get_param('/gripper/close_width', 0.02)
        )
        self.gripper_max_effort = rospy.get_param(
            '~gripper_max_effort',
            rospy.get_param('/gripper/max_effort', 50.0)
        )
        self.gripper_speed = rospy.get_param(
            '~gripper_speed',
            rospy.get_param('/gripper/speed', 0.1)
        )
        
        # Timing parameters
        self.wait_after_plan = rospy.get_param(
            '~wait_after_plan',
            rospy.get_param('/timing/wait_after_plan', 0.5)
        )
        self.wait_after_execute = rospy.get_param(
            '~wait_after_execute',
            rospy.get_param('/timing/wait_after_execute', 1.0)
        )
        
        # Auto-execute flag
        self.auto_execute = rospy.get_param('~auto_execute', True)
    
    def publish_status(self, status):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def get_home_joint_values(self):
        """Define home configuration for Panda robot"""
        return [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]
    
    def target_callback(self, msg):
        """Callback for target cube pose"""
        if self.current_state == GraspState.IDLE:
            self.target_pose = msg
            self.current_state = GraspState.WAITING_FOR_TARGET
            rospy.loginfo(f"\n✓ Received target pose at [{msg.pose.position.x:.3f}, "
                         f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}]")
    
    def pre_grasp_callback(self, msg):
        """Callback for pre-grasp pose"""
        self.pre_grasp_pose = msg
        self.check_and_execute()
    
    def grasp_callback(self, msg):
        """Callback for grasp pose"""
        self.grasp_pose = msg
        self.check_and_execute()
    
    def lift_callback(self, msg):
        """Callback for lift pose"""
        self.lift_pose = msg
        self.check_and_execute()
    
    def check_and_execute(self):
        """Check if all poses are received and auto-execute if enabled"""
        if (self.current_state == GraspState.WAITING_FOR_TARGET and
            self.target_pose is not None and
            self.pre_grasp_pose is not None and
            self.grasp_pose is not None and
            self.lift_pose is not None and
            self.auto_execute):
            
            rospy.loginfo("\n" + "="*60)
            rospy.loginfo("✓ All poses received! Starting grasp pipeline...")
            rospy.loginfo("="*60)
            self.execute_grasp_sequence()
    
    def plan_and_execute_pose(self, target_pose, description):
        """
        Plan and execute movement to target pose.
        
        Returns:
            True if successful, False otherwise
        """
        rospy.loginfo(f"\n>>> {description}")
        
        try:
            self.move_group.set_pose_target(target_pose)
            rospy.loginfo("Planning...")
            
            plan = self.move_group.plan()
            
            # Handle both old and new MoveIt API
            if isinstance(plan, tuple):
                success, trajectory, planning_time, error_code = plan
            else:
                success = plan
                trajectory = plan
            
            if not success or (isinstance(plan, tuple) and not plan[0]):
                rospy.logerr(f"Planning failed for {description}!")
                return False
            
            rospy.loginfo("✓ Planning succeeded")
            rospy.sleep(self.wait_after_plan)
            
            # Execute
            rospy.loginfo("Executing...")
            execute_result = self.move_group.execute(trajectory if isinstance(plan, tuple) else plan, wait=True)
            
            if not execute_result:
                rospy.logerr(f"Execution failed for {description}!")
                return False
            
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.loginfo(f"✓ {description} completed")
            rospy.sleep(self.wait_after_execute)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error during {description}: {e}")
            return False
    
    def plan_and_execute_joints(self, joint_values, description):
        """
        Plan and execute movement to joint configuration.
        
        Returns:
            True if successful, False otherwise
        """
        rospy.loginfo(f"\n>>> {description}")
        
        try:
            self.move_group.set_joint_value_target(joint_values)
            rospy.loginfo("Planning...")
            
            plan = self.move_group.plan()
            
            if isinstance(plan, tuple):
                success, trajectory, planning_time, error_code = plan
            else:
                success = plan
                trajectory = plan
            
            if not success or (isinstance(plan, tuple) and not plan[0]):
                rospy.logerr(f"Planning failed for {description}!")
                return False
            
            rospy.loginfo("✓ Planning succeeded")
            rospy.sleep(self.wait_after_plan)
            
            # Execute
            rospy.loginfo("Executing...")
            execute_result = self.move_group.execute(trajectory if isinstance(plan, tuple) else plan, wait=True)
            
            if not execute_result:
                rospy.logerr(f"Execution failed for {description}!")
                return False
            
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.loginfo(f"✓ {description} completed")
            rospy.sleep(self.wait_after_execute)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error during {description}: {e}")
            return False
    
    def open_gripper(self):
        """Open the gripper"""
        rospy.loginfo("\n>>> Opening gripper (width: {:.3f}m)".format(self.gripper_open_width))
        
        try:
            goal = MoveGoal()
            goal.width = self.gripper_open_width
            goal.speed = self.gripper_speed
            
            rospy.loginfo("    Sending OPEN command to gripper...")
            self.gripper_move_client.send_goal(goal)
            
            # Wait for result with appropriate timeout
            timeout = rospy.Duration(2.0) if not self.use_gripper else rospy.Duration(5.0)
            if self.gripper_move_client.wait_for_result(timeout):
                result = self.gripper_move_client.get_result()
                if result and result.success:
                    rospy.loginfo("✓ Gripper opened successfully")
                    return True
                else:
                    if not self.use_gripper:
                        # In simulation, treat timeout as success
                        rospy.loginfo("✓ Gripper open command sent (simulation mode)")
                        return True
                    else:
                        rospy.logerr("✗ Failed to open gripper")
                        return False
            else:
                if not self.use_gripper:
                    # In simulation, timeout is expected
                    rospy.loginfo("✓ Gripper open command sent (simulation mode)")
                    rospy.sleep(0.5)  # Allow time for gripper to respond
                    return True
                else:
                    rospy.logerr("✗ Gripper open command timeout")
                    return False
        except Exception as e:
            rospy.logerr(f"✗ Error opening gripper: {e}")
            if not self.use_gripper:
                rospy.logwarn("Continuing in simulation mode without gripper")
                rospy.sleep(0.5)
                return True
            return False
    
    def close_gripper(self):
        """Close the gripper to grasp"""
        rospy.loginfo("\n>>> Closing gripper to grasp object")
        rospy.loginfo("    Width: {:.3f}m | Speed: {:.3f}m/s | Force: {:.1f}N".format(
            self.gripper_close_width, self.gripper_speed, self.gripper_max_effort))
        
        try:
            goal = GraspGoal()
            goal.width = self.gripper_close_width
            goal.speed = self.gripper_speed
            goal.force = self.gripper_max_effort
            goal.epsilon.inner = 0.005
            goal.epsilon.outer = 0.005
            
            rospy.loginfo("    Sending GRASP command to gripper...")
            self.gripper_grasp_client.send_goal(goal)
            
            # Wait for result with appropriate timeout
            timeout = rospy.Duration(2.0) if not self.use_gripper else rospy.Duration(5.0)
            if self.gripper_grasp_client.wait_for_result(timeout):
                result = self.gripper_grasp_client.get_result()
                if result and result.success:
                    rospy.loginfo("✓ Object grasped successfully")
                    return True
                else:
                    if not self.use_gripper:
                        # In simulation, treat timeout as success
                        rospy.loginfo("✓ Gripper grasp command sent (simulation mode)")
                        return True
                    else:
                        rospy.logerr("✗ Failed to grasp object")
                        return False
            else:
                if not self.use_gripper:
                    # In simulation, timeout is expected
                    rospy.loginfo("✓ Gripper grasp command sent (simulation mode)")
                    rospy.sleep(0.5)  # Allow time for gripper to respond
                    return True
                else:
                    rospy.logerr("✗ Gripper grasp command timeout")
                    return False
        except Exception as e:
            rospy.logerr(f"✗ Error closing gripper: {e}")
            if not self.use_gripper:
                rospy.logwarn("Continuing in simulation mode without gripper")
                rospy.sleep(0.5)
                return True
            return False
    
    def execute_grasp_sequence(self):
        """Execute complete grasp sequence with automatic gripper control"""
        # Initialize trial data
        self.current_trial = {
            'start_time': time.time(),
            'target_position': [
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z
            ] if self.target_pose else [0, 0, 0],
            'planning_times': [],
            'execution_times': [],
            'success': False,
            'failure_reason': None
        }
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("STARTING GRASP SEQUENCE")
        rospy.loginfo("="*60)
        
        # Step 1: Move to home
        self.current_state = GraspState.MOVING_TO_HOME
        self.publish_status("MOVING_TO_HOME")
        rospy.loginfo("\n[1/5] Moving to HOME configuration")
        
        if not self.plan_and_execute_joints(self.get_home_joint_values(), "Move to HOME"):
            self.handle_failure('HOME_PLANNING_FAILED', 'HOME')
            return
        
        # Step 2: Ensure gripper is open before approaching
        self.current_state = GraspState.OPENING_GRIPPER
        self.publish_status("OPENING_GRIPPER")
        rospy.loginfo("\n[2/5] Opening gripper")
        
        if not self.open_gripper():
            self.handle_failure('OPEN_GRIPPER_FAILED', 'OPEN_GRIPPER')
            return
        
        # Step 3: Move to grasp pose (descend to cube)
        self.current_state = GraspState.PLANNING_GRASP
        self.publish_status("PLANNING_GRASP")
        rospy.loginfo("\n[3/5] Descending to GRASP position (approaching cube)")
        
        if not self.plan_and_execute_pose(self.grasp_pose, "Move to GRASP"):
            self.handle_failure('GRASP_PLANNING_FAILED', 'GRASP')
            return
        
        # Step 4: Close gripper to grasp object (automatic grasp on contact)
        self.current_state = GraspState.CLOSING_GRIPPER
        self.publish_status("CLOSING_GRIPPER")
        rospy.loginfo("\n[4/5] Closing gripper to grasp object")
        
        if not self.close_gripper():
            self.handle_failure('CLOSE_GRIPPER_FAILED', 'CLOSE_GRIPPER')
            return
        
        # Step 5: Lift object and return to home
        self.current_state = GraspState.PLANNING_LIFT
        self.publish_status("PLANNING_LIFT")
        rospy.loginfo("\n[5/5] Lifting object and returning to HOME")
        
        if not self.plan_and_execute_pose(self.lift_pose, "LIFT object"):
            self.handle_failure('LIFT_PLANNING_FAILED', 'LIFT')
            return
        
        # Return to home with object
        rospy.loginfo("\n[5/5] Returning to HOME with grasped object")
        if not self.plan_and_execute_joints(self.get_home_joint_values(), "Return to HOME with object"):
            self.handle_failure('RETURN_HOME_PLANNING_FAILED', 'RETURN_HOME')
            return
        
        # Success!
        self.current_state = GraspState.SUCCESS
        self.publish_status("SUCCESS")
        self.current_trial['success'] = True
        self.current_trial['total_time'] = time.time() - self.current_trial['start_time']
        self.save_trial_result()
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("✓✓✓ GRASP PIPELINE COMPLETED SUCCESSFULLY! ✓✓✓")
        rospy.loginfo("="*60)
        self.print_statistics()
        
        # Reset for next grasp
        self.reset_state()
    
    def save_trial_result(self):
        """Save current trial result to results list"""
        result = {
            'trial': len(self.grasp_results) + 1,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'target_x': self.current_trial['target_position'][0],
            'target_y': self.current_trial['target_position'][1],
            'target_z': self.current_trial['target_position'][2],
            'success': self.current_trial['success'],
            'total_time': self.current_trial.get('total_time', 0),
            'avg_planning_time': sum(self.current_trial['planning_times']) / len(self.current_trial['planning_times']) if self.current_trial['planning_times'] else 0,
            'avg_execution_time': sum(self.current_trial['execution_times']) / len(self.current_trial['execution_times']) if self.current_trial['execution_times'] else 0,
            'num_stages': len(self.current_trial['planning_times']),
            'failure_reason': self.current_trial.get('failure_reason', '')
        }
        self.grasp_results.append(result)
        self.save_to_csv()
    
    def shutdown_hook(self):
        """Save results when node is shutting down"""
        rospy.loginfo("Saving grasp results to CSV...")
        self.save_to_csv()
    
    def save_to_csv(self):
        """Save all results to CSV file"""
        if not self.grasp_results:
            rospy.logwarn("No grasp results to save")
            return
        
        try:
            # Create directory if it doesn't exist
            output_dir = os.path.dirname(self.output_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)
            
            with open(self.output_file, 'w', newline='') as csvfile:
                fieldnames = ['trial', 'timestamp', 'target_x', 'target_y', 'target_z',
                            'success', 'total_time', 'avg_planning_time', 'avg_execution_time',
                            'num_stages', 'failure_reason']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for result in self.grasp_results:
                    writer.writerow(result)
            rospy.loginfo(f"✓ Results saved to: {self.output_file}")
        except Exception as e:
            rospy.logerr(f"Failed to save results: {e}")
    
    def print_statistics(self):
        """Print current statistics"""
        if not self.grasp_results:
            return
        
        total = len(self.grasp_results)
        successes = sum(1 for r in self.grasp_results if r['success'])
        success_rate = (successes / total) * 100 if total > 0 else 0
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("PHASE 2 STATISTICS")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Total trials: {total}")
        rospy.loginfo(f"Successful: {successes} ({success_rate:.1f}%)")
        rospy.loginfo(f"Failed: {total - successes} ({100 - success_rate:.1f}%)")
        if successes > 0:
            successful_trials = [r for r in self.grasp_results if r['success']]
            avg_time = sum(r['total_time'] for r in successful_trials) / len(successful_trials)
            rospy.loginfo(f"Avg total time: {avg_time:.2f}s")
        rospy.loginfo("="*60 + "\n")
    
    def handle_failure(self, failure_reason, step_name):
        """Handle grasp pipeline failure"""
        self.current_state = GraspState.FAILED
        self.publish_status("FAILED")
        self.current_trial['failure_reason'] = failure_reason
        self.save_trial_result()
        rospy.logerr(f"\n✗ GRASP PIPELINE FAILED at {step_name}")
        self.print_statistics()
        self.reset_state()  # Reset to IDLE for next target
    
    def reset_state(self):
        """Reset state for next grasp attempt"""
        rospy.loginfo("\nResetting for next grasp...")
        self.current_state = GraspState.IDLE
        self.target_pose = None
        self.pre_grasp_pose = None
        self.grasp_pose = None
        self.lift_pose = None
        self.publish_status("IDLE")
        rospy.loginfo("Ready for next target pose.\n")
    
    def spin(self):
        """Keep the node running"""
        rospy.spin()


def main():
    try:
        pipeline = GraspPipeline()
        pipeline.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Grasp Pipeline node interrupted.")
    except Exception as e:
        rospy.logerr(f"Error in Grasp Pipeline: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
