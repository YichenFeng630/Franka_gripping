#!/usr/bin/env python3
"""
Simplified Grasp Pipeline - Direct Joint Space Planning
========================================================

Key Improvements:
1. Removed two-stage Cartesian complexity (Stage 1/Stage 2)
2. Direct joint-space planning to grasp position (more reliable execution)
3. Micro-pause (0.5s) before gripper close for stability
4. Vertical approach only (no side motion)
5. Grasp at cube center (grasp_offset_z = 0.0)
6. Simpler pipeline: HOME -> OPEN -> PRE_GRASP -> APPROACH_GRASP -> CLOSE -> LIFT -> HOME

Why this works better:
- Joint-space planning creates trajectories that joints can actually track
- Cartesian planning can succeed but fail execution (as we saw with GOAL_TOLERANCE_VIOLATED)
- Fewer waypoints = easier to execute smoothly
- No need for ultra-fine step sizes or ultra-slow speeds
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from franka_gripper.msg import MoveGoal, MoveAction, GraspGoal, GraspAction
import actionlib
import math
from enum import Enum
import csv
import time
from datetime import datetime
import os
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import copy

# Add parent directory to path for module imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import the standalone candidate generator
from modules.candidate_generation.grasp_candidate_generator import GraspCandidateGenerator
from modules.sorting.sorting_state_machine import SortingStateMachine


class GraspState(Enum):
    IDLE = 0
    MOVING_TO_HOME = 1
    OPENING_GRIPPER = 2
    PLANNING_PRE_GRASP = 3
    APPROACHING_GRASP = 4
    MICRO_PAUSE = 5
    CLOSING_GRIPPER = 6
    LIFTING = 7
    RETREATING = 8


class SimplifiedGraspPipeline:
    """
    Simplified grasp pipeline using direct joint-space planning
    """
    
    def __init__(self):
        """Initialize the simplified grasp pipeline"""
        
        # MoveIt already initialized by test script
        # Just setup the commander
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        
        # Load parameters
        self.load_parameters()
        
        # Set planning parameters
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_goal_tolerance(self.goal_tolerance)
        self.move_group.allow_replanning(True)
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        
        # Initialize gripper clients
        self.gripper_move_client = actionlib.SimpleActionClient(
            "/franka_gripper/move", MoveAction
        )
        self.gripper_grasp_client = actionlib.SimpleActionClient(
            "/franka_gripper/grasp", GraspAction
        )
        
        # Wait for gripper action servers
        if self.use_gripper:
            rospy.loginfo("Waiting for gripper action servers...")
            self.gripper_move_client.wait_for_server(timeout=rospy.Duration(10))
            self.gripper_grasp_client.wait_for_server(timeout=rospy.Duration(10))
            rospy.loginfo("✓ Gripper servers ready")
        
        # Initialize candidate generator
        self.candidate_generator = GraspCandidateGenerator(self.move_group)
        
        # State variables
        self.current_state = GraspState.IDLE
        self.current_target = None
        self.current_grasp_pose = None
        self.current_pre_grasp_pose = None
        self.current_lift_pose = None
        
        # Counters
        self.trial_count = 0
        self.success_count = 0
        
        # Status publisher
        self.status_pub = rospy.Publisher(
            "/grasp_pipeline/status", String, queue_size=10
        )
        
        rospy.loginfo("✓ Simplified Grasp Pipeline initialized")
    
    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        
        # Grasp parameters
        self.grasp_offset_z = rospy.get_param("~grasp_offset_z", 0.0)
        self.pre_contact_height = rospy.get_param("~pre_contact_height", 0.08)
        
        # Gripper parameters
        self.use_gripper = rospy.get_param("~use_gripper", False)
        self.gripper_open_width = rospy.get_param("~gripper_open_width", 0.04)
        self.gripper_close_width = rospy.get_param("~gripper_close_width", 0.0)
        self.gripper_max_effort = rospy.get_param("~gripper_max_effort", 170.0)
        
        # Planning parameters
        self.planning_time = rospy.get_param("~planning_time", 10.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.001)
        
        # Lift parameters
        self.lift_height = rospy.get_param("~lift_height", 0.15)
        
        rospy.loginfo(f"Parameters loaded: grasp_offset_z={self.grasp_offset_z}, "
                     f"pre_contact_height={self.pre_contact_height}")
    
    def publish_status(self, status):
        """Publish pipeline status"""
        self.status_pub.publish(String(status))
    
    def execute_grasp(self, target_object_pose):
        """
        Main grasp execution pipeline.
        Simplified: HOME -> OPEN -> PRE_GRASP -> APPROACH -> MICRO_PAUSE -> CLOSE -> LIFT -> HOME
        
        Args:
            target_object_pose: PoseStamped with cube center position
        """
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("EXECUTING GRASP PIPELINE (SIMPLIFIED)")
        rospy.loginfo("="*60)
        
        self.trial_count += 1
        self.current_target = target_object_pose
        
        try:
            # Step 1: Move to home
            if not self.move_to_home():
                rospy.logerr("Failed to move to home")
                return False
            
            # Step 2: Open gripper
            if not self.open_gripper():
                rospy.logerr("Failed to open gripper")
                return False
            
            # Step 3: Generate grasp candidates
            rospy.loginfo("\nGenerating grasp candidates...")
            candidates = self.candidate_generator.generate(
                self.current_target, 
                sort_by_score=True,
                verbose=False
            )
            
            if not candidates:
                rospy.logerr("No grasp candidates generated")
                return False
            
            rospy.loginfo(f"✓ Generated {len(candidates)} candidates")
            
            # Step 4: Try each candidate
            for idx, candidate in enumerate(candidates):
                rospy.loginfo(f"\n--- Candidate {idx + 1}/{len(candidates)} ---")
                
                if self.execute_single_candidate(candidate):
                    self.success_count += 1
                    rospy.loginfo("\n" + "="*60)
                    rospy.loginfo(f"✓✓✓ GRASP SUCCESSFUL (Trial {self.trial_count}) ✓✓✓")
                    rospy.loginfo(f"    Success Rate: {self.success_count}/{self.trial_count} "
                                f"({100*self.success_count/self.trial_count:.1f}%)")
                    rospy.loginfo("="*60)
                    return True
                
                rospy.logwarn(f"Candidate {idx + 1} failed, trying next...")
            
            rospy.logerr("All candidates failed")
            return False
            
        except Exception as e:
            rospy.logerr(f"Exception in execute_grasp: {e}")
            return False
    
    def execute_single_candidate(self, candidate):
        """Execute a single grasp candidate"""
        
        try:
            # Extract poses from candidate
            self.current_pre_grasp_pose = candidate.pre_grasp_pose
            self.current_grasp_pose = candidate.grasp_pose
            
            rospy.loginfo(f"Grasp pose: z={self.current_grasp_pose.pose.position.z:.4f}")
            
            # Calculate pre-grasp pose (above the target)
            self.current_pre_grasp_pose = copy.deepcopy(self.current_grasp_pose)
            self.current_pre_grasp_pose.pose.position.z += self.pre_contact_height
            rospy.loginfo(f"Pre-grasp pose: z={self.current_pre_grasp_pose.pose.position.z:.4f}")
            
            # Step 1: Plan to pre-grasp using joint-space planning
            if not self.plan_to_pre_grasp():
                rospy.logwarn("Failed to plan to pre-grasp")
                return False
            
            # Step 2: Plan to grasp using joint-space planning
            if not self.approach_grasp():
                rospy.logwarn("Failed to approach grasp")
                return False
            
            # Step 3: Micro-pause for stability
            self.micro_pause()
            
            # Step 4: Close gripper
            if not self.close_gripper():
                rospy.logwarn("Failed to close gripper")
                return False
            
            # Step 5: Lift
            if not self.lift_object():
                rospy.logwarn("Failed to lift object")
                return False
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Exception in execute_single_candidate: {e}")
            return False
    
    def calculate_grasp_pose(self, candidate):
        """Calculate the grasp pose from candidate"""
        
        # If candidate is a GraspCandidate object with grasp_pose attribute
        if hasattr(candidate, 'grasp_pose'):
            return candidate.grasp_pose
        
        # Otherwise calculate from center dict
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "panda_link0"
        grasp_pose.header.stamp = rospy.Time.now()
        
        # Use candidate center + offset
        grasp_pose.pose.position.x = candidate['center'][0]
        grasp_pose.pose.position.y = candidate['center'][1]
        grasp_pose.pose.position.z = candidate['center'][2] + self.grasp_offset_z
        
        # Vertical approach: gripper points down
        q = quaternion_from_euler(0, math.pi, 0)
        grasp_pose.pose.orientation.x = q[0]
        grasp_pose.pose.orientation.y = q[1]
        grasp_pose.pose.orientation.z = q[2]
        grasp_pose.pose.orientation.w = q[3]
        
        return grasp_pose
    
    def move_to_home(self):
        """Move to home configuration"""
        self.current_state = GraspState.MOVING_TO_HOME
        self.publish_status("MOVING_TO_HOME")
        rospy.loginfo("\n[HOME] Moving to home position...")
        
        home_joints = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        return self.plan_and_execute_joints(home_joints, "HOME")
    
    def open_gripper(self):
        """Open gripper"""
        self.current_state = GraspState.OPENING_GRIPPER
        self.publish_status("OPENING_GRIPPER")
        rospy.loginfo("[OPEN] Opening gripper...")
        
        if not self.use_gripper:
            rospy.sleep(0.3)
            return True
        
        goal = MoveGoal()
        goal.width = self.gripper_open_width
        goal.speed = 0.1
        
        self.gripper_move_client.send_goal(goal)
        return self.gripper_move_client.wait_for_result(rospy.Duration(5.0))
    
    def plan_to_pre_grasp(self):
        """Plan to pre-grasp position using joint-space planning"""
        self.current_state = GraspState.PLANNING_PRE_GRASP
        self.publish_status("PLANNING_PRE_GRASP")
        rospy.loginfo(f"[PRE-GRASP] Planning to pre-grasp position...")
        
        return self.plan_with_retry(
            self.current_pre_grasp_pose, "PRE-GRASP", max_retries=3
        )
    
    def approach_grasp(self):
        """Approach and reach the grasp position using joint-space planning"""
        self.current_state = GraspState.APPROACHING_GRASP
        self.publish_status("APPROACHING_GRASP")
        rospy.loginfo(f"[APPROACH] Moving to grasp position...")
        
        return self.plan_with_retry(
            self.current_grasp_pose, "GRASP", max_retries=3
        )
    
    def micro_pause(self):
        """Micro-pause for stability before closing gripper"""
        self.current_state = GraspState.MICRO_PAUSE
        self.publish_status("MICRO_PAUSE")
        rospy.loginfo("[PAUSE] Micro-pause for stability (0.5s)...")
        rospy.sleep(0.5)
    
    def close_gripper(self):
        """Close gripper"""
        self.current_state = GraspState.CLOSING_GRIPPER
        self.publish_status("CLOSING_GRIPPER")
        rospy.loginfo("[CLOSE] Closing gripper...")
        
        if not self.use_gripper:
            # Simulate gripper closing
            rospy.sleep(0.5)
            return True
        
        goal = GraspGoal()
        goal.width = self.gripper_close_width
        goal.speed = 0.1
        goal.force = self.gripper_max_effort
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        
        self.gripper_grasp_client.send_goal(goal)
        success = self.gripper_grasp_client.wait_for_result(rospy.Duration(5.0))
        
        if success:
            rospy.loginfo("[CLOSE] Gripper closed, stabilizing (0.5s)...")
            rospy.sleep(0.5)
        
        return success
    
    def lift_object(self):
        """Lift the grasped object"""
        self.current_state = GraspState.LIFTING
        self.publish_status("LIFTING")
        rospy.loginfo("[LIFT] Lifting object...")
        
        # Calculate lift pose
        lift_pose = copy.deepcopy(self.current_grasp_pose)
        lift_pose.pose.position.z += self.lift_height
        
        return self.plan_with_retry(lift_pose, "LIFT", max_retries=2)
    
    def plan_and_execute_joints(self, joint_values, description):
        """Plan and execute joint target"""
        
        try:
            self.move_group.set_joint_value_target(joint_values)
            
            plan = self.move_group.plan()[1]
            if not plan.joint_trajectory.points:
                rospy.logerr(f"Planning {description} failed")
                return False
            
            rospy.loginfo(f"  Planning {description}: OK")
            success = self.move_group.execute(plan, wait=True)
            self.move_group.clear_pose_targets()
            
            if success:
                rospy.loginfo(f"  ✓ {description} executed")
                return True
            else:
                rospy.logerr(f"  Execution {description} failed")
                return False
                
        except Exception as e:
            rospy.logerr(f"Exception in plan_and_execute_joints: {e}")
            self.move_group.clear_pose_targets()
            return False
    
    def plan_with_retry(self, pose, description, max_retries=3):
        """
        Plan and execute with retry strategy.
        Retry 1: Normal planning
        Retry 2: Increased planning time
        Retry 3: Relaxed tolerance
        """
        
        original_time = self.planning_time
        original_tolerance = self.goal_tolerance
        
        for attempt in range(max_retries):
            
            try:
                if attempt == 0:
                    rospy.loginfo(f"  Attempt 1/{max_retries}: {description}")
                elif attempt == 1:
                    new_time = original_time * 2.0
                    self.move_group.set_planning_time(new_time)
                    rospy.loginfo(f"  Attempt 2/{max_retries}: {description} "
                                 f"(planning_time={new_time:.1f}s)")
                elif attempt == 2:
                    new_tolerance = original_tolerance * 2.0
                    self.move_group.set_goal_tolerance(new_tolerance)
                    rospy.loginfo(f"  Attempt 3/{max_retries}: {description} "
                                 f"(tolerance={new_tolerance})")
                
                self.move_group.set_pose_target(pose)
                
                plan = self.move_group.plan()[1]
                if not plan.joint_trajectory.points:
                    rospy.logwarn(f"    Plan failed, retry...")
                    continue
                
                rospy.loginfo(f"    Planning succeeded, executing...")
                success = self.move_group.execute(plan, wait=True)
                self.move_group.clear_pose_targets()
                
                # Restore original parameters
                self.move_group.set_planning_time(original_time)
                self.move_group.set_goal_tolerance(original_tolerance)
                
                if success:
                    rospy.loginfo(f"    ✓ {description} executed")
                    return True
                else:
                    rospy.logwarn(f"    Execution failed, retry...")
                    continue
                    
            except Exception as e:
                rospy.logwarn(f"    Exception: {e}, retry...")
                continue
        
        rospy.logerr(f"  ✗ {description} failed after {max_retries} attempts")
        self.move_group.clear_pose_targets()
        
        # Restore original parameters
        self.move_group.set_planning_time(original_time)
        self.move_group.set_goal_tolerance(original_tolerance)
        
        return False


def main():
    """Main function"""
    
    # Initialize ROS node once
    rospy.init_node('simplified_grasp_pipeline_v3', anonymous=True)
    
    try:
        pipeline = SimplifiedGraspPipeline()
        
        # Create a dummy target for testing
        target = {'center': [0.3, 0.0, 0.02], 'size': [0.025, 0.025, 0.025]}
        
        # Execute grasp
        pipeline.execute_grasp(target)
        
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
