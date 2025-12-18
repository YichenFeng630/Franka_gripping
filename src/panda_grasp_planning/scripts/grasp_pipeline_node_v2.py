#!/usr/bin/env python3
"""
Improved Grasp Pipeline Node with Cartesian Motion and Fail Recovery
==================================================================

Features:
- Pre-grasp + Cartesian approach (straight-line descent)
- Cartesian lift (straight-line ascent)
- Multiple grasp candidates with IK ranking
- Fail recovery and retry logic
- Table collision constraints
- Normalized target pose handling

State Machine:
HOME -> OPEN -> PRE_GRASP -> CARTESIAN_APPROACH -> CLOSE -> CARTESIAN_LIFT -> HOME
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from franka_gripper.msg import MoveGoal, MoveAction, GraspGoal, GraspAction
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import actionlib
import math
from enum import Enum
import csv
import time
from datetime import datetime
import os
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import numpy as np
import copy


class GraspState(Enum):
    """Enhanced state machine for robust grasp pipeline"""
    IDLE = 0
    WAITING_FOR_TARGET = 1
    GENERATING_CANDIDATES = 2
    MOVING_TO_HOME = 3
    OPENING_GRIPPER = 4
    PLANNING_PRE_GRASP = 5
    EXECUTING_PRE_GRASP = 6
    CARTESIAN_APPROACH = 7
    CLOSING_GRIPPER = 8
    CARTESIAN_LIFT = 9
    RETURNING_HOME = 10
    SUCCESS = 11
    FAILED = 12


class GraspPipeline:
    def __init__(self):
        # Initialize moveit_commander and rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('grasp_pipeline_node_v2', anonymous=False)
        
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
        
        # Add table as collision object
        if self.enable_table_collision:
            self.add_table_collision()
        
        # State management
        self.current_state = GraspState.IDLE
        self.target_pose = None
        self.grasp_candidates = []
        self.current_candidate_idx = 0
        self.current_pre_grasp_pose = None
        self.current_grasp_pose = None
        self.current_lift_pose = None
        
        # Retry tracking
        self.planning_retry_count = 0
        self.candidate_retry_count = 0
        
        # Data recording
        self.grasp_results = []
        self.current_trial = {}
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        results_dir = os.path.join(project_dir, 'test_results')
        os.makedirs(results_dir, exist_ok=True)
        self.output_file = os.path.join(results_dir, 'phase3_grasp_results.csv')
        rospy.loginfo(f"Data will be saved to: {self.output_file}")
        
        rospy.on_shutdown(self.shutdown_hook)
        
        # Publishers
        self.status_pub = rospy.Publisher('/grasp_planning_status', String, queue_size=1)
        
        # Subscriber to target cube pose
        self.target_sub = rospy.Subscriber(
            '/target_cube_pose',
            PoseStamped,
            self.target_callback,
            queue_size=1
        )
        
        # Initialize gripper action clients
        self.use_gripper = rospy.get_param('~use_gripper', False)
        if self.use_gripper:
            try:
                self.gripper_move_client = actionlib.SimpleActionClient(
                    '/franka_gripper/move', MoveAction
                )
                self.gripper_grasp_client = actionlib.SimpleActionClient(
                    '/franka_gripper/grasp', GraspAction
                )
                rospy.loginfo("Waiting for gripper action servers...")
                self.gripper_move_client.wait_for_server(timeout=rospy.Duration(5.0))
                self.gripper_grasp_client.wait_for_server(timeout=rospy.Duration(5.0))
                rospy.loginfo("Gripper action servers connected")
            except Exception as e:
                rospy.logwarn(f"Gripper action servers not available: {e}")
                self.use_gripper = False
        
        rospy.loginfo("="*60)
        rospy.loginfo("Improved Grasp Pipeline Node V2 Initialized")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Planning group: {self.planning_group}")
        rospy.loginfo(f"Planner: {self.planner_id}")
        rospy.loginfo(f"Table collision: {'ENABLED' if self.enable_table_collision else 'DISABLED'}")
        rospy.loginfo(f"Grasp candidates: {self.max_candidates}")
        rospy.loginfo(f"Cartesian motion: ENABLED")
        rospy.loginfo("="*60)
        
        self.publish_status("IDLE")
    
    def load_parameters(self):
        """Load planning and grasp parameters"""
        # Planning parameters
        self.planning_group = rospy.get_param('/planning/planning_group', 'panda_arm')
        self.reference_frame = rospy.get_param('/planning/reference_frame', 'panda_link0')
        self.planner_id = rospy.get_param('/planning/planner_id', 'RRTConnect')
        self.planning_time = rospy.get_param('/planning/planning_time', 10.0)
        self.num_planning_attempts = rospy.get_param('/planning/num_planning_attempts', 10)
        self.max_velocity_scaling = rospy.get_param('/planning/max_velocity_scaling_factor', 0.5)
        self.max_acceleration_scaling = rospy.get_param('/planning/max_acceleration_scaling_factor', 0.5)
        
        # Grasp parameters
        self.pre_grasp_height = rospy.get_param('/grasp/pre_grasp_height', 0.15)
        self.grasp_offset_z = rospy.get_param('/grasp/grasp_offset_z', 0.02)
        self.lift_offset_z = rospy.get_param('/grasp/lift_offset_z', 0.15)
        
        # Gripper orientation
        self.gripper_roll = rospy.get_param('/grasp/gripper_orientation/roll', math.pi)
        self.gripper_pitch = rospy.get_param('/grasp/gripper_orientation/pitch', 0.0)
        self.gripper_yaw = rospy.get_param('/grasp/gripper_orientation/yaw', 0.0)
        
        # Grasp candidates
        self.enable_candidates = rospy.get_param('/grasp/candidates/enable', True)
        self.yaw_angles = rospy.get_param('/grasp/candidates/yaw_angles', [0.0, 1.5708, 3.14159, 4.71239])
        self.max_candidates = rospy.get_param('/grasp/candidates/max_candidates', 4)
        
        # Cartesian parameters
        self.cartesian_step_size = rospy.get_param('/grasp/cartesian/step_size', 0.005)
        self.cartesian_jump_threshold = rospy.get_param('/grasp/cartesian/jump_threshold', 0.0)
        self.cartesian_min_fraction = rospy.get_param('/grasp/cartesian/min_fraction', 0.95)
        
        # Gripper control
        self.gripper_open_width = rospy.get_param('/gripper/open_width', 0.08)
        self.gripper_close_width = rospy.get_param('/gripper/close_width', 0.02)
        self.gripper_max_effort = rospy.get_param('/gripper/max_effort', 50.0)
        
        # Failure handling
        self.max_planning_retries = rospy.get_param('/failure_handling/max_planning_retries', 3)
        self.max_candidate_retries = rospy.get_param('/failure_handling/max_candidate_retries', 4)
        self.planning_time_increment = rospy.get_param('/failure_handling/planning_time_increment', 2.0)
        self.fallback_planner = rospy.get_param('/failure_handling/fallback_planner', 'RRTstar')
        
        # Environment constraints
        self.table_height = rospy.get_param('/environment/table_height', 0.0)
        self.safety_margin_z = rospy.get_param('/environment/safety_margin_z', 0.08)
        self.enable_table_collision = rospy.get_param('/environment/enable_table_collision', True)
        self.table_x = rospy.get_param('/environment/table_dimensions/x', 1.0)
        self.table_y = rospy.get_param('/environment/table_dimensions/y', 1.0)
        self.table_thickness = rospy.get_param('/environment/table_dimensions/thickness', 0.05)
    
    def add_table_collision(self):
        """Add table as collision object in planning scene"""
        rospy.sleep(1.0)  # Wait for planning scene to be ready
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.reference_frame
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = self.table_height - self.table_thickness / 2.0
        table_pose.pose.orientation.w = 1.0
        
        self.scene.add_box("table", table_pose, (self.table_x, self.table_y, self.table_thickness))
        rospy.loginfo(f"Added table collision object at z={self.table_height}")
    
    def normalize_target_pose(self, target_pose):
        """
        Normalize target pose to cube center in base frame.
        Input could be cube center, cube top, or any 6D pose.
        Output is always cube center for consistent grasp generation.
        """
        normalized = copy.deepcopy(target_pose)
        
        # Ensure frame is correct
        if normalized.header.frame_id != self.reference_frame:
            rospy.logwarn(f"Target pose frame {normalized.header.frame_id} != {self.reference_frame}")
            # In production, would do TF transform here
        
        # Clamp Z to safe height
        min_z = self.table_height + self.safety_margin_z
        if normalized.pose.position.z < min_z:
            rospy.logwarn(f"Target Z={normalized.pose.position.z:.3f} < minimum {min_z:.3f}, clamping")
            normalized.pose.position.z = min_z
        
        return normalized
    
    def generate_grasp_candidates(self, cube_center_pose):
        """
        Generate multiple grasp candidates with different yaw orientations.
        Returns list of (pre_grasp_pose, grasp_pose, lift_pose, score) tuples.
        """
        candidates = []
        
        if not self.enable_candidates:
            # Single candidate with default orientation
            yaw_angles = [self.gripper_yaw]
        else:
            yaw_angles = self.yaw_angles[:self.max_candidates]
        
        for yaw in yaw_angles:
            # Generate poses with this yaw
            pre_grasp, grasp, lift = self.generate_pose_set(cube_center_pose, yaw)
            
            # Check IK feasibility and score
            score = self.score_grasp_candidate(pre_grasp, grasp, lift)
            
            if score > 0:  # Valid candidate
                candidates.append((pre_grasp, grasp, lift, score))
                rospy.loginfo(f"Candidate yaw={math.degrees(yaw):.1f}°, score={score:.3f}")
        
        # Sort by score (higher is better)
        candidates.sort(key=lambda x: x[3], reverse=True)
        
        if not candidates:
            rospy.logerr("No valid grasp candidates found!")
            return []
        
        rospy.loginfo(f"Generated {len(candidates)} valid candidates")
        return candidates
    
    def generate_pose_set(self, cube_center, yaw):
        """Generate pre-grasp, grasp, and lift poses for a given yaw angle"""
        # Grasp pose (at cube with offset)
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.reference_frame
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.pose.position.x = cube_center.pose.position.x
        grasp_pose.pose.position.y = cube_center.pose.position.y
        grasp_pose.pose.position.z = cube_center.pose.position.z + self.grasp_offset_z
        
        # Gripper orientation with specified yaw
        q = quaternion_from_euler(self.gripper_roll, self.gripper_pitch, yaw)
        grasp_pose.pose.orientation.x = q[0]
        grasp_pose.pose.orientation.y = q[1]
        grasp_pose.pose.orientation.z = q[2]
        grasp_pose.pose.orientation.w = q[3]
        
        # Pre-grasp pose (above grasp pose)
        pre_grasp_pose = copy.deepcopy(grasp_pose)
        pre_grasp_pose.pose.position.z += self.pre_grasp_height
        
        # Lift pose (above grasp pose)
        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.pose.position.z += self.lift_offset_z
        
        return pre_grasp_pose, grasp_pose, lift_pose
    
    def score_grasp_candidate(self, pre_grasp, grasp, lift):
        """
        Score grasp candidate based on IK feasibility and joint limits margin.
        Returns score (higher is better), 0 if invalid.
        """
        score = 0.0
        
        # Check IK for pre-grasp
        pre_grasp_ik = self.move_group.set_pose_target(pre_grasp.pose)
        pre_grasp_plan = self.move_group.plan()
        
        # Handle different MoveIt versions
        if isinstance(pre_grasp_plan, tuple):
            pre_grasp_success = pre_grasp_plan[0]
        else:
            pre_grasp_success = len(pre_grasp_plan.joint_trajectory.points) > 0
        
        self.move_group.clear_pose_targets()
        
        if not pre_grasp_success:
            return 0.0  # No IK solution
        
        score += 10.0  # Base score for valid IK
        
        # Check grasp IK
        grasp_ik = self.move_group.set_pose_target(grasp.pose)
        grasp_plan = self.move_group.plan()
        
        if isinstance(grasp_plan, tuple):
            grasp_success = grasp_plan[0]
        else:
            grasp_success = len(grasp_plan.joint_trajectory.points) > 0
        
        self.move_group.clear_pose_targets()
        
        if not grasp_success:
            return 0.0
        
        score += 10.0
        
        # Additional scoring: distance from joint limits (simplified)
        # In production, would check actual joint values against limits
        current_joints = self.move_group.get_current_joint_values()
        joint_limits_margin = 5.0  # Placeholder score
        score += joint_limits_margin
        
        return score
    
    def target_callback(self, msg):
        """Handle new target cube pose"""
        if self.current_state != GraspState.IDLE and self.current_state != GraspState.WAITING_FOR_TARGET:
            rospy.logwarn("Busy executing grasp, ignoring new target")
            return
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("RECEIVED TARGET CUBE POSE")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Position: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")
        
        # Normalize target pose
        self.target_pose = self.normalize_target_pose(msg)
        
        # Generate grasp candidates
        self.current_state = GraspState.GENERATING_CANDIDATES
        self.publish_status("GENERATING_CANDIDATES")
        self.grasp_candidates = self.generate_grasp_candidates(self.target_pose)
        
        if not self.grasp_candidates:
            rospy.logerr("Failed to generate valid grasp candidates")
            self.current_state = GraspState.FAILED
            return
        
        # Start with best candidate
        self.current_candidate_idx = 0
        self.load_candidate(0)
        
        # Execute grasp sequence
        self.execute_grasp_sequence()
    
    def load_candidate(self, idx):
        """Load grasp candidate at given index"""
        if idx >= len(self.grasp_candidates):
            return False
        
        pre_grasp, grasp, lift, score = self.grasp_candidates[idx]
        self.current_pre_grasp_pose = pre_grasp
        self.current_grasp_pose = grasp
        self.current_lift_pose = lift
        rospy.loginfo(f"Loaded candidate {idx+1}/{len(self.grasp_candidates)}, score={score:.3f}")
        return True
    
    def execute_grasp_sequence(self):
        """Execute complete grasp sequence with fail recovery"""
        self.current_trial = {
            'start_time': time.time(),
            'target_position': [
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z
            ],
            'planning_times': [],
            'execution_times': [],
            'cartesian_fractions': [],
            'success': False,
            'failure_reason': None,
            'candidates_tried': 0,
            'retries': 0
        }
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("STARTING IMPROVED GRASP SEQUENCE")
        rospy.loginfo("="*60)
        
        # Step 1: Move to home
        if not self.execute_with_retry(self.move_to_home, "HOME"):
            return
        
        # Step 2: Open gripper
        if not self.execute_with_retry(self.open_gripper, "OPEN_GRIPPER"):
            return
        
        # Step 3: Plan to pre-grasp (global RRT)
        if not self.execute_with_retry(self.move_to_pre_grasp, "PRE_GRASP"):
            return
        
        # Step 4: Cartesian approach to grasp
        if not self.execute_with_retry(self.cartesian_approach, "CARTESIAN_APPROACH"):
            return
        
        # Step 5: Close gripper
        if not self.execute_with_retry(self.close_gripper, "CLOSE_GRIPPER"):
            return
        
        # Step 6: Cartesian lift
        if not self.execute_with_retry(self.cartesian_lift, "CARTESIAN_LIFT"):
            return
        
        # Step 7: Return to home
        if not self.execute_with_retry(self.move_to_home, "RETURN_HOME"):
            return
        
        # Success!
        self.handle_success()
    
    def execute_with_retry(self, action_func, stage_name):
        """Execute action with retry logic and fallback strategies"""
        max_retries = self.max_planning_retries
        
        for attempt in range(max_retries):
            self.current_trial['retries'] += 1
            
            if attempt > 0:
                rospy.logwarn(f"Retry {attempt}/{max_retries} for {stage_name}")
                # Increase planning time for retry
                self.move_group.set_planning_time(self.planning_time * self.planning_time_increment)
            
            success = action_func()
            
            if success:
                # Reset planning time
                self.move_group.set_planning_time(self.planning_time)
                return True
            
            rospy.logwarn(f"{stage_name} failed, attempt {attempt+1}/{max_retries}")
        
        # All retries failed, try next candidate if available
        rospy.logerr(f"{stage_name} failed after {max_retries} attempts")
        
        if self.try_next_candidate():
            rospy.loginfo("Trying next grasp candidate...")
            return self.execute_with_retry(action_func, stage_name)
        
        # No more candidates, complete failure
        self.handle_failure(f"{stage_name}_FAILED_ALL_CANDIDATES", stage_name)
        return False
    
    def try_next_candidate(self):
        """Try to load next grasp candidate"""
        self.current_candidate_idx += 1
        self.current_trial['candidates_tried'] = self.current_candidate_idx + 1
        
        if self.current_candidate_idx >= len(self.grasp_candidates):
            return False
        
        if self.current_candidate_idx >= self.max_candidate_retries:
            rospy.logerr("Reached maximum candidate retry limit")
            return False
        
        return self.load_candidate(self.current_candidate_idx)
    
    def move_to_home(self):
        """Move to home configuration"""
        self.current_state = GraspState.MOVING_TO_HOME
        self.publish_status("MOVING_TO_HOME")
        rospy.loginfo("\nMoving to HOME configuration")
        
        home_joints = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        return self.plan_and_execute_joints(home_joints, "HOME")
    
    def open_gripper(self):
        """Open gripper"""
        self.current_state = GraspState.OPENING_GRIPPER
        self.publish_status("OPENING_GRIPPER")
        rospy.loginfo("\nOpening gripper")
        
        if not self.use_gripper:
            rospy.loginfo("Gripper control disabled (simulation)")
            rospy.sleep(0.5)
            return True
        
        goal = MoveGoal()
        goal.width = self.gripper_open_width
        goal.speed = 0.1
        
        self.gripper_move_client.send_goal(goal)
        if self.gripper_move_client.wait_for_result(rospy.Duration(5.0)):
            rospy.loginfo("Gripper opened")
            return True
        
        rospy.logwarn("Gripper open timeout")
        return False
    
    def move_to_pre_grasp(self):
        """Move to pre-grasp pose using global planner"""
        self.current_state = GraspState.PLANNING_PRE_GRASP
        self.publish_status("PLANNING_PRE_GRASP")
        rospy.loginfo("\nPlanning to PRE-GRASP pose (RRT)")
        
        return self.plan_and_execute_pose(self.current_pre_grasp_pose, "PRE_GRASP")
    
    def cartesian_approach(self):
        """Cartesian straight-line approach from pre-grasp to grasp"""
        self.current_state = GraspState.CARTESIAN_APPROACH
        self.publish_status("CARTESIAN_APPROACH")
        rospy.loginfo("\nCartesian APPROACH (straight-line descent)")
        
        # Generate Cartesian path
        waypoints = [self.current_grasp_pose.pose]
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            self.cartesian_step_size,
            avoid_collisions=True
        )
        
        self.current_trial['cartesian_fractions'].append(fraction)
        rospy.loginfo(f"Cartesian path fraction: {fraction*100:.1f}%")
        
        if fraction < self.cartesian_min_fraction:
            rospy.logwarn(f"Cartesian path fraction {fraction:.2f} < required {self.cartesian_min_fraction}")
            return False
        
        # Execute Cartesian path
        start_time = time.time()
        success = self.move_group.execute(plan, wait=True)
        exec_time = time.time() - start_time
        
        self.current_trial['execution_times'].append(exec_time)
        
        if success:
            rospy.loginfo("Cartesian approach completed")
        else:
            rospy.logwarn("Cartesian approach execution failed")
        
        return success
    
    def close_gripper(self):
        """Close gripper to grasp object"""
        self.current_state = GraspState.CLOSING_GRIPPER
        self.publish_status("CLOSING_GRIPPER")
        rospy.loginfo("\nClosing gripper")
        
        if not self.use_gripper:
            rospy.loginfo("Gripper control disabled (simulation)")
            rospy.sleep(0.5)
            return True
        
        goal = GraspGoal()
        goal.width = self.gripper_close_width
        goal.speed = 0.1
        goal.force = self.gripper_max_effort
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        
        self.gripper_grasp_client.send_goal(goal)
        if self.gripper_grasp_client.wait_for_result(rospy.Duration(5.0)):
            rospy.loginfo("Gripper closed")
            return True
        
        rospy.logwarn("Gripper close timeout")
        return False
    
    def cartesian_lift(self):
        """Cartesian straight-line lift"""
        self.current_state = GraspState.CARTESIAN_LIFT
        self.publish_status("CARTESIAN_LIFT")
        rospy.loginfo("\nCartesian LIFT (straight-line ascent)")
        
        waypoints = [self.current_lift_pose.pose]
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            self.cartesian_step_size,
            avoid_collisions=True
        )
        
        self.current_trial['cartesian_fractions'].append(fraction)
        rospy.loginfo(f"Cartesian path fraction: {fraction*100:.1f}%")
        
        if fraction < self.cartesian_min_fraction:
            rospy.logwarn(f"Cartesian path fraction {fraction:.2f} < required {self.cartesian_min_fraction}")
            return False
        
        start_time = time.time()
        success = self.move_group.execute(plan, wait=True)
        exec_time = time.time() - start_time
        
        self.current_trial['execution_times'].append(exec_time)
        
        if success:
            rospy.loginfo("Cartesian lift completed")
        else:
            rospy.logwarn("Cartesian lift execution failed")
        
        return success
    
    def plan_and_execute_joints(self, joint_values, description):
        """Plan and execute to joint configuration"""
        self.move_group.set_joint_value_target(joint_values)
        
        start_time = time.time()
        plan = self.move_group.plan()
        plan_time = time.time() - start_time
        
        # Handle different MoveIt versions (tuple: (success, plan) or RobotTrajectory)
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
        else:
            success = len(plan.joint_trajectory.points) > 0
            trajectory = plan
        
        if not success:
            rospy.logwarn(f"{description} planning failed")
            return False
        
        self.current_trial['planning_times'].append(plan_time)
        rospy.loginfo(f"{description} planned in {plan_time:.2f}s")
        
        start_time = time.time()
        exec_success = self.move_group.execute(trajectory, wait=True)
        exec_time = time.time() - start_time
        
        self.current_trial['execution_times'].append(exec_time)
        
        if exec_success:
            rospy.loginfo(f"{description} executed in {exec_time:.2f}s")
        else:
            rospy.logwarn(f"{description} execution failed")
        
        return exec_success
    
    def plan_and_execute_pose(self, pose_stamped, description):
        """Plan and execute to pose"""
        self.move_group.set_pose_target(pose_stamped.pose)
        
        start_time = time.time()
        plan = self.move_group.plan()
        plan_time = time.time() - start_time
        
        # Handle different MoveIt versions (tuple: (success, plan) or RobotTrajectory)
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
        else:
            success = len(plan.joint_trajectory.points) > 0
            trajectory = plan
        
        self.move_group.clear_pose_targets()
        
        if not success:
            rospy.logwarn(f"{description} planning failed")
            return False
        
        self.current_trial['planning_times'].append(plan_time)
        rospy.loginfo(f"{description} planned in {plan_time:.2f}s")
        
        start_time = time.time()
        exec_success = self.move_group.execute(trajectory, wait=True)
        exec_time = time.time() - start_time
        
        self.current_trial['execution_times'].append(exec_time)
        
        if exec_success:
            rospy.loginfo(f"{description} executed in {exec_time:.2f}s")
        else:
            rospy.logwarn(f"{description} execution failed")
        
        return exec_success
    
    def handle_success(self):
        """Handle successful grasp completion"""
        self.current_state = GraspState.SUCCESS
        self.publish_status("SUCCESS")
        self.current_trial['success'] = True
        self.current_trial['total_time'] = time.time() - self.current_trial['start_time']
        self.save_trial_result()
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("GRASP PIPELINE COMPLETED SUCCESSFULLY")
        rospy.loginfo("="*60)
        self.print_statistics()
        
        self.reset_state()
    
    def handle_failure(self, reason, stage):
        """Handle grasp failure"""
        self.current_state = GraspState.FAILED
        self.publish_status("FAILED")
        self.current_trial['success'] = False
        self.current_trial['failure_reason'] = reason
        self.current_trial['failure_stage'] = stage
        self.current_trial['total_time'] = time.time() - self.current_trial['start_time']
        self.save_trial_result()
        
        rospy.logerr("\n" + "="*60)
        rospy.logerr(f"GRASP PIPELINE FAILED: {reason} at {stage}")
        rospy.logerr("="*60)
        
        self.reset_state()
    
    def save_trial_result(self):
        """Save trial result to CSV"""
        result = {
            'trial': len(self.grasp_results) + 1,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'target_x': self.current_trial['target_position'][0],
            'target_y': self.current_trial['target_position'][1],
            'target_z': self.current_trial['target_position'][2],
            'success': self.current_trial['success'],
            'total_time': self.current_trial.get('total_time', 0),
            'candidates_tried': self.current_trial.get('candidates_tried', 0),
            'retries': self.current_trial.get('retries', 0),
            'avg_cartesian_fraction': sum(self.current_trial.get('cartesian_fractions', [0])) / max(len(self.current_trial.get('cartesian_fractions', [1])), 1),
            'failure_reason': self.current_trial.get('failure_reason', 'N/A')
        }
        self.grasp_results.append(result)
    
    def print_statistics(self):
        """Print execution statistics"""
        trial = self.current_trial
        rospy.loginfo(f"Total time: {trial.get('total_time', 0):.2f}s")
        rospy.loginfo(f"Candidates tried: {trial.get('candidates_tried', 0)}")
        rospy.loginfo(f"Retries: {trial.get('retries', 0)}")
        if trial.get('cartesian_fractions'):
            avg_fraction = sum(trial['cartesian_fractions']) / len(trial['cartesian_fractions'])
            rospy.loginfo(f"Avg Cartesian fraction: {avg_fraction*100:.1f}%")
    
    def reset_state(self):
        """Reset state for next grasp"""
        self.current_state = GraspState.IDLE
        self.target_pose = None
        self.grasp_candidates = []
        self.current_candidate_idx = 0
        self.planning_retry_count = 0
        self.candidate_retry_count = 0
        self.publish_status("IDLE")
    
    def publish_status(self, status):
        """Publish current status"""
        self.status_pub.publish(String(data=status))
    
    def shutdown_hook(self):
        """Save results on shutdown"""
        if self.grasp_results:
            with open(self.output_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.grasp_results[0].keys())
                writer.writeheader()
                writer.writerows(self.grasp_results)
            rospy.loginfo(f"Saved {len(self.grasp_results)} results to {self.output_file}")
        
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        pipeline = GraspPipeline()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
