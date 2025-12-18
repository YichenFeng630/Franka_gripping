#!/usr/bin/env python3
"""
Grasp Pipeline Node V3 with Enhanced Robustness
================================================

Improvements over V2:
1. Target pose normalization (internal clamping, no external constraints)
2. Relative pre_grasp_offset_z parameter
3. Multi-directional approach candidates (yaw + xy directions)
4. Hierarchical Cartesian path failure handling
5. Layered planning retry strategy (planning_time -> tolerance -> planner)
6. RETREAT stage for safer home return
7. **FIXED: Robust candidate retry mechanism**
   - When PRE_GRASP planning fails, automatically try next candidate (recursive)
   - When CARTESIAN_LIFT fails, try next candidate instead of failing immediately
   - When candidate generation fails, properly reset state to IDLE for next target
   - Prevents pipeline from getting stuck when individual candidates fail

State Machine:
HOME -> OPEN -> PRE_GRASP -> CARTESIAN_APPROACH -> CLOSE -> 
CARTESIAN_LIFT -> RETREAT -> HOME
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
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import numpy as np
import copy


class GraspState(Enum):
    """Enhanced state machine with RETREAT stage"""
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
    RETREAT = 10
    RETURNING_HOME = 11
    SUCCESS = 12
    FAILED = 13


class GraspPipeline:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('grasp_pipeline_node_v3', anonymous=False)
        
        self.load_parameters()
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)
        
        self.move_group.set_pose_reference_frame(self.reference_frame)
        self.move_group.set_planner_id(self.planner_id)
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_num_planning_attempts(self.num_planning_attempts)
        self.move_group.set_max_velocity_scaling_factor(self.max_velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(self.max_acceleration_scaling)
        
        # Store original planning params for retry strategy
        self.original_planning_time = self.planning_time
        # Handle tolerance which might be returned as tuple
        tol_raw = self.move_group.get_goal_tolerance()
        if isinstance(tol_raw, (list, tuple)):
            try:
                self.original_goal_tolerance = float(max(tol_raw))
            except Exception:
                self.original_goal_tolerance = 0.01
        else:
            self.original_goal_tolerance = float(tol_raw)
        self.available_planners = ['RRTConnect', 'RRTstar']  # For fallback
        
        if self.enable_table_collision:
            self.add_table_collision()
        
        self.current_state = GraspState.IDLE
        self.target_pose = None
        self.grasp_candidates = []
        self.current_candidate_idx = 0
        self.current_pre_grasp_pose = None
        self.current_grasp_pose = None
        self.current_lift_pose = None
        self.current_retreat_pose = None
        
        self.grasp_results = []
        self.current_trial = {}
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        results_dir = os.path.join(project_dir, 'test_results')
        os.makedirs(results_dir, exist_ok=True)
        self.output_file = os.path.join(results_dir, 'phase4_grasp_results.csv')
        
        rospy.on_shutdown(self.shutdown_hook)
        
        self.status_pub = rospy.Publisher('/grasp_planning_status', String, queue_size=1)
        
        self.target_sub = rospy.Subscriber(
            '/target_cube_pose', PoseStamped, self.target_callback, queue_size=1
        )
        
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
            except Exception as e:
                rospy.logwarn(f"Gripper not available: {e}")
                self.use_gripper = False
        
        rospy.loginfo("="*60)
        rospy.loginfo("Grasp Pipeline V3 (Enhanced Robustness)")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Candidates per yaw: {len(self.approach_directions)}")
        rospy.loginfo(f"Max total candidates: {len(self.yaw_angles) * len(self.approach_directions)}")
        rospy.loginfo("="*60)
        
        self.publish_status("IDLE")
    
    def load_parameters(self):
        """Load all configuration parameters"""
        # Planning
        self.planning_group = rospy.get_param('/planning/planning_group', 'panda_arm')
        self.reference_frame = rospy.get_param('/planning/reference_frame', 'panda_link0')
        self.planner_id = rospy.get_param('/planning/planner_id', 'RRTConnect')
        self.planning_time = rospy.get_param('/planning/planning_time', 10.0)
        self.num_planning_attempts = rospy.get_param('/planning/num_planning_attempts', 10)
        self.max_velocity_scaling = rospy.get_param('/planning/max_velocity_scaling_factor', 0.5)
        self.max_acceleration_scaling = rospy.get_param('/planning/max_acceleration_scaling_factor', 0.5)
        
        # Grasp heights (all relative)
        self.pre_grasp_offset_z = rospy.get_param('/grasp/pre_grasp_offset_z', 0.15)
        self.grasp_offset_z = rospy.get_param('/grasp/grasp_offset_z', 0.02)
        self.lift_offset_z = rospy.get_param('/grasp/lift_offset_z', 0.15)
        self.retreat_offset_z = rospy.get_param('/grasp/retreat_offset_z', 0.20)
        
        # Gripper orientation
        self.gripper_roll = rospy.get_param('/grasp/gripper_orientation/roll', math.pi)
        self.gripper_pitch = rospy.get_param('/grasp/gripper_orientation/pitch', 0.0)
        self.gripper_yaw = rospy.get_param('/grasp/gripper_orientation/yaw', 0.0)
        
        # Candidates: yaw + approach directions
        self.yaw_angles = rospy.get_param('/grasp/candidates/yaw_angles', [0.0, 1.5708, 3.14159, 4.71239])
        self.approach_directions = rospy.get_param(
            '/grasp/candidates/approach_directions',
            [[0, 0], [0.1, 0], [-0.1, 0], [0, 0.1]]  # Top + 4 side approaches
        )
        self.max_candidates = rospy.get_param('/grasp/candidates/max_candidates', 8)
        
        # Cartesian
        self.cartesian_step_size = rospy.get_param('/grasp/cartesian/step_size', 0.005)
        self.cartesian_min_fraction = rospy.get_param('/grasp/cartesian/min_fraction', 0.95)
        
        # Gripper
        self.gripper_open_width = rospy.get_param('/gripper/open_width', 0.08)
        self.gripper_close_width = rospy.get_param('/gripper/close_width', 0.02)
        self.gripper_max_effort = rospy.get_param('/gripper/max_effort', 50.0)
        
        # Environment
        self.table_height = rospy.get_param('/environment/table_height', 0.0)
        self.safety_margin_z = rospy.get_param('/environment/safety_margin_z', 0.08)
        self.cube_half_size = rospy.get_param('/environment/cube_half_size', 0.015)
        self.enable_table_collision = rospy.get_param('/environment/enable_table_collision', True)
        
        # Failure handling
        self.max_planning_retries = rospy.get_param('/failure_handling/max_planning_retries', 3)
        self.max_candidate_retries = rospy.get_param('/failure_handling/max_candidate_retries', 8)
        self.cartesian_step_downsample_factor = rospy.get_param('/failure_handling/step_downsample', 2)
        self.cartesian_distance_scale = rospy.get_param('/failure_handling/distance_scale', 0.8)
    
    def add_table_collision(self):
        """Add table as collision object"""
        rospy.sleep(1.0)
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.reference_frame
        table_pose.pose.position.z = self.table_height - 0.05
        table_pose.pose.orientation.w = 1.0
        
        self.scene.add_box("table", table_pose, (1.0, 1.0, 0.1))
        rospy.loginfo(f"Added table collision at z={self.table_height}")
    
    def normalize_target_pose(self, target_pose):
        """
        Normalize target pose to cube center in base frame.
        
        Interface contract: /target_cube_pose is always cube center in panda_link0
        Internal normalization ensures:
        - z_safe = max(target_z, table_height + cube_half_size + margin)
        - pre_grasp_z = max(z_safe + pre_grasp_offset_z, table_height + safety_margin_z)
        
        This way, any upstream z value is safe.
        """
        normalized = copy.deepcopy(target_pose)
        
        # Ensure in correct frame
        if normalized.header.frame_id != self.reference_frame:
            rospy.logwarn(f"Target frame {normalized.header.frame_id} != {self.reference_frame}")
        
        # Internal Z safety clamp: grasp must be above table + cube size + small margin
        min_grasp_z = self.table_height + self.cube_half_size + 0.01
        if normalized.pose.position.z < min_grasp_z:
            rospy.loginfo(f"Clamping z: {normalized.pose.position.z:.3f} -> {min_grasp_z:.3f}")
            normalized.pose.position.z = min_grasp_z
        
        rospy.loginfo(f"Target normalized: ({normalized.pose.position.x:.3f}, "
                     f"{normalized.pose.position.y:.3f}, {normalized.pose.position.z:.3f})")
        return normalized
    
    def generate_grasp_candidates(self, cube_center_pose):
        """
        Generate grasp candidates with different yaw + approach directions.
        
        For each (yaw, approach_direction) pair:
        - grasp_pose.z = cube_center.z + grasp_offset_z
        - pre_grasp.z = grasp_pose.z + pre_grasp_offset_z
        - pre_grasp (x,y) offset by approach_direction
        """
        candidates = []
        
        for yaw in self.yaw_angles:
            for approach_dir in self.approach_directions:
                pre_grasp, grasp, lift, retreat = self.generate_pose_set(
                    cube_center_pose, yaw, approach_dir
                )
                
                score = self.score_grasp_candidate(pre_grasp, grasp, lift)
                
                if score > 0:
                    candidates.append((pre_grasp, grasp, lift, retreat, score))
                    rospy.loginfo(
                        f"Candidate: yaw={math.degrees(yaw):.0f}° "
                        f"approach=({approach_dir[0]:+.2f}, {approach_dir[1]:+.2f}), "
                        f"score={score:.1f}"
                    )
        
        candidates.sort(key=lambda x: x[4], reverse=True)
        candidates = candidates[:self.max_candidates]
        
        rospy.loginfo(f"Generated {len(candidates)}/{len(self.yaw_angles)*len(self.approach_directions)} valid candidates")
        return candidates
    
    def generate_pose_set(self, cube_center, yaw, approach_dir):
        """Generate (pre_grasp, grasp, lift, retreat) for given yaw + approach direction"""
        # Grasp pose at cube center + offset
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.reference_frame
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.pose.position.x = cube_center.pose.position.x
        grasp_pose.pose.position.y = cube_center.pose.position.y
        grasp_pose.pose.position.z = cube_center.pose.position.z + self.grasp_offset_z
        
        # Orientation
        q = quaternion_from_euler(self.gripper_roll, self.gripper_pitch, yaw)
        grasp_pose.pose.orientation.x = q[0]
        grasp_pose.pose.orientation.y = q[1]
        grasp_pose.pose.orientation.z = q[2]
        grasp_pose.pose.orientation.w = q[3]
        
        # Pre-grasp: above grasp, offset by approach direction
        pre_grasp_pose = copy.deepcopy(grasp_pose)
        pre_grasp_pose.pose.position.z += self.pre_grasp_offset_z
        pre_grasp_pose.pose.position.x += approach_dir[0]
        pre_grasp_pose.pose.position.y += approach_dir[1]
        
        # Lift pose
        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.pose.position.z += self.lift_offset_z
        
        # Retreat pose: safe height before returning home
        retreat_pose = copy.deepcopy(grasp_pose)
        retreat_pose.pose.position.z += self.retreat_offset_z
        
        return pre_grasp_pose, grasp_pose, lift_pose, retreat_pose
    
    def score_grasp_candidate(self, pre_grasp, grasp, lift):
        """Score candidate based on IK feasibility"""
        score = 0.0
        
        try:
            self.move_group.set_pose_target(pre_grasp.pose)
            plan_result = self.move_group.plan()
            
            if isinstance(plan_result, tuple):
                pre_ok = plan_result[0]
            else:
                pre_ok = len(plan_result.joint_trajectory.points) > 0
            
            self.move_group.clear_pose_targets()
            
            if not pre_ok:
                return 0.0
            
            score += 10.0
            
            self.move_group.set_pose_target(grasp.pose)
            plan_result = self.move_group.plan()
            
            if isinstance(plan_result, tuple):
                grasp_ok = plan_result[0]
            else:
                grasp_ok = len(plan_result.joint_trajectory.points) > 0
            
            self.move_group.clear_pose_targets()
            
            if not grasp_ok:
                return 0.0
            
            score += 10.0
            
        except:
            return 0.0
        
        return score
    
    def target_callback(self, msg):
        """Handle new target cube pose"""
        if self.current_state not in [GraspState.IDLE, GraspState.WAITING_FOR_TARGET]:
            rospy.logwarn("Pipeline busy, ignoring new target")
            return
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("NEW TARGET RECEIVED")
        rospy.loginfo("="*60)
        
        self.target_pose = self.normalize_target_pose(msg)
        
        self.current_state = GraspState.GENERATING_CANDIDATES
        self.publish_status("GENERATING_CANDIDATES")
        self.grasp_candidates = self.generate_grasp_candidates(self.target_pose)
        
        if not self.grasp_candidates:
            rospy.logerr("No valid candidates generated")
            self.current_state = GraspState.FAILED
            self.publish_status("FAILED")
            self.reset_state()  # Reset state to IDLE so next target can be processed
            return
        
        self.current_candidate_idx = 0
        self.load_candidate(0)
        self.execute_grasp_sequence()
    
    def load_candidate(self, idx):
        """Load grasp candidate"""
        if idx >= len(self.grasp_candidates):
            return False
        
        pre_grasp, grasp, lift, retreat, score = self.grasp_candidates[idx]
        self.current_pre_grasp_pose = pre_grasp
        self.current_grasp_pose = grasp
        self.current_lift_pose = lift
        self.current_retreat_pose = retreat
        rospy.loginfo(f"Loaded candidate {idx+1}/{len(self.grasp_candidates)}, score={score:.1f}")
        return True
    
    def execute_grasp_sequence(self):
        """Execute complete grasp with improved state machine"""
        try:
            self.current_trial = {
                'start_time': time.time(),
                'target_position': [
                    self.target_pose.pose.position.x,
                    self.target_pose.pose.position.y,
                    self.target_pose.pose.position.z
                ],
                'success': False,
                'failure_reason': None,
                'candidates_tried': 0,
                'retries': 0,
                'cartesian_attempts': []
            }
            
            rospy.loginfo("\n" + "="*60)
            rospy.loginfo("EXECUTING GRASP SEQUENCE")
            rospy.loginfo("="*60)
            
            # Execute pipeline
            if not self.move_to_home():
                self.handle_failure("HOME_FAILED", "INITIAL_HOME")
                return
            
            if not self.open_gripper():
                self.handle_failure("GRIPPER_OPEN_FAILED", "OPEN")
                return
            
            if not self.plan_to_pre_grasp():
                # Try next candidate instead of failing immediately
                if not self.try_next_candidate_and_retry():
                    self.handle_failure("PRE_GRASP_PLANNING_FAILED", "PRE_GRASP")
                return
            
            if not self.cartesian_approach_with_fallback():
                self.handle_failure("CARTESIAN_APPROACH_FAILED", "APPROACH")
                return
            
            if not self.close_gripper():
                self.handle_failure("GRIPPER_CLOSE_FAILED", "CLOSE")
                return
            
            if not self.cartesian_lift_with_fallback():
                # Try next candidate instead of failing immediately
                if not self.try_next_candidate_and_retry():
                    self.handle_failure("CARTESIAN_LIFT_FAILED", "LIFT")
                return
            
            if not self.retreat_to_safe():
                self.handle_failure("RETREAT_FAILED", "RETREAT")
                return
            
            if not self.move_to_home():
                self.handle_failure("HOME_FAILED", "FINAL_HOME")
                return
            
            # Success
            self.handle_success()
        
        except Exception as e:
            rospy.logerr(f"Exception in execute_grasp_sequence: {e}")
            self.handle_failure(f"EXCEPTION: {str(e)}", "SEQUENCE")
            import traceback
            traceback.print_exc()
    
    def move_to_home(self):
        """Move to home configuration"""
        self.current_state = GraspState.MOVING_TO_HOME
        self.publish_status("MOVING_TO_HOME")
        rospy.loginfo("\nMoving to HOME")
        
        home_joints = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        return self.plan_and_execute_joints(home_joints, "HOME")
    
    def open_gripper(self):
        """Open gripper"""
        self.current_state = GraspState.OPENING_GRIPPER
        self.publish_status("OPENING_GRIPPER")
        rospy.loginfo("\nOpening gripper")
        
        if not self.use_gripper:
            rospy.sleep(0.3)
            return True
        
        goal = MoveGoal()
        goal.width = self.gripper_open_width
        goal.speed = 0.1
        
        self.gripper_move_client.send_goal(goal)
        return self.gripper_move_client.wait_for_result(rospy.Duration(5.0))
    
    def plan_to_pre_grasp(self):
        """Plan and execute to pre-grasp using global planner"""
        self.current_state = GraspState.PLANNING_PRE_GRASP
        self.publish_status("PLANNING_PRE_GRASP")
        rospy.loginfo("\nPlanning to PRE-GRASP")
        
        return self.plan_with_retry(
            self.current_pre_grasp_pose, "PRE_GRASP", max_retries=3
        )
    
    def cartesian_approach_with_fallback(self):
        """Cartesian approach with intelligent fallback"""
        self.current_state = GraspState.CARTESIAN_APPROACH
        self.publish_status("CARTESIAN_APPROACH")
        rospy.loginfo("\nCartesian APPROACH")
        
        # Strategy 1: Normal approach
        fraction, success = self.compute_and_execute_cartesian(
            [self.current_grasp_pose.pose],
            self.cartesian_step_size,
            "Approach (normal)"
        )
        
        if success and fraction >= self.cartesian_min_fraction:
            return True
        
        rospy.logwarn(f"Approach failed (fraction={fraction:.2f}), trying fallbacks...")
        
        # Strategy 2: Finer step size
        finer_step = self.cartesian_step_size / self.cartesian_step_downsample_factor
        fraction, success = self.compute_and_execute_cartesian(
            [self.current_grasp_pose.pose],
            finer_step,
            "Approach (fine step)"
        )
        
        if success and fraction >= self.cartesian_min_fraction:
            return True
        
        rospy.logwarn(f"Fine step failed (fraction={fraction:.2f}), trying shorter distance...")
        
        # Strategy 3: Shorter approach distance
        shortened_grasp = copy.deepcopy(self.current_grasp_pose)
        # Only go partway down
        z_offset = (self.current_grasp_pose.pose.position.z - 
                   self.current_pre_grasp_pose.pose.position.z) * self.cartesian_distance_scale
        shortened_grasp.pose.position.z = (self.current_pre_grasp_pose.pose.position.z + 
                                          z_offset)
        
        fraction, success = self.compute_and_execute_cartesian(
            [shortened_grasp.pose],
            finer_step,
            "Approach (shortened)"
        )
        
        if success and fraction >= 0.80:  # Lower threshold for partial approach
            rospy.loginfo("Partial approach succeeded, executing remaining descent...")
            
            # Now approach the full depth
            fraction2, success2 = self.compute_and_execute_cartesian(
                [self.current_grasp_pose.pose],
                finer_step,
                "Approach (remaining)"
            )
            
            if success2 and fraction2 >= 0.80:
                return True
        
        # All strategies failed, try next candidate
        rospy.logerr("All approach strategies failed")
        return self.try_next_candidate_and_retry()
    
    def compute_and_execute_cartesian(self, waypoints, step_size, description):
        """Compute and execute Cartesian path, return (fraction, success)"""
        try:
            # Noetic MoveIt: compute_cartesian_path expects list of Pose objects
            # Input waypoints may be Pose or PoseStamped; extract .pose if needed
            pose_waypoints = []
            for wp in waypoints:
                if isinstance(wp, PoseStamped):
                    pose_waypoints.append(wp.pose)
                else:
                    pose_waypoints.append(wp)
            
            (plan, fraction) = self.move_group.compute_cartesian_path(
                pose_waypoints, step_size, avoid_collisions=True
            )
            
            self.current_trial['cartesian_attempts'].append({
                'description': description,
                'step_size': step_size,
                'fraction': fraction
            })
            
            rospy.loginfo(f"{description}: fraction={fraction*100:.1f}%")
            
            if fraction < 0.5:
                rospy.logwarn(f"{description}: fraction too low")
                return fraction, False
            
            start_time = time.time()
            success = self.move_group.execute(plan, wait=True)
            exec_time = time.time() - start_time
            
            if success:
                rospy.loginfo(f"{description}: executed in {exec_time:.2f}s")
            else:
                rospy.logwarn(f"{description}: execution failed")
            
            return fraction, success
            
        except Exception as e:
            rospy.logerr(f"{description}: exception {e}")
            return 0.0, False
    
    def cartesian_lift_with_fallback(self):
        """Cartesian lift with intelligent fallback"""
        self.current_state = GraspState.CARTESIAN_LIFT
        self.publish_status("CARTESIAN_LIFT")
        rospy.loginfo("\nCartesian LIFT")
        
        # Strategy 1: Normal lift
        fraction, success = self.compute_and_execute_cartesian(
            [self.current_lift_pose.pose],
            self.cartesian_step_size,
            "Lift (normal)"
        )
        
        if success and fraction >= self.cartesian_min_fraction:
            return True
        
        rospy.logwarn(f"Lift failed (fraction={fraction:.2f}), trying finer step...")
        
        # Strategy 2: Finer step
        finer_step = self.cartesian_step_size / self.cartesian_step_downsample_factor
        fraction, success = self.compute_and_execute_cartesian(
            [self.current_lift_pose.pose],
            finer_step,
            "Lift (fine)"
        )
        
        if success and fraction >= self.cartesian_min_fraction:
            return True
        
        rospy.logerr("Cartesian lift failed all strategies")
        return False
    
    def retreat_to_safe(self):
        """Retreat to safe height before returning home"""
        self.current_state = GraspState.RETREAT
        self.publish_status("RETREAT")
        rospy.loginfo("\nRetreating to safe height")
        
        return self.plan_with_retry(
            self.current_retreat_pose, "RETREAT", max_retries=2
        )
    
    def close_gripper(self):
        """Close gripper"""
        self.current_state = GraspState.CLOSING_GRIPPER
        self.publish_status("CLOSING_GRIPPER")
        rospy.loginfo("\nClosing gripper")
        
        if not self.use_gripper:
            rospy.sleep(0.3)
            return True
        
        goal = GraspGoal()
        goal.width = self.gripper_close_width
        goal.speed = 0.1
        goal.force = self.gripper_max_effort
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        
        self.gripper_grasp_client.send_goal(goal)
        return self.gripper_grasp_client.wait_for_result(rospy.Duration(5.0))
    
    def plan_with_retry(self, pose, description, max_retries=3):
        """
        Plan and execute with layered retry strategy:
        Retry 1: Increase planning_time
        Retry 2: Relax goal tolerance
        Retry 3: Switch planner
        """
        original_time = self.original_planning_time
        original_tolerance = self.original_goal_tolerance
        current_planner = self.move_group.get_planner_id()
        
        for attempt in range(max_retries):
            self.current_trial['retries'] += 1
            
            if attempt == 0:
                rospy.loginfo(f"{description}: attempt 1")
            elif attempt == 1:
                # Increase planning time
                new_time = original_time * 2.0
                self.move_group.set_planning_time(new_time)
                rospy.loginfo(f"{description}: retry 1 with {new_time:.1f}s planning time")
            elif attempt == 2:
                # Relax goal tolerance
                relaxed_tolerance = original_tolerance * 2.0
                self.move_group.set_goal_tolerance(relaxed_tolerance)
                rospy.loginfo(f"{description}: retry 2 with relaxed tolerance")
            
            self.move_group.set_pose_target(pose.pose)
            plan_result = self.move_group.plan()
            
            if isinstance(plan_result, tuple):
                success = plan_result[0]
                plan = plan_result[1]
            else:
                plan = plan_result
                success = len(plan.joint_trajectory.points) > 0
            
            self.move_group.clear_pose_targets()
            
            if success:
                exec_success = self.move_group.execute(plan, wait=True)
                
                # Restore original parameters
                self.move_group.set_planning_time(original_time)
                self.move_group.set_goal_tolerance(original_tolerance)
                
                if exec_success:
                    rospy.loginfo(f"{description}: executed successfully")
                    return True
            
            rospy.logwarn(f"{description}: attempt {attempt+1} failed")
        
        # Restore parameters
        self.move_group.set_planning_time(original_time)
        self.move_group.set_goal_tolerance(original_tolerance)
        
        return False
    
    def plan_and_execute_joints(self, joint_values, description, max_retries=3):
        """Plan and execute to joint configuration with retry"""
        for attempt in range(max_retries):
            try:
                self.move_group.set_joint_value_target(joint_values)
                
                plan_result = self.move_group.plan()
                
                # Handle both old (tuple) and new (RobotTrajectory) return formats
                if isinstance(plan_result, tuple):
                    success = plan_result[0]
                    plan = plan_result[1]
                else:
                    plan = plan_result
                    success = len(plan.joint_trajectory.points) > 0
                
                if not success:
                    rospy.logwarn(f"{description} planning failed (attempt {attempt+1}/{max_retries})")
                    if attempt < max_retries - 1:
                        rospy.sleep(0.5)
                    continue
                
                # Execution
                exec_success = self.move_group.execute(plan, wait=True)
                
                if exec_success:
                    rospy.loginfo(f"{description} executed")
                    return True
                else:
                    rospy.logwarn(f"{description} execution failed (attempt {attempt+1}/{max_retries})")
                    if attempt < max_retries - 1:
                        rospy.sleep(0.5)
                    continue
            
            except Exception as e:
                rospy.logwarn(f"{description} exception: {e} (attempt {attempt+1}/{max_retries})")
                if attempt < max_retries - 1:
                    rospy.sleep(0.5)
        
        rospy.logerr(f"{description} failed after {max_retries} attempts")
        return False
    
    def try_next_candidate_and_retry(self):
        """Try next grasp candidate with recursive retry"""
        self.current_candidate_idx += 1
        self.current_trial['candidates_tried'] = self.current_candidate_idx + 1
        
        if self.current_candidate_idx >= len(self.grasp_candidates):
            rospy.logerr("No more candidates available")
            self.handle_failure("ALL_CANDIDATES_EXHAUSTED", "APPROACH")
            return False
        
        if self.current_candidate_idx >= self.max_candidate_retries:
            rospy.logerr("Reached max candidate retry limit")
            self.handle_failure("CANDIDATE_RETRY_LIMIT_EXCEEDED", "APPROACH")
            return False
        
        if self.load_candidate(self.current_candidate_idx):
            rospy.loginfo(f"Retrying with candidate {self.current_candidate_idx + 1}/{len(self.grasp_candidates)}...")
            # Re-plan to pre-grasp
            if self.plan_to_pre_grasp():
                # Try approach
                result = self.cartesian_approach_with_fallback()
                if result:
                    return True
                # If approach failed, it already tried next candidate, return result
                return result
            else:
                # Pre-grasp planning failed for this candidate, try next one recursively
                rospy.logwarn(f"Candidate {self.current_candidate_idx} pre-grasp planning failed, trying next...")
                return self.try_next_candidate_and_retry()
        
        return False
    
    def handle_success(self):
        """Handle successful grasp"""
        self.current_state = GraspState.SUCCESS
        self.publish_status("SUCCESS")
        self.current_trial['success'] = True
        self.current_trial['total_time'] = time.time() - self.current_trial['start_time']
        self.save_trial_result()
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("GRASP COMPLETED SUCCESSFULLY")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Total time: {self.current_trial['total_time']:.2f}s")
        rospy.loginfo(f"Candidates tried: {self.current_trial['candidates_tried']}")
        rospy.loginfo(f"Retries: {self.current_trial['retries']}")
        
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
        
        rospy.logerr(f"\nGRASP FAILED: {reason} at {stage}")
        
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
            'failure_reason': self.current_trial.get('failure_reason', 'N/A')
        }
        self.grasp_results.append(result)
    
    def reset_state(self):
        """Reset state for next grasp"""
        self.current_state = GraspState.IDLE
        self.target_pose = None
        self.grasp_candidates = []
        self.current_candidate_idx = 0
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
