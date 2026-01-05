#!/usr/bin/env python3
"""
Grasp Candidate Generator Module
=================================

Standalone module for generating grasp candidates from a target cube pose.
Designed to be independent of the full pipeline for easy integration with VLA systems.

Features:
- Multi-directional approach candidates (yaw × xy approach directions)
- Structured candidate representation with metadata
- Candidate scoring based on IK feasibility
- Configurable parameters for different use cases
- Clean API for external integration

Candidate Structure:
    GraspCandidate {
        # Core poses
        grasp_pose (PoseStamped): End-effector pose at grasp point
        pre_grasp_pose (PoseStamped): Approach pose (with xy offset + z offset)
        lift_pose (PoseStamped): Lift pose after grasping
        retreat_pose (PoseStamped): Safe retreat height
        
        # Approach info for Cartesian
        approach_vector ([dx, dy, dz]): From grasp to pre_grasp in base frame
        approach_distance (float): Total approach distance
        
        # Metadata for tracking & priority
        candidate_id (int): Unique ID for this candidate
        yaw_idx (int): Index in yaw_angles list
        direction_idx (int): Index in approach_directions list
        priority (float): 0-100, higher is better (based on score)
        score (float): Raw IK feasibility score
        feasibility ({pre_grasp, grasp, lift}): Which poses are IK-feasible
    }

Usage:
    from grasp_candidate_generator import GraspCandidateGenerator, GraspCandidate
    
    generator = GraspCandidateGenerator(
        move_group=move_group,
        params={
            'yaw_angles': [0, np.pi/2, np.pi, -np.pi/2],
            'approach_directions': [(0, 0), (0.1, 0), (-0.1, 0), (0, 0.1)],
            ...
        }
    )
    
    candidates = generator.generate(cube_center_pose)  # Returns list of GraspCandidate
    for candidate in candidates:
        print(f"ID: {candidate.candidate_id}, Priority: {candidate.priority:.0f}")
        print(f"  Yaw {candidate.yaw_idx}, Dir {candidate.direction_idx}")
        print(f"  Feasible: {candidate.feasibility}")
"""

import rospy
import copy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class GraspCandidate:
    """Structured representation of a single grasp candidate."""
    
    def __init__(self, candidate_id=0):
        """Initialize an empty grasp candidate."""
        # Core poses
        self.grasp_pose = None  # PoseStamped: Final grasp position
        self.pre_grasp_pose = None  # PoseStamped: Approach position
        self.lift_pose = None  # PoseStamped: Lift position
        self.retreat_pose = None  # PoseStamped: Retreat to safe height
        
        # Approach info for Cartesian
        self.approach_vector = None  # [dx, dy, dz] from grasp to pre_grasp
        self.approach_distance = 0.0  # Total distance for Cartesian
        self.approach_direction_xy = None  # Original (dx, dy) offset
        
        # Metadata
        self.candidate_id = candidate_id
        self.yaw_idx = 0  # Index in yaw_angles array
        self.direction_idx = 0  # Index in approach_directions array
        self.yaw_rad = 0.0  # Actual yaw value in radians
        self.yaw_deg = 0.0  # Yaw in degrees for logging
        
        # Priority & quality metrics
        self.score = 0.0  # Raw IK feasibility score
        self.priority = 0.0  # 0-100, higher is better
        
        # Feasibility info for debugging
        self.feasibility = {
            'pre_grasp': False,
            'grasp': False,
            'lift': False
        }
        
        # Timing
        self.timestamp = None
        self.target_id = None  # Link to parent target
    
    def compute_approach_vector(self):
        """Compute approach vector and distance from grasp to pre_grasp."""
        if self.grasp_pose is None or self.pre_grasp_pose is None:
            return None
        
        g = self.grasp_pose.pose.position
        p = self.pre_grasp_pose.pose.position
        
        dx = p.x - g.x
        dy = p.y - g.y
        dz = p.z - g.z
        
        self.approach_vector = [dx, dy, dz]
        self.approach_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        return self.approach_vector
    
    def to_dict(self):
        """Convert to dictionary for serialization/logging."""
        return {
            'id': self.candidate_id,
            'yaw_idx': self.yaw_idx,
            'yaw_deg': self.yaw_deg,
            'direction_idx': self.direction_idx,
            'direction_xy': self.approach_direction_xy,
            'priority': round(self.priority, 2),
            'score': round(self.score, 2),
            'approach_distance': round(self.approach_distance, 4),
            'feasibility': self.feasibility,
        }
    
    def __repr__(self):
        return (f"GraspCandidate(id={self.candidate_id}, yaw={self.yaw_deg:.0f}°, "
                f"dir={self.direction_idx}, priority={self.priority:.0f})")


class GraspCandidateGenerator:
    """Generate grasp candidates for a target cube pose."""
    
    def __init__(self, move_group, reference_frame='panda_link0', params=None):
        """
        Initialize the grasp candidate generator.
        
        Args:
            move_group: MoveIt MoveGroupCommander instance
            reference_frame: Base frame for pose generation (default: 'panda_link0')
            params: Dictionary of parameters:
                - yaw_angles: List of yaw angles in radians
                - approach_directions: List of (x, y) approach offsets
                - grasp_offset_z: Offset from cube center for grasp pose (default: 0.0)
                - pre_grasp_offset_z: Height above grasp pose (default: 0.15)
                - lift_offset_z: Height for lift pose (default: 0.15)
                - retreat_offset_z: Height for retreat pose (default: 0.25)
                - gripper_roll: Gripper roll angle (default: 0)
                - gripper_pitch: Gripper pitch angle (default: np.pi/4)
                - max_candidates: Max candidates to return (default: 16)
                - planning_time: Time for scoring plans (default: 1.0)
                - min_score: Minimum score to keep candidate (default: 0.0)
        """
        self.move_group = move_group
        self.reference_frame = reference_frame
        
        # Default parameters
        self.yaw_angles = [0, np.pi/2, np.pi, -np.pi/2]
        self.approach_directions = [
            (0.0, 0.0),   # from above
            (0.1, 0.0),   # from +X
            (-0.1, 0.0),  # from -X
            (0.0, 0.1),   # from +Y
        ]
        self.grasp_offset_z = 0.0
        self.pre_grasp_offset_z = 0.15
        self.lift_offset_z = 0.15
        self.retreat_offset_z = 0.25
        self.gripper_roll = 0.0
        self.gripper_pitch = np.pi / 4
        self.max_candidates = 16
        self.planning_time = 1.0
        self.min_score = 0.0
        
        # Override with provided parameters
        if params:
            self._update_params(params)
    
    def _update_params(self, params):
        """Update generator parameters from dictionary."""
        if 'yaw_angles' in params:
            self.yaw_angles = params['yaw_angles']
        if 'approach_directions' in params:
            self.approach_directions = params['approach_directions']
        if 'grasp_offset_z' in params:
            self.grasp_offset_z = params['grasp_offset_z']
        if 'pre_grasp_offset_z' in params:
            self.pre_grasp_offset_z = params['pre_grasp_offset_z']
        if 'lift_offset_z' in params:
            self.lift_offset_z = params['lift_offset_z']
        if 'retreat_offset_z' in params:
            self.retreat_offset_z = params['retreat_offset_z']
        if 'gripper_roll' in params:
            self.gripper_roll = params['gripper_roll']
        if 'gripper_pitch' in params:
            self.gripper_pitch = params['gripper_pitch']
        if 'max_candidates' in params:
            self.max_candidates = params['max_candidates']
        if 'planning_time' in params:
            self.planning_time = params['planning_time']
        if 'min_score' in params:
            self.min_score = params['min_score']
    
    def generate(self, cube_center_pose, sort_by_score=True, verbose=False):
        """
        Generate grasp candidates from target cube center pose.
        
        Args:
            cube_center_pose: PoseStamped message with cube center position
            sort_by_score: If True, sort candidates by priority (highest first)
            verbose: If True, log candidate details
        
        Returns:
            List of GraspCandidate objects, sorted by priority (highest first)
        
        Raises:
            Exception: If pose generation or scoring fails
        """
        candidates = []
        candidate_id = 0
        
        num_combinations = len(self.yaw_angles) * len(self.approach_directions)
        
        for yaw_idx, yaw in enumerate(self.yaw_angles):
            for dir_idx, approach_dir in enumerate(self.approach_directions):
                try:
                    # Generate poses
                    pre_grasp, grasp, lift, retreat = self.generate_pose_set(
                        cube_center_pose, yaw, approach_dir
                    )
                    
                    # Score the candidate
                    score, feasibility = self.score_candidate(pre_grasp, grasp, lift)
                    
                    if score >= self.min_score:
                        # Create structured candidate
                        candidate = GraspCandidate(candidate_id=candidate_id)
                        candidate.grasp_pose = grasp
                        candidate.pre_grasp_pose = pre_grasp
                        candidate.lift_pose = lift
                        candidate.retreat_pose = retreat
                        
                        # Metadata
                        candidate.yaw_idx = yaw_idx
                        candidate.direction_idx = dir_idx
                        candidate.yaw_rad = yaw
                        candidate.yaw_deg = math.degrees(yaw)
                        candidate.approach_direction_xy = approach_dir
                        
                        # Quality metrics
                        candidate.score = score
                        candidate.priority = self._score_to_priority(score)
                        candidate.feasibility = feasibility
                        
                        # Compute approach vector
                        candidate.compute_approach_vector()
                        
                        candidates.append(candidate)
                        candidate_id += 1
                        
                        if verbose:
                            rospy.loginfo(
                                f"Candidate {candidate.candidate_id}: yaw={candidate.yaw_deg:.0f}° "
                                f"approach=({approach_dir[0]:+.2f}, {approach_dir[1]:+.2f}), "
                                f"score={score:.1f}, priority={candidate.priority:.0f}"
                            )
                except Exception as e:
                    if verbose:
                        rospy.logwarn(
                            f"Failed to generate candidate for yaw={math.degrees(yaw):.0f}°, "
                            f"approach={approach_dir}: {str(e)}"
                        )
                    continue
        
        if sort_by_score:
            candidates.sort(key=lambda x: x.priority, reverse=True)
        
        candidates = candidates[:self.max_candidates]
        
        if verbose:
            rospy.loginfo(
                f"Generated {len(candidates)}/{num_combinations} valid candidates"
            )
        
        return candidates
    
    def generate_pose_set(self, cube_center, yaw, approach_dir):
        """
        Generate (pre_grasp, grasp, lift, retreat) for given yaw + approach direction.
        
        Args:
            cube_center: PoseStamped with target position
            yaw: Yaw angle in radians
            approach_dir: Tuple of (dx, dy) approach offset
        
        Returns:
            Tuple of (pre_grasp_pose, grasp_pose, lift_pose, retreat_pose)
        """
        # Grasp pose at cube center + offset
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.reference_frame
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.pose.position.x = cube_center.pose.position.x
        grasp_pose.pose.position.y = cube_center.pose.position.y
        grasp_pose.pose.position.z = cube_center.pose.position.z + self.grasp_offset_z
        
        # Orientation (roll, pitch, yaw)
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
        
        # Lift pose: above grasp pose
        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.pose.position.z += self.lift_offset_z
        
        # Retreat pose: safe height before returning home
        retreat_pose = copy.deepcopy(grasp_pose)
        retreat_pose.pose.position.z += self.retreat_offset_z
        
        return pre_grasp_pose, grasp_pose, lift_pose, retreat_pose
    
    def score_candidate(self, pre_grasp, grasp, lift):
        """
        Score a candidate based on IK feasibility.
        
        Args:
            pre_grasp: PoseStamped for pre-grasp position
            grasp: PoseStamped for grasp position
            lift: PoseStamped for lift position
        
        Returns:
            Tuple of (score, feasibility_dict)
            score: Float (0-20, higher is better)
            feasibility: {'pre_grasp': bool, 'grasp': bool, 'lift': bool}
        """
        score = 0.0
        feasibility = {
            'pre_grasp': False,
            'grasp': False,
            'lift': False
        }
        
        try:
            # Save original planning time
            original_time = self.move_group.get_planning_time()
            self.move_group.set_planning_time(self.planning_time)
            
            # Check pre-grasp
            self.move_group.set_pose_target(pre_grasp.pose)
            plan_result = self.move_group.plan()
            
            if isinstance(plan_result, tuple):
                pre_ok = plan_result[0]
            else:
                pre_ok = len(plan_result.joint_trajectory.points) > 0
            
            self.move_group.clear_pose_targets()
            
            if not pre_ok:
                self.move_group.set_planning_time(original_time)
                return 0.0, feasibility
            
            feasibility['pre_grasp'] = True
            score += 10.0
            
            # Check grasp
            self.move_group.set_pose_target(grasp.pose)
            plan_result = self.move_group.plan()
            
            if isinstance(plan_result, tuple):
                grasp_ok = plan_result[0]
            else:
                grasp_ok = len(plan_result.joint_trajectory.points) > 0
            
            self.move_group.clear_pose_targets()
            
            if not grasp_ok:
                self.move_group.set_planning_time(original_time)
                return score, feasibility
            
            feasibility['grasp'] = True
            score += 5.0
            
            # Check lift
            self.move_group.set_pose_target(lift.pose)
            plan_result = self.move_group.plan()
            
            if isinstance(plan_result, tuple):
                lift_ok = plan_result[0]
            else:
                lift_ok = len(plan_result.joint_trajectory.points) > 0
            
            self.move_group.clear_pose_targets()
            
            if lift_ok:
                feasibility['lift'] = True
                score += 5.0
            
            # Restore original planning time
            self.move_group.set_planning_time(original_time)
            
        except Exception as e:
            rospy.logwarn(f"Error scoring candidate: {str(e)}")
            return 0.0, feasibility
        
        return score, feasibility
    
    def _score_to_priority(self, score):
        """
        Convert raw score (0-20) to priority (0-100).
        Can be overridden for custom priority strategies.
        """
        return min(100.0, score * 5.0)
    
    def get_params(self):
        """Return current parameters as dictionary."""
        return {
            'yaw_angles': self.yaw_angles,
            'approach_directions': self.approach_directions,
            'grasp_offset_z': self.grasp_offset_z,
            'pre_grasp_offset_z': self.pre_grasp_offset_z,
            'lift_offset_z': self.lift_offset_z,
            'retreat_offset_z': self.retreat_offset_z,
            'gripper_roll': self.gripper_roll,
            'gripper_pitch': self.gripper_pitch,
            'max_candidates': self.max_candidates,
            'planning_time': self.planning_time,
            'min_score': self.min_score,
        }
    
    def set_params(self, **kwargs):
        """
        Update parameters using keyword arguments.
        
        Example:
            generator.set_params(
                yaw_angles=[0, np.pi/2],
                max_candidates=8
            )
        """
        self._update_params(kwargs)


if __name__ == '__main__':
    """Example usage and testing."""
    import sys
    
    try:
        import moveit_commander
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('test_grasp_generator', anonymous=True)
        
        # Initialize MoveIt
        robot = moveit_commander.RobotCommander()
        move_group = moveit_commander.MoveGroupCommander('panda_arm')
        
        # Create generator
        generator = GraspCandidateGenerator(
            move_group=move_group,
            reference_frame='panda_link0'
        )
        
        # Create test pose
        test_pose = PoseStamped()
        test_pose.header.frame_id = 'panda_link0'
        test_pose.pose.position.x = 0.5
        test_pose.pose.position.y = 0.0
        test_pose.pose.position.z = 0.15
        test_pose.pose.orientation.w = 1.0
        
        # Generate candidates
        rospy.loginfo("Generating grasp candidates...")
        candidates = generator.generate(test_pose, verbose=True)
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"Generated {len(candidates)} candidates")
        rospy.loginfo(f"{'='*60}")
        
        for i, candidate in enumerate(candidates):
            rospy.loginfo(f"\nCandidate #{i}:")
            rospy.loginfo(f"  ID: {candidate.candidate_id}")
            rospy.loginfo(f"  Yaw: {candidate.yaw_deg:.0f}° (idx={candidate.yaw_idx})")
            rospy.loginfo(f"  Direction: {candidate.approach_direction_xy} (idx={candidate.direction_idx})")
            rospy.loginfo(f"  Score: {candidate.score:.1f}")
            rospy.loginfo(f"  Priority: {candidate.priority:.0f}")
            rospy.loginfo(f"  Approach distance: {candidate.approach_distance:.4f}m")
            rospy.loginfo(f"  Feasibility: {candidate.feasibility}")
            rospy.loginfo(f"  Candidate dict: {candidate.to_dict()}")
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo("Test completed successfully!")
        rospy.loginfo(f"{'='*60}")
        
        moveit_commander.roscpp_shutdown()
        
    except ImportError:
        print("This module requires ROS and MoveIt. Run within a ROS environment.")
        sys.exit(1)
