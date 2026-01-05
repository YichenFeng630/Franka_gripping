#!/usr/bin/env python3
"""
Example: VLA Integration with Grasp Candidate Generator
========================================================

This example shows how to use the standalone GraspCandidateGenerator module
for integrating with Vision Language Action (VLA) systems.

The generator returns structured GraspCandidate objects with:
- Complete pose information (grasp, pre_grasp, lift, retreat)
- Approach vector and distance for Cartesian planning
- Metadata (yaw_idx, direction_idx, priority, feasibility)

This allows VLA systems to:
1. Query candidates and their properties
2. Select based on custom criteria (not just priority)
3. Visualize approach vectors in 3D
4. Provide feedback loop for learning
"""

import rospy
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped
import moveit_commander

# Import the standalone module
from grasp_candidate_generator import GraspCandidateGenerator


class VLAGraspPlanner:
    """Example VLA integration with grasp candidate generator."""
    
    def __init__(self):
        """Initialize VLA grasp planner."""
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('vla_grasp_planner', anonymous=True)
        
        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander('panda_arm')
        
        # Create candidate generator
        self.generator = GraspCandidateGenerator(
            move_group=self.move_group,
            reference_frame='panda_link0',
            params={
                'yaw_angles': [0, np.pi/2, np.pi, -np.pi/2],
                'approach_directions': [
                    (0.0, 0.0),    # Top-down
                    (0.1, 0.0),    # +X approach
                    (-0.1, 0.0),   # -X approach
                    (0.0, 0.1),    # +Y approach
                ],
                'pre_grasp_offset_z': 0.15,
                'lift_offset_z': 0.15,
                'max_candidates': 8,
            }
        )
    
    def plan_grasp_candidates(self, cube_pose):
        """
        Generate and return structured candidates for VLA to process.
        
        Args:
            cube_pose: PoseStamped with cube center in panda_link0
        
        Returns:
            List of GraspCandidate objects
        """
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("VLA GRASP PLANNING")
        rospy.loginfo("="*70)
        
        candidates = self.generator.generate(cube_pose, verbose=True)
        
        rospy.loginfo(f"\nGenerated {len(candidates)} candidates")
        rospy.loginfo("="*70)
        
        return candidates
    
    def select_candidate_vla_style(self, candidates):
        """
        Example: VLA selects a candidate based on custom criteria.
        
        In a real VLA system, this would be replaced by a learned policy
        or neural network that scores candidates based on visual features.
        """
        if not candidates:
            return None
        
        rospy.loginfo("\n" + "-"*70)
        rospy.loginfo("VLA CANDIDATE SELECTION")
        rospy.loginfo("-"*70)
        
        # Example 1: Simple heuristic - prefer top-down approaches
        rospy.loginfo("\nStrategy 1: Prefer top-down approach (dir_idx=0)")
        top_down = [c for c in candidates if c.direction_idx == 0]
        if top_down:
            selected = top_down[0]  # Take highest priority top-down
            rospy.loginfo(f"  Selected: Candidate {selected.candidate_id} "
                         f"(yaw={selected.yaw_deg:.0f}°, priority={selected.priority:.0f})")
            return selected
        
        # Example 2: Prefer shortest approach
        rospy.loginfo("\nStrategy 2: Prefer shortest approach distance")
        by_distance = sorted(candidates, key=lambda c: c.approach_distance)
        selected = by_distance[0]
        rospy.loginfo(f"  Selected: Candidate {selected.candidate_id} "
                     f"(distance={selected.approach_distance:.4f}m, priority={selected.priority:.0f})")
        return selected
    
    def analyze_candidate(self, candidate):
        """
        Detailed analysis of a single candidate for VLA debugging.
        """
        rospy.loginfo("\n" + "-"*70)
        rospy.loginfo("CANDIDATE DETAILED ANALYSIS")
        rospy.loginfo("-"*70)
        
        rospy.loginfo(f"\nCandidate ID: {candidate.candidate_id}")
        rospy.loginfo(f"  Yaw: {candidate.yaw_deg:.1f}° (idx={candidate.yaw_idx})")
        rospy.loginfo(f"  Approach direction: {candidate.approach_direction_xy} "
                     f"(idx={candidate.direction_idx})")
        
        rospy.loginfo(f"\nQuality Metrics:")
        rospy.loginfo(f"  Score (raw): {candidate.score:.2f}")
        rospy.loginfo(f"  Priority (0-100): {candidate.priority:.1f}")
        
        rospy.loginfo(f"\nFeasibility:")
        rospy.loginfo(f"  Pre-grasp plannable: {candidate.feasibility['pre_grasp']}")
        rospy.loginfo(f"  Grasp plannable: {candidate.feasibility['grasp']}")
        rospy.loginfo(f"  Lift plannable: {candidate.feasibility['lift']}")
        
        rospy.loginfo(f"\nApproach Information:")
        rospy.loginfo(f"  Approach vector: [{candidate.approach_vector[0]:.4f}, "
                     f"{candidate.approach_vector[1]:.4f}, "
                     f"{candidate.approach_vector[2]:.4f}]")
        rospy.loginfo(f"  Approach distance: {candidate.approach_distance:.4f}m")
        
        rospy.loginfo(f"\nPose Information:")
        g = candidate.grasp_pose.pose.position
        p = candidate.pre_grasp_pose.pose.position
        rospy.loginfo(f"  Grasp position: ({g.x:.4f}, {g.y:.4f}, {g.z:.4f})")
        rospy.loginfo(f"  Pre-grasp position: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
        
        # Serializable dict for storage/logging
        rospy.loginfo(f"\nSerialized form (for logging/storage):")
        rospy.loginfo(f"  {candidate.to_dict()}")
        
        rospy.loginfo("-"*70)
    
    def generate_all_approach_vectors(self, candidates):
        """
        Extract approach vectors for visualization in VLA frontend.
        
        Returns:
            List of dicts suitable for JSON serialization
        """
        vectors = []
        for candidate in candidates:
            vectors.append({
                'candidate_id': candidate.candidate_id,
                'yaw_deg': candidate.yaw_deg,
                'approach_direction': candidate.approach_direction_xy,
                'grasp_pos': [
                    candidate.grasp_pose.pose.position.x,
                    candidate.grasp_pose.pose.position.y,
                    candidate.grasp_pose.pose.position.z,
                ],
                'pre_grasp_pos': [
                    candidate.pre_grasp_pose.pose.position.x,
                    candidate.pre_grasp_pose.pose.position.y,
                    candidate.pre_grasp_pose.pose.position.z,
                ],
                'approach_vector': candidate.approach_vector,
                'approach_distance': candidate.approach_distance,
                'priority': candidate.priority,
            })
        
        return vectors


def main():
    """Example usage."""
    try:
        planner = VLAGraspPlanner()
        
        # Create test target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'panda_link0'
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.15
        target_pose.pose.orientation.w = 1.0
        
        # Generate candidates
        candidates = planner.plan_grasp_candidates(target_pose)
        
        if not candidates:
            rospy.logwarn("No valid candidates generated!")
            return
        
        # VLA selects a candidate
        selected = planner.select_candidate_vla_style(candidates)
        
        if selected:
            # Analyze the selected candidate
            planner.analyze_candidate(selected)
        
        # Extract approach vectors for visualization
        vectors = planner.generate_all_approach_vectors(candidates)
        rospy.loginfo(f"\nGenerated {len(vectors)} approach vectors for visualization")
        
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("Example completed successfully!")
        rospy.loginfo("="*70)
        
        moveit_commander.roscpp_shutdown()
        
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
