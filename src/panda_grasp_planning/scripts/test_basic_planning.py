#!/usr/bin/env python3
"""
Phase 1: Basic MoveIt Planning Test
Test if RRTConnect pipeline works from home to a fixed pose and back.

This script:
1. Initializes MoveIt for panda_arm planning group
2. Moves from current pose to a predefined target pose
3. Returns to home configuration
4. Verifies the planning pipeline is functioning correctly

All planning is done in panda_link0 frame (base frame).
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import math

class BasicPlanningTest:
    def __init__(self):
        # Initialize moveit_commander and rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('basic_planning_test', anonymous=True)
        
        # Instantiate a RobotCommander object (interface to robot kinematics and planning groups)
        self.robot = moveit_commander.RobotCommander()
        
        # Instantiate a PlanningSceneInterface object (interface to the world surrounding the robot)
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Instantiate a MoveGroupCommander object for the panda_arm planning group
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        
        # Set the reference frame for pose targets
        self.move_group.set_pose_reference_frame("panda_link0")
        
        # Set planning parameters
        self.move_group.set_planner_id("RRTConnect")
        self.move_group.set_planning_time(10.0)  # Allow up to 10 seconds for planning
        self.move_group.set_num_planning_attempts(10)  # Try up to 10 times
        self.move_group.set_max_velocity_scaling_factor(0.3)  # Slow and safe for testing
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        
        # Display trajectory publisher for RViz
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("Basic Planning Test Initialized")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Planning frame: {self.move_group.get_planning_frame()}")
        rospy.loginfo(f"End effector link: {self.move_group.get_end_effector_link()}")
        rospy.loginfo(f"Planning group: panda_arm")
        rospy.loginfo(f"Planner: RRTConnect")
        rospy.loginfo("="*60)
        
        # Wait for the planning scene to be ready
        rospy.sleep(2.0)
        
    def get_home_joint_values(self):
        """Define home configuration for Panda robot"""
        # Standard Panda home pose (all joints at 0 except for joint 4)
        home_joints = [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]
        return home_joints
    
    def create_target_pose(self, x, y, z, roll, pitch, yaw):
        """
        Create a target pose in panda_link0 frame.
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians
        
        Returns:
            PoseStamped message
        """
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "panda_link0"
        pose_target.header.stamp = rospy.Time.now()
        
        pose_target.pose.position.x = x
        pose_target.pose.position.y = y
        pose_target.pose.position.z = z
        
        # Convert RPY to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_target.pose.orientation.x = q[0]
        pose_target.pose.orientation.y = q[1]
        pose_target.pose.orientation.z = q[2]
        pose_target.pose.orientation.w = q[3]
        
        return pose_target
    
    def plan_and_execute(self, target, description=""):
        """
        Plan and execute a trajectory to the target.
        
        Args:
            target: Joint values (list) or PoseStamped
            description: Description of the movement
        
        Returns:
            True if successful, False otherwise
        """
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"Planning: {description}")
        rospy.loginfo(f"{'='*60}")
        
        try:
            # Set the target
            if isinstance(target, list):
                self.move_group.set_joint_value_target(target)
                rospy.loginfo("Target: Joint space goal")
            else:
                self.move_group.set_pose_target(target)
                rospy.loginfo(f"Target: Cartesian space goal")
                rospy.loginfo(f"  Position: [{target.pose.position.x:.3f}, "
                            f"{target.pose.position.y:.3f}, {target.pose.position.z:.3f}]")
            
            # Plan
            rospy.loginfo("Planning trajectory...")
            plan = self.move_group.plan()
            
            # Check if plan is successful (handle both old and new API)
            if isinstance(plan, tuple):
                success, trajectory, planning_time, error_code = plan
            else:
                success = plan
                trajectory = plan
            
            if not success or (isinstance(plan, tuple) and not plan[0]):
                rospy.logerr("Planning failed!")
                return False
            
            rospy.loginfo(f"Planning succeeded!")
            if isinstance(plan, tuple):
                rospy.loginfo(f"Planning time: {planning_time:.3f}s")
            
            # Execute
            rospy.loginfo("Executing trajectory...")
            execute_result = self.move_group.execute(trajectory if isinstance(plan, tuple) else plan, wait=True)
            
            if execute_result:
                rospy.loginfo("Execution succeeded! ✓")
            else:
                rospy.logerr("Execution failed!")
                return False
            
            # Make sure there are no residual movements
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            rospy.sleep(1.0)
            return True
            
        except Exception as e:
            rospy.logerr(f"Error during planning/execution: {e}")
            return False
    
    def run_test(self):
        """
        Run the complete Phase 1 test:
        1. Move from current pose to home
        2. Move to a fixed target pose
        3. Return to home
        """
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("PHASE 1: BASIC PLANNING TEST")
        rospy.loginfo("="*60 + "\n")
        
        # Get current state
        current_joints = self.move_group.get_current_joint_values()
        rospy.loginfo(f"Current joint values: {[f'{j:.3f}' for j in current_joints]}")
        
        # Step 1: Move to home configuration
        rospy.loginfo("\n>>> Step 1: Moving to HOME configuration")
        home_joints = self.get_home_joint_values()
        success = self.plan_and_execute(home_joints, "Move to HOME")
        
        if not success:
            rospy.logerr("Failed to reach home configuration!")
            return False
        
        rospy.sleep(2.0)
        
        # Step 2: Move to a fixed Cartesian target pose
        # Target: reasonable pose in front of robot, gripper pointing down
        # Position: 40cm forward, 0cm lateral, 40cm high
        # Orientation: gripper pointing down (z-axis down)
        rospy.loginfo("\n>>> Step 2: Moving to FIXED TARGET pose")
        target_pose = self.create_target_pose(
            x=0.4,      # 40cm forward
            y=0.0,      # centered
            z=0.4,      # 40cm high
            roll=math.pi,   # gripper pointing down (z-axis flipped)
            pitch=0.0,
            yaw=0.0
        )
        
        success = self.plan_and_execute(target_pose, "Move to FIXED TARGET")
        
        if not success:
            rospy.logerr("Failed to reach target pose!")
            return False
        
        rospy.sleep(2.0)
        
        # Step 3: Return to home
        rospy.loginfo("\n>>> Step 3: Returning to HOME")
        success = self.plan_and_execute(home_joints, "Return to HOME")
        
        if not success:
            rospy.logerr("Failed to return home!")
            return False
        
        # Test completed successfully
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("✓ PHASE 1 TEST COMPLETED SUCCESSFULLY!")
        rospy.loginfo("="*60)
        rospy.loginfo("RRTConnect pipeline is working correctly.")
        rospy.loginfo("Robot can plan and execute from home to target and back.")
        rospy.loginfo("="*60 + "\n")
        
        return True

def main():
    try:
        test = BasicPlanningTest()
        success = test.run_test()
        
        if success:
            rospy.loginfo("All tests passed! Ready for Phase 2.")
        else:
            rospy.logerr("Test failed. Please check the errors above.")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by user.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
