#!/usr/bin/env python3
"""
Phase 2: Grasp Pose Generator Node
Generates pre-grasp, grasp, and lift poses from a target object pose.

This node:
1. Subscribes to /target_cube_pose (geometry_msgs/PoseStamped)
2. Generates three poses based on grasp parameters:
   - Pre-grasp: Approach pose above the object
   - Grasp: Final grasp pose
   - Lift: Retreat pose after grasping
3. Publishes all three poses for the pipeline to execute

All poses are in panda_link0 frame.
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class GraspPoseGenerator:
    def __init__(self):
        rospy.init_node('grasp_pose_generator', anonymous=False)
        
        # Load parameters from config/grasp_params.yaml
        self.load_parameters()
        
        # Publishers for individual poses
        self.pre_grasp_pub = rospy.Publisher(
            '/pre_grasp_pose',
            PoseStamped,
            queue_size=1
        )
        self.grasp_pub = rospy.Publisher(
            '/grasp_pose',
            PoseStamped,
            queue_size=1
        )
        self.lift_pub = rospy.Publisher(
            '/lift_pose',
            PoseStamped,
            queue_size=1
        )
        
        # Subscribers
        self.target_sub = rospy.Subscriber(
            '/target_cube_pose',
            PoseStamped,
            self.target_callback,
            queue_size=1
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("Grasp Pose Generator Node Initialized")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Pre-grasp offset: {self.pre_grasp_offset_z:.3f}m")
        rospy.loginfo(f"Grasp offset: {self.grasp_offset_z:.3f}m")
        rospy.loginfo(f"Lift offset: {self.lift_offset_z:.3f}m")
        rospy.loginfo(f"Gripper orientation (RPY): [{self.gripper_roll:.3f}, "
                     f"{self.gripper_pitch:.3f}, {self.gripper_yaw:.3f}]")
        rospy.loginfo("="*60)
        rospy.loginfo("Waiting for target cube pose on /target_cube_pose...")
        
    def load_parameters(self):
        """Load grasp parameters from ROS parameter server"""
        # Height offsets
        self.pre_grasp_offset_z = rospy.get_param(
            '~pre_grasp_offset_z',
            rospy.get_param('/grasp/pre_grasp_offset_z', 0.10)
        )
        self.grasp_offset_z = rospy.get_param(
            '~grasp_offset_z',
            rospy.get_param('/grasp/grasp_offset_z', 0.02)
        )
        self.lift_offset_z = rospy.get_param(
            '~lift_offset_z',
            rospy.get_param('/grasp/lift_offset_z', 0.15)
        )
        
        # Gripper orientation (Roll-Pitch-Yaw)
        self.gripper_roll = rospy.get_param(
            '~gripper_roll',
            rospy.get_param('/grasp/gripper_orientation/roll', math.pi)
        )
        self.gripper_pitch = rospy.get_param(
            '~gripper_pitch',
            rospy.get_param('/grasp/gripper_orientation/pitch', 0.0)
        )
        self.gripper_yaw = rospy.get_param(
            '~gripper_yaw',
            rospy.get_param('/grasp/gripper_orientation/yaw', 0.0)
        )
        
        # Reference frame
        self.reference_frame = rospy.get_param(
            '~reference_frame',
            rospy.get_param('/planning/reference_frame', 'panda_link0')
        )
    
    def create_pose(self, x, y, z, roll, pitch, yaw, frame_id=None):
        """
        Create a PoseStamped message.
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians
            frame_id: Reference frame (default: self.reference_frame)
        
        Returns:
            PoseStamped message
        """
        if frame_id is None:
            frame_id = self.reference_frame
            
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Convert RPY to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
    
    def generate_grasp_poses(self, target_pose):
        """
        Generate pre-grasp, grasp, and lift poses from target pose.
        
        Args:
            target_pose: PoseStamped of the target object (cube center or top surface)
        
        Returns:
            Tuple of (pre_grasp_pose, grasp_pose, lift_pose)
        """
        # Extract target position
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_z = target_pose.pose.position.z
        
        # Generate pre-grasp pose (approach from above)
        pre_grasp_pose = self.create_pose(
            x=target_x,
            y=target_y,
            z=target_z + self.pre_grasp_offset_z,
            roll=self.gripper_roll,
            pitch=self.gripper_pitch,
            yaw=self.gripper_yaw,
            frame_id=target_pose.header.frame_id
        )
        
        # Generate grasp pose
        grasp_pose = self.create_pose(
            x=target_x,
            y=target_y,
            z=target_z + self.grasp_offset_z,
            roll=self.gripper_roll,
            pitch=self.gripper_pitch,
            yaw=self.gripper_yaw,
            frame_id=target_pose.header.frame_id
        )
        
        # Generate lift pose (retreat upward from grasp)
        lift_pose = self.create_pose(
            x=target_x,
            y=target_y,
            z=target_z + self.grasp_offset_z + self.lift_offset_z,
            roll=self.gripper_roll,
            pitch=self.gripper_pitch,
            yaw=self.gripper_yaw,
            frame_id=target_pose.header.frame_id
        )
        
        return pre_grasp_pose, grasp_pose, lift_pose
    
    def target_callback(self, msg):
        """
        Callback for target cube pose.
        Generates and publishes grasp poses.
        """
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("Received target cube pose")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Frame: {msg.header.frame_id}")
        rospy.loginfo(f"Position: [{msg.pose.position.x:.3f}, "
                     f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}]")
        
        # Verify frame is correct
        if msg.header.frame_id != self.reference_frame:
            rospy.logwarn(f"Target pose frame '{msg.header.frame_id}' does not match "
                         f"reference frame '{self.reference_frame}'. "
                         f"Proceeding anyway, but ensure coordinate systems are correct.")
        
        # Generate grasp poses
        rospy.loginfo("\nGenerating grasp poses...")
        pre_grasp, grasp, lift = self.generate_grasp_poses(msg)
        
        # Log generated poses
        rospy.loginfo("\n>>> Pre-Grasp Pose:")
        rospy.loginfo(f"  Position: [{pre_grasp.pose.position.x:.3f}, "
                     f"{pre_grasp.pose.position.y:.3f}, {pre_grasp.pose.position.z:.3f}]")
        
        rospy.loginfo("\n>>> Grasp Pose:")
        rospy.loginfo(f"  Position: [{grasp.pose.position.x:.3f}, "
                     f"{grasp.pose.position.y:.3f}, {grasp.pose.position.z:.3f}]")
        
        rospy.loginfo("\n>>> Lift Pose:")
        rospy.loginfo(f"  Position: [{lift.pose.position.x:.3f}, "
                     f"{lift.pose.position.y:.3f}, {lift.pose.position.z:.3f}]")
        
        # Publish individual poses for debugging
        self.pre_grasp_pub.publish(pre_grasp)
        self.grasp_pub.publish(grasp)
        self.lift_pub.publish(lift)
        
        # Note: In a real implementation, we would publish a custom message
        # containing all three poses. For now, the pipeline node will subscribe
        # to individual topics or we can use a simple approach.
        
        rospy.loginfo("\n✓ Grasp poses generated and published!")
        rospy.loginfo("="*60 + "\n")
    
    def spin(self):
        """Keep the node running"""
        rospy.spin()


def main():
    try:
        generator = GraspPoseGenerator()
        generator.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Grasp Pose Generator node interrupted.")
    except Exception as e:
        rospy.logerr(f"Error in Grasp Pose Generator: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
