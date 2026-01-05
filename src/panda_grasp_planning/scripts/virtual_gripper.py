#!/usr/bin/env python3
"""
Virtual Gripper Simulation for Gazebo
======================================

Simulates gripper grasping by creating/destroying fixed joints between
the gripper and the cube in Gazebo.

This allows testing grasp pipelines without a real gripper driver.
"""

import rospy
from gazebo_msgs.srv import AttachModel, DetachModel
from std_msgs.msg import String
import json


class VirtualGripper:
    """Virtual gripper that simulates grasping via Gazebo joint constraints"""
    
    def __init__(self):
        rospy.init_node('virtual_gripper')
        
        self.is_grasping = False
        self.grasped_cube = None
        
        # Gazebo services for attaching/detaching models
        rospy.wait_for_service('/gazebo/attach_model')
        rospy.wait_for_service('/gazebo/detach_model')
        
        self.attach_client = rospy.ServiceProxy('/gazebo/attach_model', AttachModel)
        self.detach_client = rospy.ServiceProxy('/gazebo/detach_model', DetachModel)
        
        # Subscribe to cube color/property information
        rospy.Subscriber('/cube_properties', String, self.on_cube_spawned, queue_size=10)
        
        # Subscribe to gripper commands
        rospy.Subscriber('/gripper_command', String, self.on_gripper_command, queue_size=10)
        
        rospy.loginfo("[VirtualGripper] Initialized and ready to simulate grasping")
    
    def on_cube_spawned(self, msg):
        """Receive notification of a new cube spawned"""
        try:
            data = json.loads(msg.data)
            cube_name = data.get('cube_name')
            rospy.loginfo(f"[VirtualGripper] Detected cube: {cube_name}")
        except:
            pass
    
    def on_gripper_command(self, msg):
        """Handle gripper open/close commands"""
        command = msg.data.lower()
        
        if command == 'close' or command == 'grasp':
            self.grasp_nearest_cube()
        elif command == 'open' or command == 'release':
            self.release_cube()
    
    def grasp_cube(self, cube_name):
        """
        Attach cube to gripper by creating a fixed joint in Gazebo.
        
        Args:
            cube_name: Name of the cube model in Gazebo (e.g., 'cube_0')
        """
        try:
            rospy.loginfo(f"[VirtualGripper] Grasping cube: {cube_name}")
            
            # Attach the cube to the gripper in Gazebo
            # This creates a fixed joint between panda_hand and the cube
            response = self.attach_client(
                model_name_1='panda',
                link_name_1='panda_hand',
                model_name_2=cube_name,
                link_name_2=cube_name
            )
            
            if response.success:
                self.is_grasping = True
                self.grasped_cube = cube_name
                rospy.loginfo(f"[VirtualGripper] ✓ Successfully grasped {cube_name}")
                return True
            else:
                rospy.logerr(f"[VirtualGripper] Failed to grasp {cube_name}: {response.status_message}")
                return False
                
        except Exception as e:
            rospy.logerr(f"[VirtualGripper] Exception during grasp: {e}")
            return False
    
    def grasp_nearest_cube(self):
        """Grasp the nearest cube (simplified: grasp first available)"""
        # In a real implementation, would find the nearest cube
        # For now, try to grasp cube_0 or any available cube
        for i in range(20):
            cube_name = f'cube_{i}'
            if self.grasp_cube(cube_name):
                return
    
    def release_cube(self):
        """Release the grasped cube"""
        if not self.is_grasping or not self.grasped_cube:
            rospy.loginfo("[VirtualGripper] No cube is currently grasped")
            return True
        
        try:
            rospy.loginfo(f"[VirtualGripper] Releasing cube: {self.grasped_cube}")
            
            response = self.detach_client(
                model_name_1='panda',
                link_name_1='panda_hand',
                model_name_2=self.grasped_cube,
                link_name_2=self.grasped_cube
            )
            
            if response.success:
                self.is_grasping = False
                self.grasped_cube = None
                rospy.loginfo(f"[VirtualGripper] ✓ Successfully released cube")
                return True
            else:
                rospy.logerr(f"[VirtualGripper] Failed to release: {response.status_message}")
                return False
                
        except Exception as e:
            rospy.logerr(f"[VirtualGripper] Exception during release: {e}")
            return False
    
    def run(self):
        """Keep the node running"""
        rospy.spin()


if __name__ == '__main__':
    gripper = VirtualGripper()
    gripper.run()
