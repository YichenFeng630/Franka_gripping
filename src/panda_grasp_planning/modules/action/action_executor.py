#!/usr/bin/env python3
"""
Action Executor: Bridges V3 pipeline with unified action space
用于V3 pipeline + 学习策略的动作执行器
"""

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import tf.transformations as tfs
from franka_interface import ArmInterface, GripperInterface
import moveit_commander

class ActionExecutor:
    def __init__(self, action_config_file):
        """
        Initialize action executor with unified action space
        
        Args:
            action_config_file: Path to config/action_space.yaml
        """
        self.config = self._load_yaml(action_config_file)
        self.action_dim = self.config['action_space']['dim']
        self.control_hz = self.config['action_space']['control_hz']
        self.dt = 1.0 / self.control_hz
        
        # Initialize robot interfaces
        self.arm = ArmInterface()
        self.gripper = GripperInterface()
        self.move_group = moveit_commander.MoveGroupCommander('panda_arm')
        
        # Current state
        self.current_ee_pose = None
        
    def execute_action(self, action):
        """
        Execute a single action from the unified action space
        
        Args:
            action: np.array of shape (7,) 
                   [dx, dy, dz, droll, dpitch, dyaw, gripper_cmd]
        
        Returns:
            success: bool
            next_observation: dict with state, image, etc.
        """
        if len(action) != self.action_dim:
            rospy.logerr(f"Action dim mismatch: {len(action)} vs {self.action_dim}")
            return False, None
        
        dx, dy, dz, droll, dpitch, dyaw, gripper_cmd = action
        
        # 1. Get current end-effector pose
        current_pose = self.arm.endpoint_pose()
        pos = np.array([current_pose['position'].x, 
                       current_pose['position'].y,
                       current_pose['position'].z])
        ori = np.array([current_pose['orientation'].x,
                       current_pose['orientation'].y,
                       current_pose['orientation'].z,
                       current_pose['orientation'].w])
        
        # 2. 限幅 (Safety gate)
        action_scaled = self._apply_safety_bounds(action)
        
        # 3. Compute target pose (world frame or ee-local frame)
        if self.config['action_space']['coordinate_frame'] == 'world':
            target_pos = pos + action_scaled[:3]
            
            # Convert rotation increments to pose
            current_euler = tfs.euler_from_quaternion(ori)
            target_euler = (current_euler[0] + action_scaled[3],
                          current_euler[1] + action_scaled[4],
                          current_euler[2] + action_scaled[5])
            target_ori = tfs.quaternion_from_euler(*target_euler)
        else:  # ee_local
            # Rotate increments in end-effector frame
            target_pos, target_ori = self._apply_ee_local_action(
                pos, ori, action_scaled[:3], action_scaled[3:6])
        
        # 4. 检查工作空间
        if not self._check_workspace(target_pos):
            rospy.logwarn("Action out of workspace!")
            return False, None
        
        # 5. Motion planning using Cartesian path
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'panda_link0'
        target_pose.pose.position.x = target_pos[0]
        target_pose.pose.position.y = target_pos[1]
        target_pose.pose.position.z = target_pos[2]
        target_pose.pose.orientation.x = target_ori[0]
        target_pose.pose.orientation.y = target_ori[1]
        target_pose.pose.orientation.z = target_ori[2]
        target_pose.pose.orientation.w = target_ori[3]
        
        # Try Cartesian path first (smooth)
        plan, fraction = self.move_group.compute_cartesian_path(
            [target_pose.pose], 0.01, 0.0)
        
        if fraction < 0.5:
            # Fallback: RRT
            self.move_group.set_pose_target(target_pose)
            plan = self.move_group.plan()[1]
            if not plan.joint_trajectory.points:
                rospy.logwarn("Cannot plan path!")
                return False, None
        
        # 6. Execute motion
        try:
            self.move_group.execute(plan, wait=True)
        except Exception as e:
            rospy.logerr(f"Execution failed: {e}")
            return False, None
        
        # 7. 夹爪控制
        if self.config['action_space']['bounds']['gripper']['type'] == 'binary':
            if gripper_cmd > 0.5:
                self.gripper.grasp(width=0.025, force=50.0)  # Close
            else:
                self.gripper.open()  # Open
        else:  # continuous
            self.gripper.move_to(gripper_cmd)
        
        # 8. 获取下一步观测
        next_obs = self._get_observation()
        
        return True, next_obs
    
    def _apply_safety_bounds(self, action):
        """Apply safety limits to action"""
        bounds = self.config['action_space']['bounds']
        action_safe = action.copy()
        
        # Limit translation
        trans_norm = np.linalg.norm(action_safe[:3])
        if trans_norm > bounds['translation']['range_m']:
            action_safe[:3] *= bounds['translation']['range_m'] / trans_norm
        
        # Limit rotation
        rot_norm = np.linalg.norm(action_safe[3:6])
        if rot_norm > bounds['rotation']['range_rad']:
            action_safe[3:6] *= bounds['rotation']['range_rad'] / rot_norm
        
        return action_safe
    
    def _check_workspace(self, pos):
        """Check if position is within workspace"""
        bounds = self.config['action_space']['workspace']
        return (bounds['x_range'][0] <= pos[0] <= bounds['x_range'][1] and
                bounds['y_range'][0] <= pos[1] <= bounds['y_range'][1] and
                bounds['z_range'][0] <= pos[2] <= bounds['z_range'][1])
    
    def _get_observation(self):
        """Get current observation (state + image + depth)"""
        # TODO: Implement observation collection
        pass
    
    def _load_yaml(self, filepath):
        import yaml
        with open(filepath) as f:
            return yaml.safe_load(f)
    
    def _apply_ee_local_action(self, pos, ori, trans, rot):
        """Apply action in end-effector local frame"""
        # Rotate translation to world frame
        rot_matrix = tfs.quaternion_matrix(ori)[:3, :3]
        trans_world = pos + rot_matrix @ trans
        
        # Compose rotations
        local_rot = tfs.quaternion_from_euler(*rot)
        combined_rot = tfs.quaternion_multiply(ori, local_rot)
        
        return trans_world, combined_rot


if __name__ == '__main__':
    rospy.init_node('action_executor')
    executor = ActionExecutor('/opt/ros_ws/src/panda_grasp_planning/config/action_space.yaml')
    
    # Test: Execute a simple action
    test_action = np.array([0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])  # Move 2cm in +X, grasp
    success, obs = executor.execute_action(test_action)
    print(f"Execution success: {success}")
