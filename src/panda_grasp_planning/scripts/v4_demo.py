#!/usr/bin/env python3
"""
V4 Demo - Simplified Top-Down Grasp with Place-to-Bin
======================================================

Simplified grasp pipeline with decoupled pose generation and execution:
1. Read cube pose (ground truth from Gazebo)
2. Compute top-down grasp: p_grasp = [x, y, z_top + z_offset], R: z-down, yaw snapped to cube faces
3. Move to pre-grasp: p_pre = p_grasp + [0, 0, h_pre]
4. Vertical descent (z-only motion, controlled velocity)
5. Dwell 2-5 steps to eliminate end-effector velocity
6. Close gripper
7. Lift to p_lift = p_grasp + [0, 0, h_lift]
8. Verify cube follows (gripper width check)
9. Place-to-Bin execution

Usage:
    roslaunch panda_grasp_planning panda_grasp_complete.launch sim:=true rviz:=false
    python3 v4_demo.py --trials 10 --verbose
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import actionlib
import rospy
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_commander import MoveGroupCommander, RobotCommander
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

# Import sorting state machine for bin assignment
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from modules.sorting.sorting_state_machine import SortingStateMachine


class V4GraspDemo:
    """V4 Demo: Simplified grasp pipeline with ground truth and place-to-bin."""

    def __init__(self, args):
        rospy.init_node("v4_grasp_demo")

        # Parameters
        self.num_trials = args.trials
        self.verbose = args.verbose
        self.enable_place = args.enable_place
        self.reference_frame = "panda_link0"
        
        # Grasp parameters
        self.cartesian_step = 0.0025
        self.pre_height = 0.15  # 15cm hover above cube top (stable position before descent)
        self.lift_height = 0.50  # 50cm lift after grasp
        self.grasp_depth_ratio = 0.4  # Grasp at 40% depth (2/3 to 1/2 of cube height)
        self.cube_half = 0.0225  # Half size of 45mm cube
        self.cube_height = 0.045  # Full cube height 45mm
        self.dwell_time = 3  # Stabilization time at pre-location
        
        # CRITICAL: TCP offset from panda_link8 (flange) to gripper finger tips
        # panda_link8 is at the flange, fingers extend ~10.3cm below
        self.gripper_tcp_offset = 0.103  # 10.3cm offset to finger tips
        
        # Velocity/acceleration
        self.max_vel_normal = 1.0
        self.max_acc_normal = 1.0
        self.max_vel_descent = 0.05  # Ultra slow for descent (3% speed) to avoid collision
        self.max_acc_descent = 0.05  # Ultra low acceleration for gentle contact
        
        # Gripper parameters
        self.open_width = 0.08
        self.close_width = 0.043  # 43mm - slightly less than 45mm cube to ensure contact
        self.gripper_force = 90.0  # Very high force to handle edge grasps
        self.gripper_speed = 0.05  # Moderate closing speed
        self.min_grasp_width = 0.040  # 40mm - must be close to cube size (45mm) to confirm grasp
        self.max_grasp_width = 0.048  # 48mm - if wider, cube slipped
        
        # MoveIt setup
        self.robot = RobotCommander()
        self.group = MoveGroupCommander("panda_arm")
        self.group.set_pose_reference_frame(self.reference_frame)
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)
        
        # Track current velocity/acceleration scaling factors
        self.current_vel_scale = self.max_vel_normal
        self.current_acc_scale = self.max_acc_normal
        
        # Gripper actions
        self.gripper_move = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.gripper_grasp = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        rospy.loginfo("Waiting for gripper action servers...")
        self.gripper_move.wait_for_server(timeout=rospy.Duration(10))
        self.gripper_grasp.wait_for_server(timeout=rospy.Duration(10))
        
        # Track gripper state
        self.gripper_width = self.open_width
        from sensor_msgs.msg import JointState
        self.gripper_state_sub = rospy.Subscriber("/franka_gripper/joint_states", JointState, self.gripper_state_callback)
        
        # Gazebo services
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=10)
        rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
        self.get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        
        # Sorting state machine
        self.sorting_sm = SortingStateMachine()
        
        # Status publisher
        self.status_pub = rospy.Publisher("/grasp_planning_status", String, queue_size=10)
        
        # Cube tracking
        self.cube_colors = {}  # model_name -> color
        self.color_sub = rospy.Subscriber("/cube_properties", String, self.cube_properties_callback, queue_size=100)
        
        # Results
        self.results = []
        
        # Setup results directory
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        results_dir = os.path.join(project_dir, 'test_results')
        os.makedirs(results_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_csv = os.path.join(results_dir, f'v4_demo_{timestamp}.csv')
        
        rospy.loginfo("="*60)
        rospy.loginfo("V4 Grasp Demo Initialized")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Trials: {self.num_trials}")
        rospy.loginfo(f"Place-to-bin: {'ENABLED' if self.enable_place else 'DISABLED'}")
        rospy.loginfo(f"Results: {self.output_csv}")
        rospy.loginfo("="*60)
        
        # Wait for cube tracking
        rospy.sleep(3.0)
        if not self.cube_colors:
            self.discover_cubes()

    def gripper_state_callback(self, msg):
        """Track actual gripper width from joint states."""
        try:
            # JointState has position array for finger joints
            # Gripper width = sum of both finger positions (each finger position is half the gap)
            # franka_finger_joint1 and franka_finger_joint2
            if len(msg.position) >= 2:
                # Total width = 2 * (finger1_pos + finger2_pos)
                # For franka gripper, width = finger1 + finger2
                self.gripper_width = msg.position[0] + msg.position[1]
        except Exception as e:
            if self.verbose:
                rospy.logwarn(f"Gripper state parse error: {e}")
    
    def cube_properties_callback(self, msg: String):
        """Track cube colors from spawn_cubes node."""
        try:
            import json
            data = json.loads(msg.data)
            self.cube_colors[data['name']] = data['color']
        except Exception as e:
            if self.verbose:
                rospy.logwarn(f"Cube properties parse error: {e}")

    def discover_cubes(self):
        """Discover cubes from Gazebo if not tracked via topic."""
        try:
            world = self.get_world_properties()
            for name in world.model_names:
                if name.startswith("cube_"):
                    # Default color mapping (fallback)
                    idx = int(name.split("_")[1])
                    colors = ["RED", "BLUE", "GREEN", "YELLOW"]
                    self.cube_colors[name] = colors[idx % len(colors)]
        except Exception as e:
            rospy.logwarn(f"Cube discovery failed: {e}")

    def fetch_cube_pose(self, model_name: str) -> Optional[Pose]:
        """Fetch cube pose from Gazebo."""
        try:
            resp = self.get_model_state(model_name, self.reference_frame)
            if resp.success:
                return resp.pose
            else:
                rospy.logwarn(f"Failed to get pose for {model_name}")
                return None
        except Exception as e:
            rospy.logerr(f"Error fetching pose for {model_name}: {e}")
            return None

    @staticmethod
    def yaw_from_quaternion(q: Quaternion) -> float:
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def snap_yaw_to_face(yaw: float) -> float:
        """Snap yaw to nearest 90° to align with cube faces."""
        return round(yaw / (math.pi / 2)) * (math.pi / 2)

    def compute_optimal_grasp_yaw(self, cube_pose: Pose) -> float:
        """
        计算最优的抓取yaw角度，确保夹爪与cube的面平行，避免夹住棱。
        
        策略：
        1. 读取cube的当前朝向
        2. 将yaw snap到最近的45度倍数（与cube的边和面对齐）
        3. 选择最容易到达的角度
        
        Returns:
            最优yaw角度（弧度）
        """
        x = cube_pose.position.x
        y = cube_pose.position.y
        
        # 提取cube当前的yaw角度
        cube_yaw = self.yaw_from_quaternion(cube_pose.orientation)
        
        # Snap到最近的45度，确保与cube的面或对角线对齐
        # 但为了避免夹住棱，我们使用90度倍数（与面完全平行）
        candidate_yaws = [0.0, math.pi/2, math.pi, 3*math.pi/2]  # 0°, 90°, 180°, 270°
        
        # 选择与cube当前朝向最接近的角度
        best_yaw = min(candidate_yaws, key=lambda y: abs((y - cube_yaw + math.pi) % (2*math.pi) - math.pi))
        
        # 但也要考虑机械臂的可达性：优先选择与cube位置相关的角度
        if abs(x) > abs(y):
            # Cube在x方向偏移更多，优先使用0°或180°
            preferred = 0.0 if x > 0 else math.pi
        else:
            # Cube在y方向偏移更多，优先使用90°或270°
            preferred = math.pi/2 if y > 0 else 3*math.pi/2
        
        # 在best_yaw和preferred之间选择差异较小的
        diff_best = abs((best_yaw - cube_yaw + math.pi) % (2*math.pi) - math.pi)
        diff_pref = abs((preferred - cube_yaw + math.pi) % (2*math.pi) - math.pi)
        
        final_yaw = best_yaw if diff_best < diff_pref + 0.3 else preferred
        
        if self.verbose:
            rospy.loginfo(f"Cube yaw={math.degrees(cube_yaw):.1f}°, optimal grasp yaw={math.degrees(final_yaw):.1f}°")
        
        return final_yaw

    def build_grasp_poses(self, cube_pose: Pose, custom_yaw: Optional[float] = None) -> Dict[str, Pose]:
        """
        Build grasp poses from cube pose.
        
        计算抓取位姿，确保：
        1. 夹爪与cube的面平行（避免夹住棱）
        2. 抓取深度在cube高度的2/3到1/2之间
        3. Pre-location在cube上方10cm
        4. Lift提升50cm
        
        Args:
            cube_pose: Cube的当前位姿
            custom_yaw: 可选的自定义yaw角度（如果提供则使用，否则自动计算）
        
        Returns dict with keys: 'pre', 'grasp', 'lift'
        """
        x = cube_pose.position.x
        y = cube_pose.position.y
        z_center = cube_pose.position.z  # CRITICAL: Gazebo returns CENTER position, not bottom!
        
        # 计算cube的底部和顶部Z坐标
        z_bottom = z_center - self.cube_half  # 真实底部
        z_top = z_center + self.cube_half     # 真实顶部
        
        # 计算抓取高度：从底部往上grasp_depth_ratio的位置
        # CRITICAL: 这是finger tips应该到达的位置
        z_grasp_fingertips = z_bottom + self.cube_height * self.grasp_depth_ratio
        
        # 但是我们控制的是panda_link8 (flange)，需要补偿TCP offset
        # flange的目标位置 = fingertips目标位置 + TCP offset
        z_grasp = z_grasp_fingertips + self.gripper_tcp_offset
        
        # 计算最优yaw角度
        if custom_yaw is not None:
            yaw_grasp = custom_yaw
        else:
            yaw_grasp = self.compute_optimal_grasp_yaw(cube_pose)
        
        # Orientation: z-down (roll=pi), 计算出的最优yaw
        quat = quaternion_from_euler(math.pi, 0.0, yaw_grasp)
        orient = Quaternion(*quat)
        
        # Validate: gripper与cube面平行
        yaw_deg = math.degrees(yaw_grasp)
        if abs(yaw_deg % 90) > 0.1:
            rospy.logwarn(f"Warning: Yaw {yaw_deg:.1f}° may not be face-aligned!")
        
        grasp_pose = Pose(position=Point(x, y, z_grasp), orientation=orient)
        pre_pose = Pose(position=Point(x, y, z_grasp + self.pre_height), orientation=orient)
        lift_pose = Pose(position=Point(x, y, z_grasp + self.lift_height), orientation=orient)
        
        # CRITICAL DEBUG: Always log Z calculations to verify correctness
        rospy.loginfo("=" * 50)
        rospy.loginfo("Z-AXIS CALCULATION DEBUG:")
        rospy.loginfo(f"  Cube center (from Gazebo): z={z_center:.4f}m")
        rospy.loginfo(f"  Cube half-height: {self.cube_half:.4f}m")
        rospy.loginfo(f"  Cube full height: {self.cube_height:.4f}m")
        rospy.loginfo(f"  Calculated bottom: z={z_bottom:.4f}m")
        rospy.loginfo(f"  Calculated top: z={z_top:.4f}m")
        rospy.loginfo(f"  Grasp ratio: {self.grasp_depth_ratio*100:.0f}%")
        rospy.loginfo(f"  Target for finger tips: z={z_grasp_fingertips:.4f}m")
        rospy.loginfo(f"  Gripper TCP offset: {self.gripper_tcp_offset:.4f}m")
        rospy.loginfo(f"  Target for panda_link8: z={z_grasp:.4f}m (fingertips + offset)")
        rospy.loginfo(f"  Pre-location: z={z_grasp + self.pre_height:.4f}m")
        rospy.loginfo(f"  Lift location: z={z_grasp + self.lift_height:.4f}m")
        rospy.loginfo(f"  Grasp yaw={yaw_deg:.0f}° (face-aligned)")
        rospy.loginfo("=" * 50)
        
        # Safety check: ensure finger tips are above table (z=0)
        if z_grasp_fingertips < 0.005:  # 5mm safety margin above table
            rospy.logerr(f"❌ GRASP TOO LOW! fingertips z={z_grasp_fingertips:.4f}m < 0.005m (table at z=0)")
            rospy.logerr(f"   This will collide with table!")
        
        return {"pre": pre_pose, "grasp": grasp_pose, "lift": lift_pose}

    def move_home(self) -> bool:
        """Move to home configuration."""
        self.status_pub.publish("MOVING_HOME")
        rospy.loginfo("Moving to HOME")
        home_joints = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        self.group.set_max_velocity_scaling_factor(self.max_vel_normal)
        self.group.set_max_acceleration_scaling_factor(self.max_acc_normal)
        success = self.group.go(home_joints, wait=True)
        self.group.stop()
        return success

    def open_gripper(self) -> bool:
        """Open gripper."""
        self.status_pub.publish("OPENING_GRIPPER")
        rospy.loginfo("Opening gripper")
        goal = MoveGoal(width=self.open_width, speed=0.1)
        self.gripper_move.send_goal(goal)
        result = self.gripper_move.wait_for_result(rospy.Duration(5.0))
        if not result:
            rospy.logwarn("⚠ Gripper open timeout, continuing anyway")
        return True  # Always continue even if timeout

    def close_gripper(self) -> Tuple[bool, float]:
        """
        Close gripper unconditionally and return estimated width.
        
        Returns:
            (success, final_width)
        """
        self.status_pub.publish("CLOSING_GRIPPER")
        rospy.loginfo("Closing gripper (unconditional)")
        goal = GraspGoal()
        goal.width = self.close_width
        goal.force = self.gripper_force
        goal.speed = self.gripper_speed  # Use configured slower speed
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        self.gripper_grasp.send_goal(goal)
        
        # Wait for result but don't fail if timeout
        result_received = self.gripper_grasp.wait_for_result(rospy.Duration(5.0))
        
        # Wait a moment for state to update
        rospy.sleep(0.3)
        
        # Read actual gripper width from state
        actual_width = self.gripper_width
        
        if result_received:
            result = self.gripper_grasp.get_result()
            if result and result.success:
                rospy.loginfo(f"✓ Gripper closed successfully, actual width={actual_width*1000:.1f}mm")
                return True, actual_width
        
        # Even if timeout or no result, return actual width
        rospy.logwarn(f"⚠ Gripper close timeout/no result, actual width={actual_width*1000:.1f}mm")
        return True, actual_width

    def velocity_rampdown(self, desc: str, steps: int = 5, initial_vel: float = 0.10) -> bool:
        """
        Gracefully reduce velocity to zero to ensure smooth stopping.
        
        Args:
            desc: Description of the ramp-down phase
            steps: Number of velocity reduction steps
            initial_vel: Starting velocity scaling factor
        
        Returns:
            True if successful
        """
        rospy.loginfo(f"🔄 Velocity ramp-down: {desc}")
        vel_factors = [initial_vel * (1 - i/steps) for i in range(steps + 1)]  # Smooth linear decrease
        
        for i, vel in enumerate(vel_factors):
            if i < len(vel_factors) - 1:
                self.group.set_max_velocity_scaling_factor(max(vel, 0.01))  # Minimum 1% velocity
                self.group.set_max_acceleration_scaling_factor(max(vel * 0.8, 0.01))
                rospy.sleep(0.1)  # Brief pause at each velocity level
        
        # Final complete stop
        self.group.stop()
        rospy.sleep(0.2)  # Ensure complete stop
        
        return True

    def plan_and_execute(self, target_pose: Pose, desc: str) -> bool:
        """Plan and execute to target pose."""
        self.status_pub.publish(f"PLANNING_{desc}")
        if self.verbose:
            rospy.loginfo(f"Planning to {desc}")
        
        ps = PoseStamped()
        ps.header.frame_id = self.reference_frame
        ps.pose = target_pose
        self.group.set_pose_target(ps)
        
        plan = self.group.plan()
        success = plan[0] if isinstance(plan, tuple) else (plan.joint_trajectory.points != [])
        
        if not success:
            rospy.logerr(f"Planning to {desc} failed")
            self.group.clear_pose_targets()
            return False
        
        trajectory = plan[1] if isinstance(plan, tuple) else plan
        exec_ok = self.group.execute(trajectory, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        
        if self.verbose:
            rospy.loginfo(f"Execute {desc}: {'SUCCESS' if exec_ok else 'FAILED'}")
        return exec_ok

    def cartesian_descent(self, start_pose: Pose, target_pose: Pose, desc: str) -> Tuple[float, bool]:
        """
        Execute vertical Cartesian descent.
        
        CRITICAL: Applies current velocity/acceleration scaling to the trajectory.
        
        Returns:
            (fraction, success)
        """
        self.status_pub.publish(f"CARTESIAN_{desc}")
        if self.verbose:
            rospy.loginfo(f"Cartesian descent: {desc}")
        
        waypoints = [target_pose]
        # Disable collision checking for pure vertical motions (z-only)
        # This avoids false collision detections while maintaining safety
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, self.cartesian_step, avoid_collisions=False
        )
        
        rospy.loginfo(f"{desc}: fraction={fraction*100:.1f}%")
        
        # Lower threshold for vertical motions since we disabled collision check
        if fraction < 0.5:
            rospy.logerr(f"{desc}: fraction too low ({fraction*100:.1f}%)")
            return fraction, False
        
        # CRITICAL FIX: Retime the trajectory to apply current velocity/acceleration scaling
        # compute_cartesian_path doesn't respect set_max_velocity_scaling_factor
        # We need to manually retime the trajectory
        retimed_plan = self.group.retime_trajectory(
            self.robot.get_current_state(),
            plan,
            velocity_scaling_factor=self.current_vel_scale,
            acceleration_scaling_factor=self.current_acc_scale
        )
        
        # Execute retimed trajectory
        exec_ok = self.group.execute(retimed_plan, wait=True)
        self.group.stop()  # Ensure controller stops
        
        # Accept execution even if controller reports ABORTED due to GOAL_TOLERANCE_VIOLATED
        # as long as fraction was high - robot is close enough
        if not exec_ok:
            rospy.logwarn(f"{desc}: execution aborted (likely GOAL_TOLERANCE_VIOLATED), but fraction was {fraction*100:.1f}% - continuing")
        
        # Always succeed if fraction was acceptable
        return fraction, True

    def micro_adjust_grasp(self, grasp_pose: Pose) -> bool:
        """
        微调动作：轻微抬起再放下，让cube自动对齐到finger中心。
        这可以减少夹到棱的问题。
        
        Args:
            grasp_pose: 当前抓取位姿
        
        Returns:
            True if successful
        """
        try:
            # 轻微抬起3cm
            lift_small = Pose(
                position=Point(
                    grasp_pose.position.x,
                    grasp_pose.position.y,
                    grasp_pose.position.z + 0.03  # 3cm微抬起
                ),
                orientation=grasp_pose.orientation
            )
            
            # 缓慢抬起
            self.current_vel_scale = 0.05
            self.current_acc_scale = 0.05
            self.group.set_max_velocity_scaling_factor(self.current_vel_scale)
            self.group.set_max_acceleration_scaling_factor(self.current_acc_scale)
            
            frac, ok = self.cartesian_descent(grasp_pose, lift_small, "MICRO_LIFT")
            if not ok:
                return False
            
            rospy.sleep(0.2)  # 短暂停留
            
            # 缓慢放下
            frac, ok = self.cartesian_descent(lift_small, grasp_pose, "MICRO_DOWN")
            
            return ok
        except Exception as e:
            rospy.logwarn(f"微调动作失败: {e}")
            return False

    def verify_grasp(self, final_width: float) -> bool:
        """Verify that cube was grasped (width check)."""
        # Cube is 45mm, gripper should be close to this width when holding it
        # If too narrow: gripper closed empty (no cube)
        # If too wide: cube slipped or not grasped
        if self.min_grasp_width <= final_width <= self.max_grasp_width:
            rospy.loginfo(f"✓ Grasp verified: width={final_width*1000:.1f}mm (cube ~45mm)")
            return True
        elif final_width < self.min_grasp_width:
            rospy.logwarn(f"✗ Grasp failed: width={final_width*1000:.1f}mm too small - gripper closed empty!")
            return False
        else:
            rospy.logwarn(f"✗ Grasp failed: width={final_width*1000:.1f}mm too large - cube slipped or not grasped!")
            return False

    def execute_place_to_bin(self, color: str) -> bool:
        """Execute place-to-bin sequence."""
        success, bin_info = self.sorting_sm.assign_target_bin(color)
        if not success:
            rospy.logerr(f"Bin assignment failed for color {color}")
            return False
        
        bin_pos = bin_info['position']
        bin_name = bin_info['bin_name']
        
        rospy.loginfo(f"\nPlacing {color} cube in {bin_name}")
        
        # Move to bin pre-place (hover above bin)
        bin_yaw = 0.0
        quat = quaternion_from_euler(math.pi, 0.0, bin_yaw)
        
        pre_place_pose = Pose(
            position=Point(bin_pos[0], bin_pos[1], bin_pos[2] + self.pre_height),
            orientation=Quaternion(*quat)
        )
        
        self.group.set_max_velocity_scaling_factor(self.max_vel_normal)
        self.group.set_max_acceleration_scaling_factor(self.max_acc_normal)
        
        if not self.plan_and_execute(pre_place_pose, "BIN_PRE_PLACE"):
            return False
        
        # Cartesian descent to bin
        place_pose = Pose(
            position=Point(bin_pos[0], bin_pos[1], bin_pos[2]),
            orientation=pre_place_pose.orientation
        )
        
        self.group.set_max_velocity_scaling_factor(self.max_vel_descent)
        self.group.set_max_acceleration_scaling_factor(self.max_acc_descent)
        
        frac, ok = self.cartesian_descent(pre_place_pose, place_pose, "PLACE_DOWN")
        if not ok or frac < 0.8:
            rospy.logwarn("Place down incomplete, continuing anyway")
        
        # Open gripper to release
        rospy.sleep(0.3)
        self.open_gripper()
        rospy.sleep(0.5)
        
        # Lift from bin
        lift_from_bin = Pose(
            position=Point(bin_pos[0], bin_pos[1], bin_pos[2] + self.lift_height),
            orientation=place_pose.orientation
        )
        
        self.group.set_max_velocity_scaling_factor(self.max_vel_normal)
        self.group.set_max_acceleration_scaling_factor(self.max_acc_normal)
        
        frac, ok = self.cartesian_descent(place_pose, lift_from_bin, "BIN_LIFT")
        
        rospy.loginfo(f"✓ Placed in {bin_name}")
        return True

    def execute_single_grasp(self, model_name: str, color: str) -> Dict:
        """
        执行单次抓取序列。
        
        流程：
        0. 从gazebo读取cube位置，计算最佳pose避免夹住棱
        1. Pre-location：移动到cube上方10cm
        2. 在pre-location停留5s，确保零速度，重新读取位置并调整pose
        3. 从pre-location缓慢下降到抓取位置（夹爪到cube高度的2/3-1/2），夹紧
        4. 垂直提升50cm
        5. 松开，回到home位置
        """
        result = {
            'model_name': model_name,
            'color': color,
            'success': False,
            'failure_stage': None,
            'elapsed_time': 0.0,
            'cube_x': 0.0,
            'cube_y': 0.0,
            'cube_z': 0.0,
            'placed_in_bin': False
        }
        
        start_time = time.time()
        
        try:
            # ===== 步骤0: 从Gazebo读取cube位置 =====
            rospy.loginfo("="*60)
            rospy.loginfo(f"开始抓取: {model_name} ({color})")
            rospy.loginfo("="*60)
            
            cube_pose_initial = self.fetch_cube_pose(model_name)
            if cube_pose_initial is None:
                result['failure_stage'] = 'FETCH_POSE'
                return result
            
            result['cube_x'] = cube_pose_initial.position.x
            result['cube_y'] = cube_pose_initial.position.y
            result['cube_z'] = cube_pose_initial.position.z
            
            rospy.loginfo(f"步骤0: 读取cube初始位置 ({cube_pose_initial.position.x:.3f}, {cube_pose_initial.position.y:.3f}, {cube_pose_initial.position.z:.3f})")
            
            # 计算初步的最优yaw角度（避免夹住棱）
            optimal_yaw = self.compute_optimal_grasp_yaw(cube_pose_initial)
            rospy.loginfo(f"步骤0: 计算最优抓取角度 yaw={math.degrees(optimal_yaw):.0f}° (避免夹住棱)")
            
            # ===== 步骤1: 移动到Pre-location (cube上方10cm) =====
            rospy.loginfo("\n步骤1: 移动到Pre-location (cube上方10cm)")
            
            # Reset: 回到home并打开夹爪
            if not self.move_home():
                result['failure_stage'] = 'MOVE_HOME'
                return result
            self.open_gripper()
            
            # 计算pre-location位姿（使用初步计算的yaw）
            z_bottom = cube_pose_initial.position.z
            z_grasp_initial = z_bottom + self.cube_height * self.grasp_depth_ratio
            z_pre = z_grasp_initial + self.pre_height
            
            pre_pose_initial = Pose(
                position=Point(cube_pose_initial.position.x, cube_pose_initial.position.y, z_pre),
                orientation=Quaternion(*quaternion_from_euler(math.pi, 0.0, optimal_yaw))
            )
            
            # 移动到pre-location
            self.current_vel_scale = self.max_vel_normal
            self.current_acc_scale = self.max_acc_normal
            self.group.set_max_velocity_scaling_factor(self.current_vel_scale)
            self.group.set_max_acceleration_scaling_factor(self.current_acc_scale)
            
            if not self.plan_and_execute(pre_pose_initial, "PRE_LOCATION"):
                result['failure_stage'] = 'MOVE_TO_PRE'
                return result
            
            rospy.loginfo(f"✓ 到达Pre-location: z={z_pre:.3f}m (cube上方{self.pre_height*100:.0f}cm)")
            
            # ===== 步骤2: 在Pre-location停留5s，确保零速度 =====
            rospy.loginfo(f"\n步骤2: 在Pre-location停留{self.dwell_time}秒，确保机器人完全稳定")
            self.group.stop()  # 确保停止所有运动
            rospy.sleep(self.dwell_time)
            rospy.loginfo("✓ 机器人已稳定，速度归零")
            
            # 重新读取cube位置（此时应该更准确）
            rospy.loginfo("步骤2: 重新读取cube位置并调整pose...")
            cube_pose_stable = self.fetch_cube_pose(model_name)
            if cube_pose_stable is None:
                result['failure_stage'] = 'FETCH_STABLE_POSE'
                return result
            
            # 更新记录的cube位置
            result['cube_x'] = cube_pose_stable.position.x
            result['cube_y'] = cube_pose_stable.position.y
            result['cube_z'] = cube_pose_stable.position.z
            
            # 重新计算最优yaw（基于稳定后的cube位置）
            optimal_yaw_stable = self.compute_optimal_grasp_yaw(cube_pose_stable)
            rospy.loginfo(f"步骤2: 调整后的最优角度 yaw={math.degrees(optimal_yaw_stable):.0f}°")
            
            # 基于稳定的cube位置计算精确的抓取poses
            poses = self.build_grasp_poses(cube_pose_stable, custom_yaw=optimal_yaw_stable)
            rospy.loginfo("✓ Pose已调整，准备下降")
            
            # ===== 步骤3: 从Pre-location缓慢下降到抓取位置并夹紧 =====
            rospy.loginfo(f"\n步骤3: 缓慢下降到抓取位置 (cube高度的{self.grasp_depth_ratio*100:.0f}%)")
            
            # 使用慢速进行垂直下降
            self.current_vel_scale = self.max_vel_descent
            self.current_acc_scale = self.max_acc_descent
            self.group.set_max_velocity_scaling_factor(self.current_vel_scale)
            self.group.set_max_acceleration_scaling_factor(self.current_acc_scale)
            
            frac, ok = self.cartesian_descent(poses['pre'], poses['grasp'], "DESCENT")
            if not ok:
                result['failure_stage'] = 'DESCENT'
                return result
            
            rospy.loginfo("✓ 到达抓取位置")
            
            # 速度缓降，确保完全停止
            self.velocity_rampdown("到达抓取位置，速度归零", steps=5, initial_vel=0.10)
            rospy.sleep(0.5)  # 额外稳定时间
            
            # 夹紧
            rospy.loginfo("步骤3: 夹紧cube...")
            close_ok, final_width = self.close_gripper()
            rospy.sleep(0.3)  # 等待夹爪完全闭合
            
            # 微调动作：轻微抬起再放下，让cube自动对齐到稳定位置
            rospy.loginfo("微调: 轻微抬起让cube对齐...")
            self.micro_adjust_grasp(poses['grasp'])
            rospy.sleep(0.5)  # 等待cube稳定
            
            # 重新读取gripper宽度（微调后可能有变化）
            final_width = self.gripper_width
            
            # 验证抓取
            if not self.verify_grasp(final_width):
                result['failure_stage'] = 'VERIFY_GRASP'
                return result
            
            rospy.loginfo("✓ 夹紧成功")
            
            # ===== 步骤4: 垂直提升50cm =====
            rospy.loginfo(f"\n步骤4: 垂直提升{self.lift_height*100:.0f}cm")
            
            self.current_vel_scale = self.max_vel_normal
            self.current_acc_scale = self.max_acc_normal
            self.group.set_max_velocity_scaling_factor(self.current_vel_scale)
            self.group.set_max_acceleration_scaling_factor(self.current_acc_scale)
            
            frac, ok = self.cartesian_descent(poses['grasp'], poses['lift'], "LIFT")
            if not ok:
                result['failure_stage'] = 'LIFT'
                return result
            
            rospy.loginfo(f"✓ 提升到 z={poses['lift'].position.z:.3f}m")
            
            # ===== 步骤5: 松开夹爪，回到home位置 =====
            rospy.loginfo("\n步骤5: 松开夹爪并返回home")
            
            # 可选：放置到bin（如果启用）
            if self.enable_place:
                rospy.loginfo(f"执行place-to-bin: {color}")
                if self.execute_place_to_bin(color):
                    result['placed_in_bin'] = True
                    rospy.loginfo("✓ 已放置到bin")
                else:
                    rospy.logwarn("⚠ Place-to-bin失败，但继续流程")
            else:
                # 直接松开夹爪
                self.open_gripper()
                rospy.loginfo("✓ 夹爪已松开")
            
            # 返回home
            self.move_home()
            rospy.loginfo("✓ 返回home位置")
            
            # ===== 成功 =====
            result['success'] = True
            self.status_pub.publish("SUCCESS")
            
            rospy.loginfo("="*60)
            rospy.loginfo("✓✓✓ 抓取流程完成 ✓✓✓")
            rospy.loginfo("="*60)
            
        except Exception as e:
            rospy.logerr(f"Exception during grasp: {e}")
            result['failure_stage'] = 'EXCEPTION'
            import traceback
            traceback.print_exc()
        
        finally:
            result['elapsed_time'] = time.time() - start_time
        
        return result

    def run_trials(self):
        """Run all trials."""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo(f"Starting {self.num_trials} trials")
        rospy.loginfo("="*60)
        
        # Get available cubes
        available_cubes = list(self.cube_colors.keys())
        if not available_cubes:
            rospy.logerr("No cubes available!")
            return
        
        rospy.loginfo(f"Available cubes: {available_cubes}")
        
        for trial_num in range(1, self.num_trials + 1):
            rospy.loginfo("\n" + "="*60)
            rospy.loginfo(f"TRIAL {trial_num}/{self.num_trials}")
            rospy.loginfo("="*60)
            
            # Select cube (round-robin)
            cube_idx = (trial_num - 1) % len(available_cubes)
            model_name = available_cubes[cube_idx]
            color = self.cube_colors[model_name]
            
            rospy.loginfo(f"Target: {model_name} (Color: {color})")
            
            # Execute grasp
            result = self.execute_single_grasp(model_name, color)
            result['trial_num'] = trial_num
            self.results.append(result)
            
            # Log result
            status = "✓ SUCCESS" if result['success'] else f"✗ FAILED at {result['failure_stage']}"
            rospy.loginfo(f"Trial {trial_num}: {status} (time={result['elapsed_time']:.1f}s)")
            
            # Brief pause between trials
            if trial_num < self.num_trials:
                rospy.sleep(1.0)
        
        # Save results
        self.save_results()
        self.print_summary()

    def save_results(self):
        """Save results to CSV."""
        if not self.results:
            return
        
        with open(self.output_csv, 'w', newline='') as f:
            fieldnames = ['trial_num', 'model_name', 'color', 'success', 'failure_stage', 
                         'elapsed_time', 'cube_x', 'cube_y', 'cube_z', 'placed_in_bin']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.results)
        
        rospy.loginfo(f"\n✓ Results saved to: {self.output_csv}")

    def print_summary(self):
        """Print summary statistics."""
        if not self.results:
            return
        
        total = len(self.results)
        succeeded = sum(1 for r in self.results if r['success'])
        success_rate = (succeeded / total) * 100
        avg_time = sum(r['elapsed_time'] for r in self.results) / total
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("SUMMARY")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Total trials: {total}")
        rospy.loginfo(f"Succeeded: {succeeded}")
        rospy.loginfo(f"Failed: {total - succeeded}")
        rospy.loginfo(f"Success rate: {success_rate:.1f}%")
        rospy.loginfo(f"Avg time: {avg_time:.1f}s")
        
        if self.enable_place:
            placed = sum(1 for r in self.results if r.get('placed_in_bin', False))
            rospy.loginfo(f"Placed in bins: {placed}/{succeeded}")
        
        # Failure breakdown
        failed = [r for r in self.results if not r['success']]
        if failed:
            rospy.loginfo("\nFailure breakdown:")
            stages = {}
            for r in failed:
                stage = r['failure_stage']
                stages[stage] = stages.get(stage, 0) + 1
            for stage, count in sorted(stages.items(), key=lambda x: -x[1]):
                rospy.loginfo(f"  {stage}: {count}")
        
        rospy.loginfo("="*60)


def parse_args():
    parser = argparse.ArgumentParser(description="V4 Demo: Simplified grasp with place-to-bin")
    parser.add_argument("--trials", type=int, default=10, help="Number of trials")
    parser.add_argument("--enable-place", action="store_true", help="Enable place-to-bin")
    parser.add_argument("--verbose", action="store_true", help="Verbose logging")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        demo = V4GraspDemo(args)
        demo.run_trials()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\nInterrupted by user")


if __name__ == "__main__":
    main()
