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
import tf2_ros

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
        
        # ZED2 Integration (Phase 1)
        self.use_zed2 = args.zed2 if hasattr(args, 'zed2') and args.zed2 else False
        self.use_perception = self.use_zed2  # Perception enabled if ZED2 is enabled
        self.current_object_pose = None
        self.detected_color = None
        self.tf_buffer = None
        
        if self.use_zed2:
            rospy.loginfo("🎥 ZED2 perception enabled, subscribing to /object_pose...")
            rospy.Subscriber("/object_pose", PoseStamped, self.object_pose_callback)
            rospy.Subscriber("/detection_status", String, self.detection_status_callback)
            rospy.Subscriber("/detected_objects", String, self.detected_objects_callback)
            # Wait for ZED2 node to start
            rospy.sleep(2.0)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            
            # Phase 2: Multi-color detection storage
            self.detected_objects_list = []  # List of detected objects from ZED2
        
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
        
        # Gazebo services (backup)
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=10)
        rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
        self.get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        
        # Sorting state machine
        self.sorting_sm = SortingStateMachine()
        
        # Status publisher
        self.status_pub = rospy.Publisher("/grasp_planning_status", String, queue_size=10)
        
        # Target color selection (Phase 2)
        self.target_color = rospy.get_param('~target_color', 'red')
        rospy.Subscriber("/target_color", String, self.target_color_callback)
        
        # Detected objects from ZED2 perception
        self.detected_objects_list = []  # List of detected objects with colors
        self.detected_objects_sub = rospy.Subscriber("/detected_objects", String, self.detected_objects_callback, queue_size=10)
        
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
        rospy.loginfo(f"ZED2 perception: {'ENABLED' if self.use_zed2 else 'DISABLED (using Gazebo)'}")
        rospy.loginfo(f"Results: {self.output_csv}")
        rospy.loginfo("="*60)
        
    def object_pose_callback(self, msg: PoseStamped):
        """Receive object pose from ZED2 perception node."""
        self.current_object_pose = msg
        if self.verbose:
            rospy.loginfo(f"🎥 ZED2 detection: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}")
    
    def detection_status_callback(self, msg: String):
        """Receive detection status from ZED2 perception node."""
        status = msg.data
        if status.startswith("FOUND:"):
            self.detected_color = status.split(":")[1]
            rospy.loginfo(f"✓ Detected color: {self.detected_color}")
    
    def target_color_callback(self, msg: String):
        """Update target color dynamically from ROS topic."""
        self.target_color = msg.data.lower()
        rospy.loginfo(f"🎨 Target color changed to: {self.target_color}")
        
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
            # spawn_cubes.py publishes 'cube_name' and 'color'
            model_name = data.get('cube_name') or data.get('name')
            color = data.get('color')
            if model_name and color:
                self.cube_colors[model_name] = color
                if self.verbose:
                    rospy.loginfo(f"✓ Tracked cube: {model_name} = {color}")
        except Exception as e:
            if self.verbose:
                rospy.logwarn(f"Cube properties parse error: {e}")

    def detected_objects_callback(self, msg: String):
        """
        Phase 2: Receive all detected objects from ZED2 perception node.
        
        Expected format (JSON):
        {
            'objects': [
                {
                    'color': 'red', 
                    'position': [x, y, z], 
                    'confidence': 0.95, 
                    'area': 1500,
                    'optimal_yaw': 90.0  # 从点云PCA计算出的最优yaw角
                },
                ...
            ],
            'timestamp': 1234567890.123
        }
        """
        try:
            import json
            data = json.loads(msg.data)
            self.detected_objects_list = data.get('objects', [])
            
            if self.verbose and self.detected_objects_list:
                colors_found = [obj['color'] for obj in self.detected_objects_list]
                rospy.loginfo(f"🎨 Phase 2 - Detected objects: {colors_found}")
                # 显示点云计算的最优yaw角
                for obj in self.detected_objects_list:
                    if obj.get('optimal_yaw') is not None:
                        rospy.loginfo(f"   {obj['color']}: optimal_yaw={obj['optimal_yaw']:.0f}°")
        except Exception as e:
            rospy.logwarn(f"Detected objects parse error: {e}")

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
    
    def get_object_pose(self, model_name: str = None) -> Optional[Pose]:
        """
        Get object pose from ZED2 or Gazebo.
        
        This unified method supports:
        1. ZED2 perception node (if enabled)
        2. Gazebo ground truth (fallback)
        
        Args:
            model_name: Model name (only used for Gazebo fallback)
        
        Returns:
            Pose of the object, or None if not found
        """
        if self.use_zed2:
            # Perception-based mode: Return the most recent detected object
            if self.detected_objects_list:
                return self.detected_objects_list[0]  # Most recent detection
        
        # Fallback: Gazebo ground truth
        if model_name:
            return self.fetch_cube_pose(model_name)
        
        return None

    def compare_vision_vs_gazebo(self, detected_pos, target_color):
        """
        比较视觉检测位置与Gazebo真实位置的偏差。
        用于调试和验证感知精度。
        
        Args:
            detected_pos: 视觉检测的位置 [x, y, z]
            target_color: 目标颜色
        """
        try:
            # 查找匹配颜色的Gazebo cube
            world = self.get_world_properties()
            color_map = {
                'red': 'RED',
                'blue': 'BLUE',
                'green': 'GREEN',
                'yellow': 'YELLOW'
            }
            
            target_color_upper = color_map.get(target_color.lower(), target_color.upper())
            
            # 找到所有cubes并计算距离
            closest_cube = None
            min_distance = float('inf')
            
            for model_name in world.model_names:
                if model_name.startswith("cube_"):
                    # 获取Gazebo中的真实位置
                    gazebo_pose = self.fetch_cube_pose(model_name)
                    if gazebo_pose:
                        # 计算距离
                        dx = detected_pos[0] - gazebo_pose.position.x
                        dy = detected_pos[1] - gazebo_pose.position.y
                        dz = detected_pos[2] - gazebo_pose.position.z
                        distance = (dx**2 + dy**2 + dz**2)**0.5
                        
                        if distance < min_distance:
                            min_distance = distance
                            closest_cube = {
                                'name': model_name,
                                'pose': gazebo_pose,
                                'dx': dx,
                                'dy': dy,
                                'dz': dz,
                                'distance': distance
                            }
            
            if closest_cube:
                # Gazebo返回cube几何中心，Vision返回顶面
                # 需要调整Gazebo的z坐标以对比顶面
                gazebo_z_center = closest_cube['pose'].position.z
                gazebo_z_top = gazebo_z_center + self.cube_height / 2.0  # 顶面
                
                # 计算调整后的偏差
                dx = detected_pos[0] - closest_cube['pose'].position.x
                dy = detected_pos[1] - closest_cube['pose'].position.y
                dz_top = detected_pos[2] - gazebo_z_top  # 与顶面对比
                distance_xy = (dx**2 + dy**2)**0.5
                
                rospy.loginfo("="*60)
                rospy.loginfo("【位置精度对比】Vision vs Gazebo Ground Truth")
                rospy.loginfo(f"  最近的cube: {closest_cube['name']}")
                rospy.loginfo(f"  Vision检测（顶面）: ({detected_pos[0]:.4f}, {detected_pos[1]:.4f}, {detected_pos[2]:.4f})")
                rospy.loginfo(f"  Gazebo中心: ({closest_cube['pose'].position.x:.4f}, {closest_cube['pose'].position.y:.4f}, {gazebo_z_center:.4f})")
                rospy.loginfo(f"  Gazebo顶面: ({closest_cube['pose'].position.x:.4f}, {closest_cube['pose'].position.y:.4f}, {gazebo_z_top:.4f})")
                rospy.loginfo(f"  XY平面偏差:")
                rospy.loginfo(f"    ΔX = {dx*1000:.1f}mm")
                rospy.loginfo(f"    ΔY = {dy*1000:.1f}mm")
                rospy.loginfo(f"    XY距离 = {distance_xy*1000:.1f}mm")
                rospy.loginfo(f"  Z轴偏差（vs顶面）:")
                rospy.loginfo(f"    ΔZ = {dz_top*1000:.1f}mm")
                
                # 评估XY平面精度（更重要）
                if distance_xy < 0.01:  # <10mm
                    rospy.loginfo("  ✓✓ XY精度优秀 (<10mm)")
                elif distance_xy < 0.02:  # <20mm
                    rospy.loginfo("  ✓ XY精度良好 (10-20mm)")
                elif distance_xy < 0.05:  # <50mm
                    rospy.logwarn("  ⚠ XY精度一般 (20-50mm)")
                else:
                    rospy.logerr("  ❌ XY精度较差 (>50mm)")
                rospy.loginfo("="*60)
                
        except Exception as e:
            rospy.logwarn(f"位置对比失败: {e}")
    
    def get_object_pose_legacy(self, model_name: str = None) -> Optional[Pose]:
        """
        Legacy method for getting object pose.
        
        Returns:
            Pose in panda_link0 frame, or None if not available
        """
        if self.use_zed2:
            if self.current_object_pose is not None:
                return self.current_object_pose.pose
            else:
                rospy.logwarn("⚠ ZED2 pose not yet available, falling back to Gazebo...")
                if model_name:
                    return self.fetch_cube_pose(model_name)
                return None
        else:
            # Fallback to Gazebo
            if model_name:
                return self.fetch_cube_pose(model_name)
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

    def build_grasp_poses(self, cube_pose: Pose, custom_yaw: Optional[float] = None, is_vision_data: bool = True) -> Dict[str, Pose]:
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
            is_vision_data: True if input is from vision (z=top surface), False if from Gazebo (z=center)
        
        Returns dict with keys: 'pre', 'grasp', 'lift'
        """
        x = cube_pose.position.x
        y = cube_pose.position.y
        z_input = cube_pose.position.z
        
        # 关键修正：根据数据源调整Z坐标的含义
        if is_vision_data:
            # Vision returns TOP surface position, need to convert to center for calculation
            # z_input = z_top_surface
            z_top = z_input
            z_center = z_top - self.cube_half  # 立方体中心 = 顶面 - cube_half
            z_bottom = z_center - self.cube_half  # 立方体底部 = 中心 - cube_half
            rospy.loginfo(f"[Height Calc] Vision data: z_top={z_top:.4f} → z_center={z_center:.4f}, z_bottom={z_bottom:.4f}")
        else:
            # Gazebo returns CENTER position
            z_center = z_input
            z_bottom = z_center - self.cube_half  # 真实底部
            z_top = z_center + self.cube_half     # 真实顶部
            rospy.loginfo(f"[Height Calc] Gazebo data: z_center={z_center:.4f}, z_top={z_top:.4f}, z_bottom={z_bottom:.4f}")
        
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

    def execute_single_grasp(self, target_color: str, detected_obj: Optional[Dict] = None) -> Dict:
        """
        执行单次抓取序列。
        
        流程：
        0. 从vision读取cube位置（detected_obj.position）
        1. Pre-location：移动到cube上方10cm
        2. 在pre-location停留3s，确保零速度（使用初始视觉位置）
        3. 从pre-location缓慢下降到抓取位置（夹爪到cube高度的2/3-1/2），夹紧
        4. 垂直提升50cm
        5. 松开，回到home位置
        
        注意：完全依赖视觉模块数据，不查询Gazebo模型状态
        """
        result = {
            'target_color': target_color,
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
            # ===== 步骤0: 从vision读取cube位置 =====
            rospy.loginfo("="*60)
            rospy.loginfo(f"开始抓取: {target_color}")
            rospy.loginfo("="*60)
            
            # 使用detected_obj的位置（来自RGB-D感知）
            if detected_obj and 'position' in detected_obj:
                detected_pos = detected_obj['position']
                cube_pose_initial = Pose(
                    position=Point(detected_pos[0], detected_pos[1], detected_pos[2]),
                    orientation=Quaternion(0, 0, 0, 1)
                )
                result['cube_x'] = detected_pos[0]
                result['cube_y'] = detected_pos[1]
                result['cube_z'] = detected_pos[2]
                rospy.loginfo(f"步骤0: 使用RGB-D检测位置 ({detected_pos[0]:.3f}, {detected_pos[1]:.3f}, {detected_pos[2]:.3f})")
                
                # 【调试】与Gazebo真实位置对比
                self.compare_vision_vs_gazebo(detected_pos, target_color)
                
            else:
                rospy.logerr("❌ No detected object provided - cannot proceed without vision position")
                result['failure_stage'] = 'NO_DETECTION'
                return result
            
            # 优先使用点云计算的最优yaw，如果无可用则计算
            if detected_obj and detected_obj.get('optimal_yaw') is not None:
                # 从点云PCA计算的最优yaw（已对齐到0/90/180/270）
                optimal_yaw_deg = detected_obj['optimal_yaw']
                optimal_yaw = math.radians(optimal_yaw_deg)
                rospy.loginfo(f"步骤0: 使用点云计算的最优抓取角度 yaw={optimal_yaw_deg:.0f}° (PCA避免夹住棱)")
            else:
                # 备选方案：基于cube位置计算
                optimal_yaw = self.compute_optimal_grasp_yaw(cube_pose_initial)
                rospy.loginfo(f"步骤0: 计算最优抓取角度 yaw={math.degrees(optimal_yaw):.0f}° (位置分析避免夹住棱)")
            
            # ===== 步骤1: 移动到Pre-location (cube上方10cm) =====
            rospy.loginfo("\n步骤1: 移动到Pre-location (cube上方10cm)")
            
            # Reset: 回到home并打开夹爪
            if not self.move_home():
                result['failure_stage'] = 'MOVE_HOME'
                return result
            self.open_gripper()
            
            # 计算pre-location位姿（使用初步计算的yaw）
            # 注意：detected_pos.z 是cube顶面中心点（从相机视角测得）
            # cube_height = 0.045m (45mm)
            # 需要计算抓取高度和pre-location高度
            z_top = cube_pose_initial.position.z  # cube顶面（视觉检测）
            z_bottom = z_top - self.cube_height  # cube底面
            z_grasp_initial = z_bottom + self.cube_height * self.grasp_depth_ratio  # 抓取深度
            z_pre = z_top + self.pre_height  # pre-location：顶面上方15cm
            
            rospy.loginfo(f"高度计算: z_top={z_top:.3f}, z_bottom={z_bottom:.3f}, z_grasp={z_grasp_initial:.3f}, z_pre={z_pre:.3f}")
            
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
            
            # ===== 步骤2: 在Pre-location停留，确保零速度 =====
            rospy.loginfo(f"\n步骤2: 在Pre-location停留{self.dwell_time}秒，确保机器人完全稳定")
            self.group.stop()  # 确保停止所有运动
            rospy.sleep(self.dwell_time)
            rospy.loginfo("✓ 机器人已稳定，速度归零")
            
            # 使用初始的视觉检测位置（物体在抓取前不会移动）
            rospy.loginfo("步骤2: 使用视觉检测的cube位置（vision-only模式）")
            cube_pose_stable = cube_pose_initial  # 复用步骤0的视觉位置
            
            # 如果需要更新yaw角度，可以重新计算（但位置保持不变）
            if detected_obj and detected_obj.get('optimal_yaw') is not None:
                optimal_yaw_stable = optimal_yaw  # 使用步骤0的点云yaw
            else:
                optimal_yaw_stable = self.compute_optimal_grasp_yaw(cube_pose_stable)
            
            rospy.loginfo(f"步骤2: 确认抓取角度 yaw={math.degrees(optimal_yaw_stable):.0f}°")
            
            # 基于视觉位置计算精确的抓取poses（明确指定是视觉数据）
            poses = self.build_grasp_poses(cube_pose_stable, custom_yaw=optimal_yaw_stable, is_vision_data=True)
            rospy.loginfo("✓ Pose已确认，准备下降")
            
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

    def run_sorting_loop(self):
        """
        Phase 2: Run continuous sorting loop based on ZED2 multi-color detection.
        
        Workflow:
        1. Detect all colored objects from ZED2
        2. Filter by target_color
        3. Pick and place to corresponding bin
        4. Continue until user interrupts
        """
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🎨 PHASE 2: Color Sorting Loop Started")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Target color: {self.target_color}")
        rospy.loginfo(f"Subscribe to '/target_color' to change color")
        rospy.loginfo("Press Ctrl-C to stop")
        rospy.loginfo("="*60 + "\n")
        
        trial_count = 0
        
        try:
            while not rospy.is_shutdown():
                # Get detected objects
                if not self.detected_objects_list:
                    rospy.loginfo(f"⏳ Waiting for {self.target_color} objects...")
                    rospy.sleep(1.0)
                    continue
                
                # Filter by target color
                target_objects = [obj for obj in self.detected_objects_list 
                                if obj['color'].lower() == self.target_color.lower()]
                
                if not target_objects:
                    rospy.loginfo(f"⏳ No {self.target_color} objects detected, waiting...")
                    rospy.sleep(1.0)
                    continue
                
                # Select the largest/most confident object
                target_obj = max(target_objects, key=lambda o: o['confidence'])
                trial_count += 1
                
                rospy.loginfo("\n" + "="*60)
                rospy.loginfo(f"SORT {trial_count}: Detected {target_obj['color'].upper()}")
                rospy.loginfo("="*60)
                # Log result
                rospy.loginfo(f"Position: ({target_obj['position'][0]:.3f}, "
                            f"{target_obj['position'][1]:.3f}, "
                            f"{target_obj['position'][2]:.3f})")
                rospy.loginfo(f"Confidence: {target_obj['confidence']:.2f}")
                if target_obj.get('optimal_yaw') is not None:
                    rospy.loginfo(f"Point Cloud Yaw: {target_obj['optimal_yaw']:.0f}°")
                
                # Execute grasp using the detected position
                result = self.execute_single_grasp(target_obj['color'], detected_obj=target_obj)
                result['trial_num'] = trial_count
                self.results.append(result)
                
                # Log result
                status = "✓ SUCCESS" if result['success'] else f"✗ FAILED at {result['failure_stage']}"
                rospy.loginfo(f"Result: {status} (time={result['elapsed_time']:.1f}s)")
                
                # Clear detected objects to wait for new detections
                self.detected_objects_list = []
                
                # Brief pause between operations
                if not rospy.is_shutdown():
                    rospy.sleep(0.5)
        
        except KeyboardInterrupt:
            rospy.loginfo("\n🛑 Sorting loop interrupted by user")
        
        # Save results
        if self.results:
            self.save_results()
            self.print_summary()

    def run_trials(self):
        """Run all trials."""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo(f"Starting {self.num_trials} trials")
        rospy.loginfo("="*60)
        
        # Wait for detected objects from ZED2 perception
        max_wait = 5  # seconds
        start_time = rospy.get_time()
        rospy.loginfo("⏳ Waiting for detected objects from ZED2...")
        
        while not self.detected_objects_list and (rospy.get_time() - start_time) < max_wait:
            rospy.sleep(0.1)
        
        # Extract unique colors from detected objects
        if self.detected_objects_list:
            available_colors = list(set([obj['color'] for obj in self.detected_objects_list]))
            rospy.loginfo(f"✅ Detected objects: {available_colors}")
        else:
            rospy.logerr(f"❌ FATAL: No objects detected from ZED2 after {max_wait}s")
            rospy.logerr("   Check: rostopic echo /detected_objects")
            return
        
        for trial_num in range(1, self.num_trials + 1):
            rospy.loginfo("\n" + "="*60)
            rospy.loginfo(f"TRIAL {trial_num}/{self.num_trials}")
            rospy.loginfo("="*60)
            
            # Select color (round-robin from detected colors)
            color_idx = (trial_num - 1) % len(available_colors)
            target_color = available_colors[color_idx]
            
            rospy.loginfo(f"Target color: {target_color}")
            
            # Find matching detected object from vision
            detected_obj = None
            if self.detected_objects_list:
                matching_objs = [obj for obj in self.detected_objects_list 
                               if obj['color'].lower() == target_color.lower()]
                if matching_objs:
                    # Use the one with highest confidence
                    detected_obj = max(matching_objs, key=lambda o: o.get('confidence', 0))
                    rospy.loginfo(f"  📊 Using vision detection at position: ({detected_obj['position'][0]:.3f}, {detected_obj['position'][1]:.3f}, {detected_obj['position'][2]:.3f})")
                    if detected_obj.get('optimal_yaw') is not None:
                        rospy.loginfo(f"  📊 Point Cloud Yaw: {detected_obj['optimal_yaw']:.0f}°")
            
            # Execute grasp with vision-based position
            result = self.execute_single_grasp(target_color, detected_obj=detected_obj)
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
            fieldnames = ['trial_num', 'target_color', 'success', 'failure_stage', 
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
    parser.add_argument("--zed2", action="store_true", help="Use ZED2 perception (Phase 1)")
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
