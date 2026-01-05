#!/usr/bin/env python3
"""
Perception Node: Point cloud processing + object detection + color segmentation
输入: /zed2/zed_node/point_cloud/cloud_registered, /zed2/zed_node/rgb/image_rect_color
输出: /detected_objects, /target_cube_pose

用于Phase 1: 将ZED2的RGB-D数据转换为检测到的目标物体位置
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from scipy import ndimage
from collections import deque
import tf2_ros
import tf2_geometry_msgs

class PerceptionNode:
    """
    完整的感知节点：
    1. 点云预处理（VoxelGrid下采样 + Z轴范围滤波）
    2. RANSAC平面分割（提取非桌面物体）
    3. 点云聚类（DBSCAN找独立物体）
    4. RGB颜色分割（识别物体类别）
    5. 3D质心 + 滑动平均（稳定性）
    """
    
    def __init__(self):
        rospy.init_node('perception_node', log_level=rospy.DEBUG)
        
        # TF2 buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 参数
        self.voxel_size = rospy.get_param('~voxel_size', 0.005)  # 5mm
        self.z_min = rospy.get_param('~z_min', -0.05)
        self.z_max = rospy.get_param('~z_max', 0.3)
        self.ransac_dist_threshold = rospy.get_param('~ransac_dist', 0.01)  # 1cm
        self.dbscan_eps = rospy.get_param('~dbscan_eps', 0.02)  # 2cm
        self.dbscan_min_samples = rospy.get_param('~dbscan_min_samples', 50)
        self.ema_alpha = rospy.get_param('~ema_alpha', 0.3)  # Exponential moving average
        
        # 颜色阈值 (HSV)
        self.color_thresholds = {
            'red': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255]),
                    'lower2': np.array([170, 100, 100]), 'upper2': np.array([180, 255, 255])},
            'blue': {'lower': np.array([100, 100, 100]), 'upper': np.array([130, 255, 255])},
            'green': {'lower': np.array([35, 100, 100]), 'upper': np.array([85, 255, 255])},
            'yellow': {'lower': np.array([20, 100, 100]), 'upper': np.array([35, 255, 255])},
        }
        
        # Subscribers
        self.cloud_sub = rospy.Subscriber(
            '/zed2/zed_node/point_cloud/cloud_registered',
            PointCloud2, self.on_cloud, queue_size=1)
        
        self.rgb_sub = rospy.Subscriber(
            '/zed2/zed_node/rgb/image_rect_color',
            Image, self.on_rgb, queue_size=1)
        
        # Publishers
        self.target_pub = rospy.Publisher(
            '/target_cube_pose', PoseStamped, queue_size=1)
        
        self.debug_pub = rospy.Publisher(
            '/perception/color_mask', Image, queue_size=1)
        
        # State
        self.current_rgb = None
        self.current_depth = None
        self.current_cloud = None
        self.bridge = CvBridge()
        
        # Tracking (EMA smoothing)
        self.object_history = {}  # {object_id: deque of centroids}
        self.max_history_len = 5  # Keep 5 frames
        
        rospy.loginfo("Perception node initialized")
        rospy.loginfo(f"Voxel size: {self.voxel_size}m, Z range: [{self.z_min}, {self.z_max}]m")
    
    def on_rgb(self, msg):
        """Store latest RGB image"""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, 'BGR8')
        except Exception as e:
            rospy.logwarn(f"Failed to convert RGB: {e}")
    
    def on_cloud(self, msg):
        """
        Main processing callback: Extract objects from point cloud
        """
        try:
            # 1. 读取点云
            points = self.read_point_cloud(msg)
            if len(points) < 100:
                rospy.logwarn(f"Too few points: {len(points)}")
                return
            
            rospy.logdebug(f"Read {len(points)} points from cloud")
            
            # 2. 下采样 (VoxelGrid)
            points_downsampled = self.voxel_downsample(points)
            rospy.logdebug(f"After voxel downsampling: {len(points_downsampled)} points")
            
            # 3. Z轴范围滤波
            points_filtered = self.filter_z_range(points_downsampled)
            if len(points_filtered) < 100:
                rospy.logwarn(f"Too few points after Z filter: {len(points_filtered)}")
                return
            
            rospy.logdebug(f"After Z filter: {len(points_filtered)} points")
            
            # 4. RANSAC平面分割（桌面）
            points_objects = self.remove_plane(points_filtered)
            if len(points_objects) < 50:
                rospy.logwarn(f"Too few object points: {len(points_objects)}")
                return
            
            rospy.logdebug(f"After plane removal: {len(points_objects)} points")
            
            # 5. DBSCAN聚类
            clusters = self.cluster_objects(points_objects)
            if len(clusters) == 0:
                rospy.logwarn("No clusters found")
                return
            
            rospy.loginfo(f"Found {len(clusters)} clusters")
            
            # 6. 计算质心
            objects = []
            for cluster_id, cluster_points in enumerate(clusters):
                centroid = cluster_points.mean(axis=0)
                num_points = len(cluster_points)
                
                # 识别颜色
                color = self.detect_color_for_cluster(cluster_points) if self.current_rgb is not None else 'unknown'
                
                obj = {
                    'id': cluster_id,
                    'centroid': centroid,
                    'num_points': num_points,
                    'color': color,
                    'cluster_points': cluster_points,
                }
                objects.append(obj)
            
            rospy.logdebug(f"Objects: {[(o['id'], o['color'], o['num_points']) for o in objects]}")
            
            # 7. EMA平滑
            objects = self.smooth_tracking(objects)
            
            # 8. 选择目标
            target = self.select_target(objects)
            if target is not None:
                self.publish_target(target, msg.header)
        
        except Exception as e:
            rospy.logerr(f"Error in on_cloud: {e}", exc_info=True)
    
    def read_point_cloud(self, msg):
        """Convert ROS PointCloud2 to numpy array"""
        points = list(pc2.read_points(msg, skip_nans=True))
        if len(points) == 0:
            return np.empty((0, 3))
        return np.array(points, dtype=np.float32)[:, :3]
    
    def voxel_downsample(self, points):
        """Voxel grid downsampling"""
        if len(points) == 0:
            return points
        
        # Create voxel grid
        min_bound = points.min(axis=0)
        max_bound = points.max(axis=0)
        grid_size = ((max_bound - min_bound) / self.voxel_size).astype(int) + 1
        
        # 预留内存
        voxel_indices = ((points - min_bound) / self.voxel_size).astype(int)
        voxel_indices = np.clip(voxel_indices, 0, grid_size - 1)
        
        # 计算体素索引的哈希值
        hash_vec = np.array([1, grid_size[0], grid_size[0] * grid_size[1]])
        hashes = voxel_indices @ hash_vec
        
        # 为每个唯一体素取一个点
        unique_hashes, unique_indices = np.unique(hashes, return_index=True)
        downsampled = points[unique_indices]
        
        return downsampled
    
    def filter_z_range(self, points):
        """Z轴范围滤波（只保留桌面附近）"""
        mask = (points[:, 2] > self.z_min) & (points[:, 2] < self.z_max)
        return points[mask]
    
    def remove_plane(self, points):
        """RANSAC平面分割（移除桌面）"""
        if len(points) < 10:
            return points
        
        try:
            # Simple RANSAC: 随机选择3个点拟合平面，找最多inliers的平面
            best_inliers = 0
            best_normal = None
            
            for _ in range(100):  # 100次迭代
                # 随机选3个点
                indices = np.random.choice(len(points), 3, replace=False)
                p0, p1, p2 = points[indices]
                
                # 计算平面法线
                v1 = p1 - p0
                v2 = p2 - p0
                normal = np.cross(v1, v2)
                norm = np.linalg.norm(normal)
                if norm < 1e-6:
                    continue
                normal = normal / norm
                
                # 计算平面方程: normal · (p - p0) = 0
                d = np.dot(normal, p0)
                
                # 计算所有点到平面的距离
                distances = np.abs(np.dot(points, normal) - d)
                inliers = np.sum(distances < self.ransac_dist_threshold)
                
                if inliers > best_inliers:
                    best_inliers = inliers
                    best_normal = normal
            
            # 移除内点（属于平面）
            if best_normal is not None:
                d = np.dot(best_normal, points[0])
                distances = np.abs(np.dot(points, best_normal) - d)
                non_plane_mask = distances >= self.ransac_dist_threshold
                return points[non_plane_mask]
            else:
                return points
        
        except Exception as e:
            rospy.logwarn(f"RANSAC failed: {e}")
            return points
    
    def cluster_objects(self, points):
        """DBSCAN聚类"""
        from sklearn.cluster import DBSCAN
        
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_
        
        clusters = []
        for label in np.unique(labels):
            if label == -1:  # 噪声点
                continue
            cluster_points = points[labels == label]
            if len(cluster_points) >= self.dbscan_min_samples:
                clusters.append(cluster_points)
        
        return clusters
    
    def detect_color_for_cluster(self, cluster_points):
        """
        识别聚类的颜色
        使用RGB投影到图像坐标，采样周围像素
        """
        if self.current_rgb is None:
            return 'unknown'
        
        try:
            # 取聚类的几个代表点投影到图像
            # 假设简单透视投影（实际应使用相机内参）
            centroid = cluster_points.mean(axis=0)
            
            # 伪投影（此处应使用真实的相机内参）
            h, w = self.current_rgb.shape[:2]
            px = int(w / 2 + centroid[0] * 100)  # 简化公式
            py = int(h / 2 - centroid[1] * 100)
            
            if 0 <= px < w and 0 <= py < h:
                # 取3x3邻域的平均颜色
                y_min, y_max = max(0, py - 3), min(h, py + 4)
                x_min, x_max = max(0, px - 3), min(w, px + 4)
                region = self.current_rgb[y_min:y_max, x_min:x_max]
                
                if region.size == 0:
                    return 'unknown'
                
                # 转HSV
                hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
                
                # 匹配颜色
                best_color = 'unknown'
                best_match = 0
                
                for color_name, thresholds in self.color_thresholds.items():
                    if 'lower2' in thresholds:  # 红色特殊处理
                        mask1 = cv2.inRange(hsv, thresholds['lower'], thresholds['upper'])
                        mask2 = cv2.inRange(hsv, thresholds['lower2'], thresholds['upper2'])
                        mask = cv2.bitwise_or(mask1, mask2)
                    else:
                        mask = cv2.inRange(hsv, thresholds['lower'], thresholds['upper'])
                    
                    match_ratio = np.sum(mask > 0) / (mask.size + 1e-6)
                    if match_ratio > best_match:
                        best_match = match_ratio
                        best_color = color_name
                
                # 仅当匹配度>30%时认为识别成功
                if best_match > 0.3:
                    return best_color
            
            return 'unknown'
        
        except Exception as e:
            rospy.logwarn(f"Color detection failed: {e}")
            return 'unknown'
    
    def smooth_tracking(self, objects):
        """EMA平滑跟踪"""
        smoothed_objects = []
        
        for obj in objects:
            obj_id = obj['id']
            centroid = obj['centroid']
            
            if obj_id not in self.object_history:
                self.object_history[obj_id] = deque(maxlen=self.max_history_len)
            
            self.object_history[obj_id].append(centroid)
            
            # 计算EMA
            if len(self.object_history[obj_id]) > 0:
                smoothed_centroid = np.zeros(3)
                total_weight = 0
                
                for i, prev_centroid in enumerate(self.object_history[obj_id]):
                    weight = self.ema_alpha ** (len(self.object_history[obj_id]) - i - 1)
                    smoothed_centroid += weight * prev_centroid
                    total_weight += weight
                
                smoothed_centroid /= total_weight
                obj['centroid'] = smoothed_centroid
            
            smoothed_objects.append(obj)
        
        return smoothed_objects
    
    def select_target(self, objects):
        """
        选择目标物体：离相机最近的物体
        可扩展为其他策略（e.g., 特定颜色）
        """
        if len(objects) == 0:
            return None
        
        # 策略: 离相机(z轴)最近
        target = min(objects, key=lambda o: o['centroid'][2])
        rospy.logdebug(f"Selected target: ID={target['id']}, color={target['color']}, "
                      f"centroid={target['centroid']}")
        return target
    
    def publish_target(self, target, header):
        """发布目标位置为PoseStamped (转换到panda_link0坐标系)"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = 'camera_link'  # 原始点云的参考系
        
        pos = target['centroid']
        pose_msg.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        pose_msg.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)  # 无方向
        
        # 转换到panda_link0坐标系
        try:
            # 等待TF可用
            if not self.tf_buffer.can_transform('panda_link0', 'camera_link', rospy.Time(0), timeout=rospy.Duration(1.0)):
                rospy.logwarn("Cannot transform from camera_link to panda_link0, using camera_link frame")
            else:
                # 执行坐标系转换
                pose_msg = self.tf_buffer.transform(pose_msg, 'panda_link0', timeout=rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn(f"Transform failed: {e}, using camera_link frame")
        
        self.target_pub.publish(pose_msg)
        rospy.loginfo(f"Published target: {target['color']} at ({pose_msg.pose.position.x:.3f}, {pose_msg.pose.position.y:.3f}, {pose_msg.pose.position.z:.3f}) in {pose_msg.header.frame_id}")


if __name__ == '__main__':
    try:
        node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
