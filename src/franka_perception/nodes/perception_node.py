#!/usr/bin/env python3
"""
Perception Node for Franka Robot
=================================

Features:
- ZED2 camera RGB-D processing with Open3D
- Object detection via color segmentation
- Point cloud processing (voxel filtering, RANSAC plane removal, DBSCAN clustering)
- 6D pose estimation using ICP registration
- Support for both simulation and real robot

Input Topics:
- /zed2/zed_node/point_cloud/cloud_registered (sensor_msgs/PointCloud2)
- /zed2/zed_node/rgb/image_rect_color (sensor_msgs/Image)
- /target_color (std_msgs/String) - Dynamic color selection

Output Topics:
- /detected_objects (std_msgs/String) - JSON list of all detected objects
- /object_pose (geometry_msgs/PoseStamped) - Target object pose
- /detection_status (std_msgs/String) - Detection status messages
- /segmented_pc (sensor_msgs/PointCloud2) - Segmented point cloud visualization
"""

import json
import rospy
import numpy as np
import cv2
from collections import deque
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
import open3d as o3d
import matplotlib.pyplot as plt
from pc_helper import *


class PerceptionNode:
    """
    Main perception node for object detection and pose estimation using Open3D.
    """
    
    def __init__(self):
        rospy.init_node('perception_node', log_level=rospy.INFO)
        
        # TF2 for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Parameters
        self.sim_mode = rospy.get_param('~sim_mode', True)
        self.target_color = rospy.get_param('~target_color', 'red')
        self.enable_color_detection = rospy.get_param('~enable_color_detection', False)
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.3)
        self.detection_rate = rospy.get_param('~detection_rate', 10.0)
        self.use_icp = rospy.get_param('~use_icp', True)
        self.world_frame = rospy.get_param('~world_frame', 'world')
        
        # Cube parameters
        self.cube_edge_len = rospy.get_param('~cube_edge_len', 0.045)  # 4.5cm
        self.cube_diagonal = rospy.get_param('~cube_diagonal', 0.0389)  # Diagonal distance
        self.max_cubes = rospy.get_param('~max_cubes', 30)
        
        # Create cube ground truth for ICP
        self.cube_gt = create_cube_gt(self.cube_edge_len)
        
        # Point cloud processing parameters
        self.voxel_size = rospy.get_param('~voxel_size', 0.005)  # 5mm
        self.dbscan_eps = rospy.get_param('~dbscan_eps', 0.03)  # 3cm
        self.dbscan_min_points = rospy.get_param('~dbscan_min_points', 40)
        self.ransac_dist_threshold = rospy.get_param('~ransac_dist', 0.02)  # 2cm
        self.icp_min_points = rospy.get_param('~icp_min_points', 50)
        
        # Workspace bounds (world frame)
        self.boundX = rospy.get_param('~boundX', [0.0, 1.0])
        self.boundY = rospy.get_param('~boundY', [-0.5, 0.5])
        self.boundZ = rospy.get_param('~boundZ', [-0.1, 0.3])
        
        
        # Subscribers
        cloud_topic = rospy.get_param('~cloud_topic', '/zed2/zed_node/point_cloud/cloud_registered')
        rgb_topic = rospy.get_param('~rgb_topic', '/zed2/zed_node/left/image_rect_color')
        
        rospy.loginfo(f"Subscribing to point cloud: {cloud_topic}")
        rospy.loginfo(f"Subscribing to RGB image: {rgb_topic}")
        
        self.cloud_sub = rospy.Subscriber(
            cloud_topic, PointCloud2, self.on_cloud, queue_size=1)
        self.rgb_sub = rospy.Subscriber(
            rgb_topic, Image, self.on_rgb, queue_size=1)
        self.color_sub = rospy.Subscriber(
            '/target_color', String, self.on_target_color, queue_size=1)
        
        # Publishers
        self.detected_objects_pub = rospy.Publisher(
            '/detected_objects', String, queue_size=1)
        self.object_pose_pub = rospy.Publisher(
            '/object_pose', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher(
            '/detection_status', String, queue_size=1)
        self.segmented_pc_pub = rospy.Publisher(
            '/segmented_pc', PointCloud2, queue_size=1)
        self.num_cubes_pub = rospy.Publisher(
            '/num_cubes', String, queue_size=1)
        
        # Create publishers for each cube
        self.cube_publishers = []
        for i in range(self.max_cubes):
            pub = rospy.Publisher(f'cube_{i}_odom_pc', Odometry, queue_size=1)
            self.cube_publishers.append(pub)
        
        # State
        self.current_rgb = None
        self.bridge = CvBridge()
        
        # Rate limiting
        self.last_detection_time = rospy.Time.now()
        self.detection_period = rospy.Duration(1.0 / self.detection_rate)
        
        rospy.loginfo("="*60)
        rospy.loginfo("Perception Node Initialized (Open3D)")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Mode: {'SIMULATION' if self.sim_mode else 'REAL ROBOT'}")
        rospy.loginfo(f"Use ICP: {self.use_icp}")
        rospy.loginfo(f"Color Detection: {'ENABLED' if self.enable_color_detection else 'DISABLED'}")
        if self.enable_color_detection:
            rospy.loginfo(f"  Target Color: {self.target_color}")
        rospy.loginfo(f"Detection Rate: {self.detection_rate} Hz")
        rospy.loginfo(f"Voxel Size: {self.voxel_size}m")
        rospy.loginfo(f"Workspace bounds: X{self.boundX}, Y{self.boundY}, Z{self.boundZ}")
        rospy.loginfo("="*60)
    
    def on_target_color(self, msg):
        """Update target color dynamically."""
        self.target_color = msg.data.lower()
        rospy.loginfo(f"Target color changed to: {self.target_color}")
    
    def on_rgb(self, msg):
        """Store latest RGB image."""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn(f"Failed to convert RGB: {e}")
    
    def publish_odometry(self, pos, rot, frame_id, index):
        """
        Publish the odometry of a detected cube
        Args:
            pos (list): position [x, y, z]
            rot (list): orientation quaternion [x, y, z, w]
            frame_id (string): frame id of the odometry
            index (int): index of the cube
        """
        if index >= len(self.cube_publishers):
            return
        
        cube_odom = Odometry()
        cube_odom.header.stamp = rospy.Time.now()
        cube_odom.header.frame_id = frame_id
        cube_odom.child_frame_id = f"cube_{index}"
        cube_odom.pose.pose.position.x = pos[0]
        cube_odom.pose.pose.position.y = pos[1]
        cube_odom.pose.pose.position.z = pos[2]
        cube_odom.pose.pose.orientation.x = rot[0]
        cube_odom.pose.pose.orientation.y = rot[1]
        cube_odom.pose.pose.orientation.z = rot[2]
        cube_odom.pose.pose.orientation.w = rot[3]
        
        self.cube_publishers[index].publish(cube_odom)
    
    def on_cloud(self, msg):
        """
        Main processing callback: Extract objects from point cloud using Open3D.
        Pipeline:
        1. Transform point cloud to world frame
        2. Voxel downsampling
        3. Crop to workspace bounds
        4. RANSAC plane segmentation (remove table)
        5. DBSCAN clustering
        6. ICP registration for 6D pose estimation
        7. Color detection and publishing
        """
        # Rate limiting
        now = rospy.Time.now()
        if (now - self.last_detection_time) < self.detection_period:
            return
        self.last_detection_time = now
        
        try:
            # 1. Transform point cloud to world frame using Open3D
            o3d_pc = transform_pointcloud(msg, self.world_frame)
            if o3d_pc is None or len(np.asarray(o3d_pc.points)) < 100:
                rospy.loginfo_throttle(5.0, "Failed to transform point cloud or too few points")
                return
            
            rospy.loginfo_throttle(10.0, f"Processing {len(np.asarray(o3d_pc.points))} points")
            
            # 2. Voxel downsampling
            downpcd = o3d_pc.voxel_down_sample(self.voxel_size)
            rospy.loginfo_throttle(10.0, f"After voxel downsampling: {len(np.asarray(downpcd.points))} points")
            
            # 3. Crop to workspace bounds
            cropped_pc = crop_pointcloud(downpcd, self.boundX, self.boundY, self.boundZ)
            if len(np.asarray(cropped_pc.points)) < 100:
                rospy.loginfo_throttle(5.0, f"Too few points after cropping: {len(np.asarray(cropped_pc.points))}")
                return
            
            rospy.loginfo_throttle(10.0, f"After cropping: {len(np.asarray(cropped_pc.points))} points")
            
            # 4. RANSAC plane segmentation (remove table if Z starts below 0)
            if self.boundZ[0] <= 0:
                outlier_cloud = segment_pc(cropped_pc, self.ransac_dist_threshold)
                if outlier_cloud.is_empty():
                    rospy.loginfo_throttle(5.0, "No objects after plane removal")
                    return
            else:
                outlier_cloud = cropped_pc
            
            rospy.loginfo_throttle(10.0, f"After RANSAC: {len(np.asarray(outlier_cloud.points))} object points")
            
            # 5. DBSCAN clustering
            labels = cluster_pc(outlier_cloud, self.dbscan_eps, self.dbscan_min_points)
            max_label = labels.max()
            
            if max_label < 0:
                rospy.loginfo_throttle(5.0, "No clusters found")
                return
            
            rospy.loginfo_throttle(10.0, f"Found {max_label + 1} clusters")
            
            # 6. Process each cluster
            cube_count = 0
            objects = []
            all_points = np.asarray(outlier_cloud.points)
            
            for i in range(max_label + 1):
                cluster_indices = np.where(labels == i)[0]
                cluster = outlier_cloud.select_by_index(cluster_indices)
                cluster_points = np.asarray(cluster.points)
                
                rospy.logdebug(f"Cluster {i}: {len(cluster_points)} points")
                
                # Process cluster with ICP
                if self.use_icp:
                    detected_cubes, cube_count = self.process_cluster_with_icp(
                        cluster, cube_count, msg.header
                    )
                    objects.extend(detected_cubes)
                else:
                    # Simple centroid-based detection
                    centroid = cluster_points.mean(axis=0)
                    
                    # Detect color
                    color, confidence = color_detection_from_points(
                        cluster_points, self.current_rgb
                    )
                    
                    obj = {
                        'id': cube_count,
                        'color': color,
                        'position': centroid.tolist(),
                        'confidence': confidence,
                        'num_points': len(cluster_points)
                    }
                    objects.append(obj)
                    
                    # Publish odometry
                    quat = [0, 0, 0, 1]  # No rotation estimation without ICP
                    self.publish_odometry(centroid.tolist(), quat, self.world_frame, cube_count)
                    cube_count += 1
            
            # 7. Publish results
            self.num_cubes_pub.publish(str(cube_count))
            
            # Publish detected objects as JSON
            self.publish_detected_objects(objects, msg.header)
            
            # Visualize segmented point cloud
            colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
            colors[labels < 0] = 0
            outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
            pcd_ros = o3dpc_to_rospc(outlier_cloud, msg.header, msg.fields, frame_id=self.world_frame)
            self.segmented_pc_pub.publish(pcd_ros)
            
            # Publish detection status
            if cube_count > 0:
                self.status_pub.publish(f"FOUND:{cube_count}_objects")
            else:
                self.status_pub.publish("NO_TARGET")
        
        except Exception as e:
            rospy.logerr(f"Error in on_cloud: {e}", exc_info=True)
            self.status_pub.publish(f"ERROR:{str(e)}")
    
    def process_cluster_with_icp(self, cluster, cube_count, header):
        """
        Process a cluster using ICP registration to detect individual cubes.
        Args:
            cluster (open3d.geometry.PointCloud): cluster point cloud
            cube_count (int): current cube count
            header: ROS header
        Returns:
            detected_cubes (list): list of detected cube objects
            cube_count (int): updated cube count
        """
        max_iter = 10
        iteration = 0
        detected_cubes = []
        
        while len(np.asarray(cluster.points)) > self.icp_min_points and iteration < max_iter:
            iteration += 1
            
            # Perform ICP registration
            try:
                # Initial transformation guess
                init_transform = np.array([
                    [1, 0, 0, 0.5],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.8],
                    [0, 0, 0, 1]
                ])
                
                reg_p2p = o3d.pipelines.registration.registration_icp(
                    self.cube_gt, cluster, 1.0, init_transform,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint()
                )
                
                # Extract position and orientation from transformation matrix
                raw_pos = [
                    reg_p2p.transformation[0, 3],
                    reg_p2p.transformation[1, 3],
                    reg_p2p.transformation[2, 3]
                ]
                
                # Transform from O3D frame to world frame
                pos = [
                    reg_p2p.transformation[1, 3],
                    -reg_p2p.transformation[0, 3],
                    reg_p2p.transformation[2, 3]
                ]
                
                # Extract rotation matrix
                rotation = np.array([
                    [reg_p2p.transformation[0, 0], -reg_p2p.transformation[1, 0], reg_p2p.transformation[2, 0]],
                    [reg_p2p.transformation[0, 1], -reg_p2p.transformation[1, 1], reg_p2p.transformation[2, 1]],
                    [reg_p2p.transformation[0, 2], -reg_p2p.transformation[1, 2], reg_p2p.transformation[2, 2]]
                ])
                
                # Convert rotation matrix to quaternion
                quat = rotation_matrix_to_quaternion(rotation)
                
                # Detect color if enabled
                color = 'unknown'
                confidence = 0.0
                if self.enable_color_detection and self.current_rgb is not None:
                    cluster_points = np.asarray(cluster.points)
                    color, confidence = color_detection_from_points(cluster_points, self.current_rgb)
                
                # Create object dict
                obj = {
                    'id': cube_count,
                    'color': color,
                    'position': raw_pos,
                    'confidence': confidence,
                    'fitness': reg_p2p.fitness,
                    'num_points': len(np.asarray(cluster.points))
                }
                detected_cubes.append(obj)
                
                # Publish odometry
                self.publish_odometry(raw_pos, quat, self.world_frame, cube_count)
                
                # Log detection
                euler = rotation_matrix_to_euler_angles(rotation)
                log_msg = f"Cube {cube_count}: pos=[{raw_pos[0]:.3f}, {raw_pos[1]:.3f}, {raw_pos[2]:.3f}], "
                log_msg += f"euler=[{euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f}], fitness={reg_p2p.fitness:.3f}"
                if self.enable_color_detection:
                    log_msg += f", color={color}({confidence:.2f})"
                rospy.loginfo(log_msg)
                
                cube_count += 1
                
                # Remove detected cube points from cluster
                cluster_points = np.asarray(cluster.points)
                distances = np.linalg.norm(cluster_points - raw_pos, axis=1)
                remaining_indices = np.where(distances > self.cube_diagonal + 0.03)[0]
                
                if len(remaining_indices) == 0:
                    break
                
                cluster = cluster.select_by_index(remaining_indices)
                
            except Exception as e:
                rospy.logwarn(f"ICP failed for iteration {iteration}: {e}")
                break
        
        return detected_cubes, cube_count
    
    def publish_detected_objects(self, objects, header):
        """Publish all detected objects as JSON."""
        try:
            data = {
                'objects': objects,
                'timestamp': rospy.Time.now().to_sec(),
                'frame_id': header.frame_id
            }
            
            msg = String()
            msg.data = json.dumps(data)
            self.detected_objects_pub.publish(msg)
            
        except Exception as e:
            rospy.logwarn(f"Failed to publish detected objects: {e}")



def main():
    try:
        node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
