#!/usr/bin/env python3
"""
Point Cloud Helper Functions
============================
Adapted from frankastadt reference project with improvements.
Provides Open3D-based point cloud processing utilities.
"""

import rospy
import copy
import tf2_ros
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import open3d as o3d
import matplotlib.pyplot as plt


def cluster_pc(pc, eps, min_points):
    """
    Cluster the point cloud using DBSCAN
    Args:
        pc (open3d.geometry.PointCloud): point cloud
        eps (float): maximum distance between two points to be considered in the same neighborhood
        min_points (int): minimum number of points to form a cluster
    Returns:
        labels (np.array): array of cluster labels
    """
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error) as cm:
        labels = np.array(pc.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    return labels


def create_cube_gt(edge_len):
    """
    Create a cube ground truth model with the given edge length
    Args:
        edge_len (float): edge length of the cube
    Returns:
        cube_gt (open3d.geometry.PointCloud): point cloud of the cube
    """
    mesh = o3d.geometry.TriangleMesh.create_box(width=edge_len, height=edge_len, depth=edge_len)
    cube_gt = mesh.sample_points_uniformly(number_of_points=500)
    
    # Keep only three visible faces (x-min, y-min, z-max)
    cube_gt_x = cube_gt.select_by_index(np.where(np.asarray(cube_gt.points)[:, 0] < 0.001)[0])
    cube_gt_y = cube_gt.select_by_index(np.where(np.asarray(cube_gt.points)[:, 1] < 0.001)[0])
    cube_gt_z = cube_gt.select_by_index(np.where(np.asarray(cube_gt.points)[:, 2] > edge_len - 0.001)[0])
    cube_gt = cube_gt_x + cube_gt_y + cube_gt_z
    
    # Center the cube
    points = np.asarray(cube_gt.points)
    transformed_points = points - np.array([edge_len/2, edge_len/2, edge_len/2])
    cube_gt.points = o3d.utility.Vector3dVector(transformed_points)
    
    return cube_gt


def crop_pointcloud(pc, boundX, boundY, boundZ):
    """
    Crop the point cloud to the given bounds
    Args:
        pc (open3d.geometry.PointCloud): point cloud
        boundX (list): x-axis bounds [min, max]
        boundY (list): y-axis bounds [min, max]
        boundZ (list): z-axis bounds [min, max]
    Returns:
        zf_cloud (open3d.geometry.PointCloud): cropped point cloud
    """
    xf_cloud = pc.crop(
        o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(boundX[0], -np.inf, -np.inf),
            max_bound=(boundX[1], np.inf, np.inf)
        )
    )
    
    yf_cloud = xf_cloud.crop(
        o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(-np.inf, boundY[0], -np.inf),
            max_bound=(np.inf, boundY[1], np.inf)
        )
    )
    
    zf_cloud = yf_cloud.crop(
        o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(-np.inf, -np.inf, boundZ[0]),
            max_bound=(np.inf, np.inf, boundZ[1])
        )
    )
    return zf_cloud


def o3dpc_to_rospc(o3dpc, header, fields, frame_id=None):
    """
    Convert open3d point cloud to ROS point cloud
    Args:
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
        header: ROS header
        fields: ROS point cloud fields
        frame_id (string): frame id of ros point cloud header
    Returns:
        rospc (sensor_msgs.PointCloud2): ros point cloud message
    """
    cloud_npy = np.asarray(copy.deepcopy(o3dpc.points))
    is_color = len(o3dpc.colors) > 0
        
    n_points = len(cloud_npy[:, 0])
    if is_color:
        data = np.zeros(n_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])
    else:
        data = np.zeros(n_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
    
    data['x'] = cloud_npy[:, 0]
    data['y'] = cloud_npy[:, 1]
    data['z'] = cloud_npy[:, 2]
    
    if is_color:
        rgb_npy = np.asarray(copy.deepcopy(o3dpc.colors))
        rgb_npy = np.floor(rgb_npy * 255)  # nx3 matrix
        rgb_npy = rgb_npy[:, 0] * 2**16 + rgb_npy[:, 1] * 2**8 + rgb_npy[:, 2]
        rgb_npy = rgb_npy.astype(np.uint32)
        data['rgb'] = rgb_npy

    rospc = pc2.create_cloud(header, fields, data)
    if frame_id is not None:
        rospc.header.frame_id = frame_id
    
    return rospc


def obtain_pc_rotation(pc):
    """
    Obtain the rotation of the point cloud by minimizing bounding box volume
    Args:
        pc (open3d.geometry.PointCloud): point cloud
    Returns:
        rotation (np.array): rotation matrix
    """
    # Save volume of axis aligned bounding box of pc
    bounding_box = pc.get_axis_aligned_bounding_box()
    volume = bounding_box.volume()

    # Rotate the point cloud by 1/32 of a circle and save the rotation with smallest volume
    rotation = 0
    for i in range(0, 32):
        pc.rotate(pc.get_rotation_matrix_from_xyz((0, 0, np.pi/64)), center=(0, 0, 0))
        bounding_box = pc.get_axis_aligned_bounding_box()
        new_volume = bounding_box.volume()
        if new_volume < volume:
            volume = new_volume
            rotation = (i+1) * np.pi / 64
    
    return pc.get_rotation_matrix_from_xyz((0, 0, rotation))


def rotation_matrix_to_euler_angles(R):
    """
    Convert a 3x3 rotation matrix to its euler angles representation
    Args:
        R (np.array): 3x3 rotation matrix
    Returns:
        euler_angles (np.array): euler angles [roll, pitch, yaw]
    """
    # Extract pitch (around y-axis)
    pitch = np.arcsin(-R[2, 0])

    # Calculate the other angles based on pitch
    if np.abs(np.cos(pitch)) > 1e-6:
        # Not at poles
        yaw = np.arctan2(R[1, 0], R[0, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:
        # At poles
        yaw = 0
        roll = np.arctan2(-R[0, 1], R[1, 1])

    return np.array([roll, pitch, yaw])


def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix to its quaternion representation
    Args:
        R (np.array): 3x3 rotation matrix
    Returns:
        q (np.array): quaternion [x, y, z, w]
    """
    q = np.empty(4)

    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

    return q


def segment_pc(pc, distance_threshold):
    """
    Segment the largest planar component from the point cloud using RANSAC
    Args:
        pc (open3d.geometry.PointCloud): point cloud
        distance_threshold (float): distance threshold for RANSAC
    Returns:
        outlier_cloud (open3d.geometry.PointCloud): point cloud without the largest planar component
    """
    plane_model, inliers = pc.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=1000
    )
    [a, b, c, d] = plane_model
    outlier_cloud = pc.select_by_index(inliers, invert=True)
    
    return outlier_cloud


def transform_pointcloud(msg, target_frame):
    """
    Transform the point cloud to the target frame
    Args:
        msg (sensor_msgs.PointCloud2): point cloud message
        target_frame (string): target frame
    Returns:
        o3d_pc (open3d.geometry.PointCloud): transformed point cloud
    """
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # Get the transform from the camera frame to the target frame
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            msg.header.frame_id,
            rospy.Time(0),
            rospy.Duration(2.0)
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Error while transforming the point cloud: {e}")
        return None
    
    # Convert to Open3D point cloud
    msg.header.frame_id = target_frame
    o3d_pc = o3d.geometry.PointCloud()
    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    
    if len(points) == 0:
        return None
    
    o3d_pc.points = o3d.utility.Vector3dVector(np.array(points))

    # Transform the point cloud
    translation = np.array([
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    ])
    rotation = np.array([
        transform.transform.rotation.w,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z
    ])
    
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(rotation)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation
    o3d_pc.transform(transformation_matrix)
    
    return o3d_pc


def color_detection_from_points(points, rgb_image, camera_matrix=None):
    """
    Detect color for a set of 3D points by projecting to image
    Args:
        points (np.array): Nx3 array of 3D points
        rgb_image (np.array): RGB image
        camera_matrix (np.array): 3x3 camera intrinsic matrix (optional)
    Returns:
        color_name (str): detected color name
        confidence (float): detection confidence
    """
    if rgb_image is None or len(points) == 0:
        return 'unknown', 0.0
    
    import cv2
    
    # Color thresholds in HSV
    color_thresholds = {
        'red': {
            'lower': np.array([0, 100, 100]),
            'upper': np.array([10, 255, 255]),
            'lower2': np.array([170, 100, 100]),
            'upper2': np.array([180, 255, 255])
        },
        'blue': {
            'lower': np.array([100, 100, 100]),
            'upper': np.array([130, 255, 255])
        },
        'green': {
            'lower': np.array([35, 100, 100]),
            'upper': np.array([85, 255, 255])
        },
        'yellow': {
            'lower': np.array([20, 100, 100]),
            'upper': np.array([35, 255, 255])
        },
    }
    
    try:
        centroid = points.mean(axis=0)
        h, w = rgb_image.shape[:2]
        
        # Simple projection (should use camera_matrix for accurate projection)
        if camera_matrix is not None:
            # Project using camera matrix
            fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
            cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
            px = int(fx * centroid[0] / centroid[2] + cx)
            py = int(fy * centroid[1] / centroid[2] + cy)
        else:
            # Simplified projection
            px = int(w / 2 + centroid[0] * 400)
            py = int(h / 2 - centroid[1] * 400)
        
        if not (0 <= px < w and 0 <= py < h):
            return 'unknown', 0.0
        
        # Sample region around projected point
        y_min, y_max = max(0, py - 5), min(h, py + 6)
        x_min, x_max = max(0, px - 5), min(w, px + 6)
        region = rgb_image[y_min:y_max, x_min:x_max]
        
        if region.size == 0:
            return 'unknown', 0.0
        
        # Convert to HSV
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        
        # Match color thresholds
        best_color = 'unknown'
        best_match = 0.0
        
        for color_name, thresholds in color_thresholds.items():
            if 'lower2' in thresholds:  # Red wraps around HSV
                mask1 = cv2.inRange(hsv, thresholds['lower'], thresholds['upper'])
                mask2 = cv2.inRange(hsv, thresholds['lower2'], thresholds['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, thresholds['lower'], thresholds['upper'])
            
            match_ratio = np.sum(mask > 0) / (mask.size + 1e-6)
            if match_ratio > best_match:
                best_match = match_ratio
                best_color = color_name
        
        # Only accept if confidence > threshold
        if best_match > 0.3:
            return best_color, best_match
        
        return 'unknown', 0.0
    
    except Exception as e:
        rospy.logdebug(f"Color detection failed: {e}")
        return 'unknown', 0.0
