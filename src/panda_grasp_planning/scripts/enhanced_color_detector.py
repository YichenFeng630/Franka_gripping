#!/usr/bin/env python3
"""
增强的颜色检测模块 - 参考Reference项目设计
支持多色检测、TF2变换、坐标发布

对齐target: Franka_Panda_Color_Sorting_Robot/panda_vision/color_detector.py
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf_transformations
from geometry_msgs.msg import PointStamped


class EnhancedColorDetector:
    """增强的颜色检测器，采用Reference项目的HSV范围定义"""
    
    def __init__(self):
        rospy.init_node('enhanced_color_detector', anonymous=True)
        
        # 参数
        self.debug_display = rospy.get_param('~display_debug', False)
        self.target_frame = rospy.get_param('~target_frame', 'panda_link0')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        
        # 订阅RGB图像
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw',
            Image,
            self.image_callback,
            queue_size=10
        )
        
        # 发布检测到的坐标 (格式: "COLOR,x,y,z")
        self.coords_pub = rospy.Publisher(
            '/color_coordinates',
            String,
            queue_size=10
        )
        
        # 发布调试图像
        self.debug_pub = rospy.Publisher(
            '/color_detector/debug_image',
            Image,
            queue_size=10
        )
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # TF2设置
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 相机内参（来自标定或hardcode）
        # 如果使用Gazebo，这些值应该从Gazebo plugin读取
        self.fx = rospy.get_param('~fx', 585.0)
        self.fy = rospy.get_param('~fy', 588.0)
        self.cx = rospy.get_param('~cx', 320.0)
        self.cy = rospy.get_param('~cy', 160.0)
        
        # 假设深度（如果没有真实深度）
        self.default_depth = rospy.get_param('~default_depth', 0.1)
        
        # HSV颜色范围 (参考Reference项目)
        # 格式: (lower_HSV, upper_HSV)
        self.color_ranges = {
            'R': {  # 红色
                'lower': np.array([0, 120, 70]),
                'upper': np.array([10, 255, 255]),
            },
            'G': {  # 绿色
                'lower': np.array([55, 200, 200]),
                'upper': np.array([60, 255, 255]),
            },
            'B': {  # 蓝色
                'lower': np.array([90, 200, 200]),
                'upper': np.array([128, 255, 255]),
            },
            'Y': {  # 黄色 (扩展，超出Reference范围)
                'lower': np.array([20, 120, 70]),
                'upper': np.array([40, 255, 255]),
            },
        }
        
        # 最小检测面积 (像素平方)
        self.min_area = rospy.get_param('~min_area', 100)
        
        # 形态学操作的迭代次数
        self.morph_iterations = rospy.get_param('~morph_iterations', 2)
        
        rospy.loginfo("EnhancedColorDetector initialized")
        rospy.loginfo(f"Target frame: {self.target_frame}")
        rospy.loginfo(f"Camera frame: {self.camera_frame}")
        rospy.loginfo(f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
        rospy.loginfo(f"Default depth: {self.default_depth}m")
    
    def image_callback(self, msg):
        """处理RGB图像"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return
        
        # 转换到HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 用于调试的副本
        debug_frame = frame.copy()
        
        # 检测各个颜色
        detections = self._detect_colors_in_image(hsv, debug_frame)
        
        # 发布检测结果
        for detection in detections:
            self._publish_detection(detection)
        
        # 发布调试图像
        if self.debug_display:
            self._publish_debug_image(debug_frame)
    
    def _detect_colors_in_image(self, hsv_frame, debug_frame):
        """
        在HSV图像中检测所有定义的颜色
        
        Returns:
            list of dict: 每个检测结果包含color、pixel_coord、world_coord
        """
        detections = []
        
        for color_id, color_range in self.color_ranges.items():
            lower = color_range['lower']
            upper = color_range['upper']
            
            # 创建mask
            mask = cv2.inRange(hsv_frame, lower, upper)
            
            # 噪声去除 (参考Reference)
            mask = cv2.erode(mask, None, iterations=self.morph_iterations)
            mask = cv2.dilate(mask, None, iterations=self.morph_iterations)
            
            # 查找轮廓
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )
            
            # 处理每个检测到的对象
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                # 过滤最小面积
                if area < self.min_area:
                    continue
                
                # 获取外接矩形
                x, y, w, h = cv2.boundingRect(cnt)
                
                # 计算中心像素坐标
                cx_pix = x + w // 2
                cy_pix = y + h // 2
                
                # 转换到相机坐标系
                camera_coords = self._pixel_to_camera_coords(cx_pix, cy_pix, self.default_depth)
                
                # 转换到目标坐标系 (panda_link0)
                world_coords = self._camera_to_world_coords(camera_coords, msg_header=None)
                
                detection = {
                    'color': color_id,
                    'pixel': (cx_pix, cy_pix),
                    'bbox': (x, y, w, h),
                    'area': area,
                    'camera_coords': camera_coords,
                    'world_coords': world_coords,
                }
                detections.append(detection)
                
                # 调试: 在图像上绘制
                if self.debug_display:
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(
                        debug_frame,
                        f"{color_id} ({area:.0f})",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2
                    )
        
        return detections
    
    def _pixel_to_camera_coords(self, cx_pix, cy_pix, depth):
        """
        将像素坐标转换到相机坐标系
        
        使用标准的相机标定模型:
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        """
        x = (cx_pix - self.cx) * depth / self.fx
        y = (cy_pix - self.cy) * depth / self.fy
        z = depth
        return np.array([x, y, z])
    
    def _camera_to_world_coords(self, camera_coords, msg_header=None):
        """
        使用TF2将相机坐标转换到世界坐标系
        
        Args:
            camera_coords: 相机坐标系中的3D点 [x, y, z]
            msg_header: ROS message header (用于时间戳)
        
        Returns:
            numpy array: 世界坐标系中的3D点
        """
        try:
            # 查询变换: camera_link -> target_frame (通常是panda_link0)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )
            
            # 提取旋转和平移
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 转换为4x4矩阵
            quaternion = [
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ]
            rotation_matrix = tf_transformations.quaternion_matrix(quaternion)
            translation_vector = [translation.x, translation.y, translation.z]
            
            # 构建变换矩阵
            T = np.eye(4)
            T[:3, :3] = rotation_matrix[:3, :3]
            T[:3, 3] = translation_vector
            
            # 应用变换
            camera_coords_homo = np.append(camera_coords, 1.0)
            world_coords_homo = T @ camera_coords_homo
            world_coords = world_coords_homo[:3]
            
            return world_coords
            
        except Exception as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            # 降级方案: 假设相机坐标就是世界坐标
            return camera_coords
    
    def _publish_detection(self, detection):
        """
        发布检测结果
        格式: "COLOR,world_x,world_y,world_z"
        对应Reference项目的String消息格式
        """
        color = detection['color']
        world_x, world_y, world_z = detection['world_coords']
        
        msg = String(data=f"{color},{world_x:.4f},{world_y:.4f},{world_z:.4f}")
        self.coords_pub.publish(msg)
        
        rospy.loginfo(
            f"Published {color}: pixel={detection['pixel']}, "
            f"world=[{world_x:.3f}, {world_y:.3f}, {world_z:.3f}]"
        )
    
    def _publish_debug_image(self, frame):
        """发布调试图像"""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Failed to publish debug image: {e}")


def main():
    detector = EnhancedColorDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
