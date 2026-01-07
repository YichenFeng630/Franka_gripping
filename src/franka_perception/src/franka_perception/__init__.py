"""
franka_perception package

Vision perception module for Franka robot using ZED2 camera.
Provides object detection, pose estimation, and color segmentation.
"""

__version__ = '1.0.0'
__author__ = 'Yichen Feng'

# Import main classes for convenience
try:
    from .object_detector import ObjectDetector
    from .pose_estimator import PoseEstimator
except ImportError:
    # Allow package to be imported even if classes not yet implemented
    pass
