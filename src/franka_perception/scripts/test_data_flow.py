#!/usr/bin/env python3
"""Quick debug script to test perception data flow"""

import rospy
from sensor_msgs.msg import PointCloud2, Image

received_image = False
received_cloud = False

def image_callback(msg):
    global received_image
    if not received_image:
        rospy.loginfo(f"✓ Received RGB image: {msg.width}x{msg.height}, encoding: {msg.encoding}")
        received_image = True

def cloud_callback(msg):
    global received_cloud
    if not received_cloud:
        rospy.loginfo(f"✓ Received point cloud: {msg.width}x{msg.height} points")
        received_cloud = True

if __name__ == '__main__':
    rospy.init_node('test_perception_data')
    
    rospy.loginfo("Testing perception data sources...")
    rospy.loginfo("Subscribing to topics...")
    
    rospy.Subscriber('/zed2/zed_node/left/image_rect_color', Image, image_callback)
    rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, cloud_callback)
    
    rospy.loginfo("Waiting for data (10 seconds)...")
    
    rate = rospy.Rate(1)
    for i in range(10):
        if received_image and received_cloud:
            rospy.loginfo("✓ Both data sources confirmed!")
            break
        rate.sleep()
    else:
        rospy.logwarn("Timeout - did not receive all data")
        if not received_image:
            rospy.logwarn("  ✗ No RGB image received")
        if not received_cloud:
            rospy.logwarn("  ✗ No point cloud received")
