#!/usr/bin/env python3
"""
Automated Phase 2 Testing Script
自动执行 Phase 2 测试，依次发送多个目标位置

简化的 5 步抓取流程：
1. HOME configuration
2. Open gripper
3. Descend to GRASP position
4. Close gripper (automatic grasp)
5. Lift and return HOME

结果输出到 test_results/auto_test_[timestamp].log
"""

import rospy
from geometry_msgs.msg import PoseStamped
import time
import os
from datetime import datetime
import sys

def create_target_pose(x, y, z, frame_id="panda_link0"):
    """创建目标位姿消息"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

class TestLogger:
    """日志记录器，同时输出到控制台和文件"""
    def __init__(self, log_file):
        self.log_file = log_file
        self.file = open(log_file, 'w', buffering=1)  # Line buffering
        self.start_time = datetime.now()
        
    def log(self, message):
        """输出日志到控制台和文件"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_msg = f"[{timestamp}] {message}"
        print(log_msg)
        self.file.write(log_msg + "\n")
        
    def close(self):
        """关闭日志文件"""
        if self.file:
            self.file.close()

def main():
    rospy.init_node('auto_test_phase2', anonymous=True)
    
    # 创建 test_results 目录
    workspace_path = os.path.expanduser("~/panda_grasp_planning/test_results")
    if not os.path.exists(workspace_path):
        os.makedirs(workspace_path, exist_ok=True)
    
    # 生成时间戳
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(workspace_path, f"auto_test_{timestamp}.log")
    
    # 初始化日志记录器
    logger = TestLogger(log_file)
    
    # 创建发布者
    pub = rospy.Publisher('/target_cube_pose', PoseStamped, queue_size=10)
    
    # 等待发布者连接
    rospy.sleep(1.0)
    
    # 定义测试目标 (x, y, z, 描述)
    # 注意：Z坐标必须≥0.10m，以确保grasp位置(z-0.02)在工作空间内
    test_targets = [
        (0.5, 0.0, 0.10, "正前方目标"),
        (0.4, 0.2, 0.10, "右侧目标"),
        (0.6, -0.1, 0.12, "左侧偏高目标"),
        (0.3, 0.3, 0.10, "边缘目标"),
        (0.7, 0.0, 0.15, "远端目标"),
        (0.45, -0.15, 0.11, "左前方目标"),
    ]
    
    logger.log("=" * 60)
    logger.log("PHASE 2 AUTOMATED TESTING")
    logger.log("5-Step Grasp Sequence with Automatic Gripper Control")
    logger.log("=" * 60)
    logger.log(f"Total targets: {len(test_targets)}")
    logger.log("Each grasp sequence takes ~15-20 seconds")
    logger.log(f"Estimated total time: ~{len(test_targets) * 18 / 60:.1f} minutes")
    logger.log(f"Log file: {log_file}")
    logger.log("=" * 60)
    logger.log("")
    
    # 等待用户确认
    logger.log("请确保已启动以下节点：")
    logger.log("  Terminal 1: roslaunch franka_zed_gazebo moveit_gazebo_panda.launch")
    logger.log("  Terminal 2: roslaunch panda_grasp_planning grasp_planning_pipeline.launch")
    logger.log("")
    input("按 Enter 键开始测试...")
    logger.log("测试开始时间: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    logger.log("")
    
    # 记录测试的开始时间
    test_start_time = time.time()
    
    # 依次发送每个目标
    for i, (x, y, z, description) in enumerate(test_targets, 1):
        target_start = time.time()
        logger.log(f"[{i}/{len(test_targets)}] 发送目标: {description}")
        logger.log(f"        位置: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # 创建并发布目标位姿
        target_pose = create_target_pose(x, y, z)
        pub.publish(target_pose)
        
        logger.log(f"        ✓ 目标已发送")
        
        # 等待抓取序列完成
        if i < len(test_targets):
            wait_time = 18
            logger.log(f"        等待抓取完成... (约 {wait_time} 秒)")
            logger.log("")
            time.sleep(wait_time)  # 等待约18秒让机器人完成5步抓取
            elapsed = time.time() - target_start
            logger.log(f"        本次耗时: {elapsed:.1f} 秒")
        else:
            logger.log(f"        最后一个目标已发送")
            logger.log("")
    
    logger.log("=" * 60)
    logger.log("✓ 所有测试目标已发送完成！")
    logger.log("=" * 60)
    logger.log("")
    logger.log("等待最后一个抓取序列完成...")
    time.sleep(30)
    
    total_time = time.time() - test_start_time
    logger.log("")
    logger.log("=" * 60)
    logger.log("测试完成总结")
    logger.log("=" * 60)
    logger.log(f"总耗时: {total_time/60:.1f} 分钟 ({total_time:.1f} 秒)")
    logger.log(f"平均单次耗时: {total_time/len(test_targets):.1f} 秒")
    logger.log(f"日志文件: {log_file}")
    logger.log("")
    logger.log("分析结果命令（测试完成后）：")
    logger.log("")
    logger.log("  cd /opt/ros_ws/src/panda_grasp_planning")
    logger.log("  source /opt/ros_ws/devel/setup.bash")
    logger.log("  rosrun panda_grasp_planning analyze_phase2_results.py \\")
    logger.log("    test_results/phase2_grasp_results.csv --plot")
    logger.log("")
    
    logger.close()
    
    print("")
    print("=" * 60)
    print("✓ 日志已保存到:")
    print(f"  {log_file}")
    print("=" * 60)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("\n测试被中断")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
