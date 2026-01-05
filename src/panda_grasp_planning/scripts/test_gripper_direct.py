#!/usr/bin/env python3
"""Test gripper control directly"""

import rospy
import actionlib
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

def test_gripper():
    rospy.init_node('gripper_test')
    
    print("Creating gripper move client...")
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    
    print("Waiting for gripper move server...")
    if not move_client.wait_for_server(timeout=rospy.Duration(5)):
        print("ERROR: Gripper move server not available!")
        return False
    
    print("✓ Gripper move server found!")
    
    # Test gripper open
    print("\nTesting gripper OPEN...")
    goal = MoveGoal()
    goal.width = 0.08
    goal.speed = 0.1
    move_client.send_goal(goal)
    move_client.wait_for_result(rospy.Duration(5))
    print("Gripper opened to 0.08m")
    
    rospy.sleep(1)
    
    # Test gripper close
    print("\nTesting gripper CLOSE...")
    goal = MoveGoal()
    goal.width = 0.02
    goal.speed = 0.1
    move_client.send_goal(goal)
    result = move_client.wait_for_result(rospy.Duration(5))
    if result:
        print("✓ Gripper closed to 0.02m")
    else:
        print("✗ Gripper close timeout")
    
    print("\n=== Test Complete ===")
    return True

if __name__ == '__main__':
    try:
        test_gripper()
    except Exception as e:
        print(f"Error: {e}")
