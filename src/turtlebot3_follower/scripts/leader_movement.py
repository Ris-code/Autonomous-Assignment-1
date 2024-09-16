#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import actionlib
import random

def move_randomly():
    rospy.init_node('leader_move_randomly', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    while not rospy.is_shutdown():
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        
        goal_x = random.uniform(-2.0, 2.0)
        goal_y = random.uniform(-2.0, 2.0)
        
        goal.goal.target_pose.pose.position.x = goal_x
        goal.goal.target_pose.pose.position.y = goal_y
        goal.goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo("Sending goal to leader: x=%f, y=%f", goal_x, goal_y)
        client.send_goal(goal.goal)
        client.wait_for_result()
        
        rospy.sleep(1)  # Wait 5 seconds before sending the next goal

if __name__ == '__main__':
    move_randomly()
