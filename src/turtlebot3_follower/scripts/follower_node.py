#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

leader_pose = None
def leader_callback(msg):
    global leader_pose
    leader_pose = msg

def follower_node():
    global leader_pose
    rospy.init_node('turtlebot3_follower', anonymous=True)
    
    follower_pub = rospy.Publisher('/turtlebot3_2/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtlebot3_1/odom', Odometry, leader_callback)
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        if leader_pose:
            twist = Twist()
            # Implement basic following logic
            distance_to_leader = ((leader_pose.pose.pose.position.x) ** 2 + (leader_pose.pose.pose.position.y) ** 2) ** 0.5
            twist.linear.x = min(0.5, distance_to_leader)  # Move forward at a speed proportional to the distance
            twist.angular.z = 0.0  # No rotation for simplicity
            
            follower_pub.publish(twist)
        
        rate.sleep()

if __name__ == '__main__':
    follower_node()
