#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def callback(msg):
    global follower_pub
    twist = Twist()
    # Copy the linear and angular velocities from the leader
    twist.linear.x = msg.linear.x
    twist.angular.z = msg.angular.z
    follower_pub.publish(twist)

def listener():
    global follower_pub
    rospy.init_node('turtlebot3_follower', anonymous=True)
    # Publisher to send movement commands to the follower TurtleBot
    follower_pub = rospy.Publisher('/turtlebot3_2/cmd_vel', Twist, queue_size=10)
    # Subscriber to receive movement commands from the leader TurtleBot
    rospy.Subscriber('/turtlebot3_1/cmd_vel', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
