from pydantic import BaseModel
from typing import Optional, Tuple
import rospy
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry

class Visitor(BaseModel):
    """Visitor class to move the visitor"""
    vel_sub: Optional[rospy.Subscriber] = None
    current_position_visitor: Optional[Tuple[float, float]] = None

    class Config:
        arbitrary_types_allowed = True

    def __init__(self, **data):
        super().__init__(**data)
        # Subscribe to building agent's velocity commands
        self.vel_sub = rospy.Subscriber('visitor/cmd_vel', Twist, queue_size=10)
        
        # Topic to get the position and velocity data from /gazebo node
        self.odom_sub = rospy.Subscriber('visitor/odom', Odometry, self.odom_callback)
        rospy.loginfo("Building follower initialized.")

    def odom_callback(self, msg: Odometry):
        """Odometry callback to update the current position of the building agent."""
        self.current_position_visitor = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo(f"Updated building agent position: {self.current_position_visitor}")
        
    def move_visitor(self, path):
        """Moves the building agent along the path and checks if it reaches its goal."""
        vel_msg = Twist()

        rate = rospy.Rate(10)
        
        while self.current_position_visitor is None:
            rospy.loginfo("Waiting for odometry data...")
            rate.sleep()

        rospy.loginfo("Odometry data received, starting movement...")

        for i in range(len(path) - 1):
            current_point = self.convert_coordinates(path[i][0], path[i][1])
            next_point = self.convert_coordinates(path[i+1][0], path[i+1][1])
            print("Bulding Agent")
            print(current_point , (path[i][0] , path[i][1]))
            print(self.current_position_visitor)

            # Keep moving towards the next point until it's reached
            while not self.reached_point_visitor(next_point):
                # Calculate the distance and angle to the next point
                target_distance = math.sqrt((next_point[0] - self.current_position_visitor[0]) ** 2 + 
                                            (next_point[1] - self.current_position_visitor[1]) ** 2)
                target_angle = math.atan2(next_point[1] - self.current_position_visitor[1], 
                                          next_point[0] - self.current_position_visitor[0])

                # Calculate the difference in orientation
                angle_diff = self.normalize_angle(target_angle - self.current_orientation_visitor)

                # Rotate the robot towards the target
                if abs(angle_diff) > 0.1:  # Threshold for minimal angle difference
                    vel_msg.linear.x = 0.0  # Stop linear movement during rotation
                    vel_msg.angular.z = 0.3 * angle_diff  # Proportional rotation adjustment
                else:
                    vel_msg.angular.z = 0.0  # Stop rotation once aligned
                    vel_msg.linear.x = 0.5  # Move forward

                self.vel_pub.publish(vel_msg)
                rate.sleep()

            # Stop the robot after reaching the next point
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            rate.sleep()

        # Stop the robot at the goal
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        
        return "Reached destination"

    def reached_point_visitor(self, point: Tuple[float, float], tolerance: float = 0.1) -> bool:
        """Check if the building agent has reached the target point within the given tolerance."""
        if self.current_position_visitor is None:
            return False  # If odometry is not available yet
        distance = math.sqrt((point[0] - self.current_position_visitor[0]) ** 2 + 
                             (point[1] - self.current_position_visitor[1]) ** 2)
        return distance < tolerance