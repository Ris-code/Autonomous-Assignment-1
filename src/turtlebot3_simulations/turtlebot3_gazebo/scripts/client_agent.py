from pydantic import BaseModel
from typing import Optional, Tuple
import rospy
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry


class ClientAgent(BaseModel):
    """Client Agent  class that follows the commands from the client agent and relays them to the visitor.""" 
    visitor_vel_pub = Optional[rospy.Publisher] = None
    vel_sub: Optional[rospy.Subscriber] = None
    stop_follower: bool = False  # Flag to stop the follower thread
    odom_sub: Optional[rospy.Subscriber] = None
    current_position_client: Optional[Tuple[float, float]] = None  # Current position of the client agent

    class Config:
        arbitrary_types_allowed = True

    def __init__(self, **data):
        super().__init__(**data)
        # Subscribe to client agent's velocity commands
        self.vel_sub = rospy.Subscriber('client_agent/cmd_vel', Twist, queue_size=10)
        
        # Topic to publish the velocity msg to visitor
        self.visitor_vel_pub = rospy.Publisher('visitor/cmd_vel', Twist, self.callback)
        
        # add odometry to get the velocity and position data from gazebo node
        self.odom_sub = rospy.Subscriber('client_agent/odom', Odometry, self.odom_callback)

        rospy.loginfo("Client follower initialized.")

    def odom_callback(self, msg: Odometry):
        """Odometry callback to update the current position of the client."""
        self.current_position_client = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo(f"Updated client position: {self.current_position_client}")

    def move_client_agent(self, path):
        """Moves the client agent along the path and checks if it reaches its goal."""
        vel_msg = Twist()

        rate = rospy.Rate(10)  # 10 Hz loop rate
        
        while self.current_position_client is None:
            rospy.loginfo("Waiting for odometry data...")
            rate.sleep()

        rospy.loginfo("Odometry data received, starting movement...")

        for i in range(len(path) - 1):
            print(f"Current Postion : {path[i][0] , path[i][1]}")
            print(f"Next Postioon : {path[i+1][0] , path[i+1][1]}")
            
            current_point = self.convert_coordinates(path[i][0], path[i][1])
            next_point = self.convert_coordinates(path[i+1][0], path[i+1][1])
            
            print(current_point)
            print(self.current_position_client)

            # Keep moving towards the next point until it's reached
            while not self.reached_point_client(next_point):
                # Calculate the distance and angle to the next point
                target_distance = math.sqrt((next_point[0] - self.current_position_client[0]) ** 2 + 
                                            (next_point[1] - self.current_position_client[1]) ** 2)
                target_angle = math.atan2(next_point[1] - self.current_position_client[1], 
                                          next_point[0] - self.current_position_client[0])

                # Calculate the difference in orientation
                angle_diff = self.normalize_angle(target_angle - self.current_orientation_client)

                # Rotate the robot towards the target
                if abs(angle_diff) > 0.1:  # Threshold for minimal angle difference
                    vel_msg.linear.x = 0.0  # Stop linear movement during rotation
                    vel_msg.angular.z = 0.2 * angle_diff  # Proportional rotation adjustment
                else:
                    vel_msg.angular.z = 0.0  # Stop rotation once aligned
                    vel_msg.linear.x = 0.3  # Move forward

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
        
        self.stop_follower = True
        return "Reached destination"
    
    def reached_point_client(self, point: Tuple[float, float], tolerance: float = 0.1) -> bool:
        """Check if the client agent has reached the target point within the given tolerance."""
        if self.current_position_client is None:
            return False  # If odometry is not available yet
        distance = math.sqrt((point[0] - self.current_position_client[0]) ** 2 + 
                             (point[1] - self.current_position_client[1]) ** 2)
        return distance < tolerance

    def callback(self, msg: Twist):
        """Callback to relay velocity commands from building_agent to visitor."""
        if not self.stop_follower:
            rospy.loginfo("I am Visitor! Currently following the client agent")
            self.visitor_vel_pub.publish(msg)

    def run(self):
        """Run method that keeps relaying commands until stop_follower is True."""
        rate = rospy.Rate(20)  
        while not self.stop_follower:
            rate.sleep()
        rospy.loginfo("Visitor reached the destination")