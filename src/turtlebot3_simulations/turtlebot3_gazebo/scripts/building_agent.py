from pydantic import BaseModel
from typing import Optional, Tuple
import rospy
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry


class BuildingAgent(BaseModel):
    """Building Agentclass that follows commands from the building agent and relays them to the visitor."""
    visitor_vel_pub = Optional[rospy.Publisher] = None
    vel_sub: Optional[rospy.Subscriber] = None
    stop_follower: bool = False  # Flag to stop the follower thread
    current_position_building_agent: Optional[Tuple[float, float]] = None  # Current position of the building agent

    class Config:
        arbitrary_types_allowed = True

    def __init__(self, **data):
        super().__init__(**data)
        # Subscribe to building agent's velocity commands
        self.vel_sub = rospy.Subscriber('building_agent/cmd_vel', Twist, queue_size=10)
        
        # Topic to publish msg to visitor about building agent velocity
        self.visitor_vel_pub = rospy.Publisher('client_agent/cmd_vel', Twist, self.callback)
        
        # Topic to get the current position info from /gazebo node
        self.odom_sub = rospy.Subscriber('building_agent/odom', Odometry, self.odom_callback)
        rospy.loginfo("Building follower initialized.")

    def odom_callback(self, msg: Odometry):
        """Odometry callback to update the current position of the building agent."""
        self.current_position_building_agent = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo(f"Updated building agent position: {self.current_position_building_agent}")
        
    def move_building_agent(self, path):
        """Moves the building agent along the path and checks if it reaches its goal."""
        vel_msg = Twist()

        rate = rospy.Rate(10)  # 10 Hz loop rate
        
        while self.current_position_building_agent is None:
            rospy.loginfo("Waiting for odometry data...")
            rate.sleep()

        rospy.loginfo("Odometry data received, starting movement...")

        for i in range(len(path) - 1):
            current_point = self.convert_coordinates(path[i][0], path[i][1])
            next_point = self.convert_coordinates(path[i+1][0], path[i+1][1])
            print("Bulding Agent")
            print(current_point , (path[i][0] , path[i][1]))
            print(self.current_position_client)

                        # Keep moving towards the next point until it's reached
            while not self.reached_point_building_agent(next_point):
                # Calculate the distance and angle to the next point
                target_distance = math.sqrt((next_point[0] - self.current_position_building_agent[0]) ** 2 + 
                                            (next_point[1] - self.current_position_building_agent[1]) ** 2)
                target_angle = math.atan2(next_point[1] - self.current_position_building_agent[1], 
                                          next_point[0] - self.current_position_building_agent[0])

                # Calculate the difference in orientation
                angle_diff = self.normalize_angle(target_angle - self.current_orientation_building_agent)

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
        
        self.stop_follower = True
        return "Reached destination"

    def reached_point_building_agent(self, point: Tuple[float, float], tolerance: float = 0.1) -> bool:
        """Check if the building agent has reached the target point within the given tolerance."""
        if self.current_position_building_agent is None:
            return False  # If odometry is not available yet
        distance = math.sqrt((point[0] - self.current_position_building_agent[0]) ** 2 + 
                             (point[1] - self.current_position_building_agent[1]) ** 2)
        return distance < tolerance

    def callback(self, msg: Twist):
        """Callback to relay velocity commands from building_agent to visitor."""
        if not self.stop_follower:
            rospy.loginfo('I am the visitor! Currently following building agent')
            self.visitor_vel_pub.publish(msg)

    def run(self):
        """Run method that keeps relaying commands until stop_follower is True."""
        rate = rospy.Rate(20)  # 10 Hz loop rate
        while not self.stop_follower:
            rate.sleep()
        rospy.loginfo("Visitor reached the destination.")