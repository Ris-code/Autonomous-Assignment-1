from pydantic import BaseModel
from typing import Tuple, List
import numpy as np
import yaml
import heapq
from client_agent import ClientAgent
from building_agent import BuildingAgent
from visitor import Visitor
import threading
import rospy
import matplotlib.pyplot as plt

class PathPlanningTool(BaseModel):
    """A class for managing path planning logic for the client and building agents."""
    start_pose: Tuple[float, float]
    goal_pose: Tuple[float, float]
    map_yaml_path: str
    
    class Config:
        arbitrary_types_allowed = True
        
    def __init__(self, **data):
        super().__init__(**data)
        self.start_pose = (0.0, 0.0)
        self.goal_pose = (10.0, 10.0)
        self.map_yaml_path = 'my_map.yaml'
        
    def load_map(self):
        map_data = self.load_map_yaml(self.map_yaml_path)
        self.occupancy_grid, self.resolution, self.origin = self.map_image_to_grid(map_data)
        return self.occupancy_grid

    @staticmethod
    def load_map_yaml(file_path: str) -> dict:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    @staticmethod
    def map_image_to_grid(map_data: dict) -> Tuple[np.ndarray, float, Tuple[float, float, float]]:
        f = 'my_map.pgm'

        with open(f, 'rb') as pgmf:
            image = plt.imread(pgmf)
            
        # image = cv2.imread(map_data['image'], cv2.IMREAD_GRAYSCALE)
        resolution = map_data['resolution']
        origin = map_data['origin']
        print(image)
        print(np.unique(image))
        occupancy_grid = np.zeros_like(image)

        occupancy_grid[image == 205] = 100
        occupancy_grid[image == 254] = 100
        occupancy_grid[image == 0] = 0
        
        print(occupancy_grid.shape)
        return occupancy_grid, resolution, origin

    def a_star_algorithm(self, occupancy_grid, start, goal):
        """Implements the A* pathfinding algorithm."""
        # A* algorithm logic to find a path from start to goal
        start = (round(start[0]), round(start[1]))
        goal = (round(goal[0]), round(goal[1]))
        
        print(start)
        print(goal)
        
        steps = 0 
        def euclideanDist(start , end ) : 
            return ((start[0] - end[0])**2 + (start[1] - end[1])**2) ** 0.5
        def heuristic_function( start , end ) : 
            return abs(start[0] - end[0]) + abs(start[1] - end[1])

        pq = []
        heapq.heappush( pq , ( 0 , start , [start] ))
        move_directions: List[Tuple[float, float]] = [(-1, 0), (1, 0), (0, -1), (0, 1), 
                                                  (-1, -1), (-1, 1), (1, -1), (1, 1)]
        if(occupancy_grid[start[0]][start[1]] != 100 ) : 
            print("Start point invalid")
            return None
        visited = set()
        visited.add(start)
        while pq:
            steps += 1
            cost , node , path = heapq.heappop(pq)

            if(node == goal ) : 
                print(f"path found in {steps} steps")
                return path
            
            for dir in move_directions:
                x = dir[0]
                y = dir[1] 
                newNode = (node[0] + x , node[1] + y )
                # print(newNode)
                # print(occupancy_grid[node[0] + x][node[1] + y])
                if(newNode not in visited and occupancy_grid[node[0] + x][node[1] + y] == 100 ) : 
                    # q.append([newNode , path + [newNode]])
                    h = heuristic_function(newNode , goal)
            
                    heapq.heappush(pq , (cost + euclideanDist(node , newNode ) + h , newNode , path + [newNode]))
                    visited.add(newNode)
        print("path not found")
        return None
    
    def client_run_follower():
        ClientAgent.run()

    def building_agent_run_follower():
        BuildingAgent.run()

    def run_client_agent(path):
        result = ClientAgent.move_client_agent(path)
        print(result)

    def run_building_agent(path):
        result = BuildingAgent.move_building_agent(path)
        print(result)
    
    def visitor_and_client(self, occupancy_grid):
        print("Client + Visitor system operating")
        start_pose = ClientAgent.inverse_convert_coordinates(2, -5)
        goal_pose = ClientAgent.inverse_convert_coordinates(1, -1)
        path = self.a_star_algorithm(occupancy_grid, start_pose, goal_pose)

        # Run follower and client_agent in parallel using threads
        client_thread = threading.Thread(target=self.run_client_agent, args=(path,))
        client_follower_thread = threading.Thread(target=self.client_run_follower, args=())

        # Start both threads
        client_thread.start()
        client_follower_thread.start()

        # Wait for both threads to finish
        client_thread.join()
        client_follower_thread.join()
        
        rospy.loginfo("Hey Visitor!! You have reached the building")
        
    def move_client_aside(self, occupancy_grid):
        print("Shifting the client")
        start_pose = ClientAgent.inverse_convert_coordinates(1, -1)
        goal_pose = ClientAgent.inverse_convert_coordinates(2, -1)
        path1 = ClientAgent.a_star_algorithm(occupancy_grid, start_pose, goal_pose)

        print("Visitor system operating")
        start_pose = ClientAgent.inverse_convert_coordinates(1, -2)
        goal_pose = ClientAgent.inverse_convert_coordinates(1, 0)
        path2 = self.a_star_algorithm(occupancy_grid, start_pose, goal_pose)

        client_shift_thread = threading.Thread(target=ClientAgent.move_client_agent, args=(path1,))
        visitor_shift_thread = threading.Thread(target=Visitor.move_visitor, args=(path2,))

        client_shift_thread.start()
        visitor_shift_thread.start()

        client_shift_thread.join()
        visitor_shift_thread.join()
        
        rospy.loginfo("Hey Visitor!! You have reached the building")
    
    def visitor_and_building_agent(self, occupancy_grid):
        print("Building agent + visitor system operating")
        start_pose = ClientAgent.inverse_convert_coordinates(0, 0.5)

        room = {'LAB1': (-6,4), 'LAB2': (), 'Office': (), 'Discussion Room': ()}
        visitor_goal = input("Hey Visitor!! I am Building agent. I will escort you to the desired room. You want to go to LAB1 or LAB2 or Office or Discussion Room")

        goal_pose = ClientAgent.inverse_convert_coordinates(room[visitor_goal][0], room[visitor_goal][1])


        print("Start Postiion : " , start_pose )
        print( "Goal Position : " , goal_pose )


        path = self.a_star_algorithm(occupancy_grid, start_pose, goal_pose)
        # path = obj.bfs_traversal(occupancy_grid, start_pose, goal_pose)
        print(path)

        # Run follower and building_agent in parallel using threads
        building_agent_thread = threading.Thread(target=self.run_building_agent, args=(path,))
        building_agent_follower_thread = threading.Thread(target=self.building_agent_run_follower, args=())

        # Start both threads
        building_agent_thread.start()
        building_agent_follower_thread.start()

        # Wait for both threads to finish
        building_agent_thread.join()
        building_agent_follower_thread.join()

        rospy.loginfo('Kudos Visitor!! You have reached your final location')
        
    def run(self):
        # Initialize your path planning object and run your code as before
        occupancy_grid= self.load_map()

        self.visitor_and_client(occupancy_grid)
        self.move_client_aside(occupancy_grid)
        self.visitor_and_building_agent(occupancy_grid)
        
        rospy.loginfo('Mission Accomplished Boss!!')