#!/usr/bin/env python3
from pydantic import BaseModel, Field
from typing import Optional, Tuple, List
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from crewai_tools import BaseTool
from path_planning_tool import PathPlanningTool
from crewai import Agent, Task, Crew, Process

# Define a CrewAI Tool for managing path planning
class PathPlanningToolWrapper(BaseTool):
    name: str = "Name of my tool"
    description: str = "What this tool does. It's vital for effective utilization."
    map_yaml_path: str
    occupancy_grid: Optional[np.ndarray] = None  # Allowing numpy array as an optional field
    path_planning_tool: Optional['PathPlanningTool'] = None
    class Config:
        arbitrary_types_allowed = True  # Allow arbitrary types such as numpy.ndarray
        
    def _run(self):
        pass

client_agent = Agent(
    role='Campus Incharge Agent (CI Agent)',
    goal='Escort visitors from the campus entrance to the appropriate building and coordinate with the Building Incharge Agent (BI Agent) to guide the visitor to the correct location.',
    backstory="As a CI agent, I am responsible for ensuring that visitors are escorted safely and efficiently from the campus entrance to the building where their host is located. I rely on communication with BI agents for building-specific information, as I do not have access to internal building maps.",
    tools=[PathPlanningToolWrapper()]  # External map
)

visitor_agent = Agent(
    role='Visitor Agent',
    goal='Follow the CI agent from the campus entrance to the building entrance and, after that, follow the instructions of the BI agent to reach the host\'s location.',
    backstory="I am a visitor who has arrived on campus to meet a host. I rely on the CI agent to escort me through the campus and the BI agent to guide me inside the building to the correct room. My journey is simple but depends on the cooperation between these agents to reach my destination smoothly.",
    tools=[PathPlanningToolWrapper()]
)

building_agent = Agent(
    role='Building Incharge Agent (BI Agent)',
    goal='Provide CI agents with internal navigation paths within the building, guiding visitors to their host\'s location and ensuring secure access based on visitor authorization.',
    backstory="As a BI agent, I control access to specific areas within my assigned building. When a CI agent requests assistance, I provide the necessary navigation details from the building entrance to the host's location. I am also responsible for managing out-of-service (OOS) periods when I serve as a host and must ensure I return to service promptly to avoid penalties.",
    tools=[PathPlanningTool()]  # Internal map
)

escort_inside_building_task = Task(
    description='Escort the visitor to the correct room inside the building.',
    agent=building_agent,
    expected_output="The visitor successfully reaches the host's room after navigating the internal building paths provided by the BI agent."
)

escort_to_building_task = Task(
    description='Escort the visitor from the campus entrance to the building entrance.',
    agent=client_agent,
    expected_output="The visitor is successfully guided from the campus entrance to the building, completing the external navigation portion of the tour.")


follow_client_task = Task(
    description='The visitor follows the CI agent to the building entrance.',
    agent=visitor_agent,
    expected_output=" The visitor follows the correct navigation path to reach the building entrance alongside the CI agent.")

# Create the Crew
crew = Crew(
    agents=[client_agent, visitor_agent, building_agent],
    tasks=[escort_to_building_task, follow_client_task, escort_inside_building_task],
    process=Process.sequential
)

# Initialize ROS node
rospy.init_node('campus_tour_node', anonymous=True)
PathPlanningTool.run()
