import numpy as np
from geometry_msgs.msg import Point

def apply_transform(goal_position, drone_position):
    goal_x = goal_position.x
    goal_y = goal_position.y
    goal_z = goal_position.z
    drone_x = drone_position[0]
    drone_y = drone_position[1]
    drone_z = drone_position[2]

    relative_position = np.array([goal_x - drone_x, goal_y - drone_y, goal_z - drone_z])
    transformed_position = relative_position + np.array([0, -2, 0])  # Adjust the transformation based on the drone's initial position

    transformed_goal_position = Point()
    transformed_goal_position.x = transformed_position[0]
    transformed_goal_position.y = transformed_position[1]
    transformed_goal_position.z = transformed_position[2]

    return transformed_goal_position
