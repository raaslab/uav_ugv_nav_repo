#!/usr/bin/env python
from geometry_msgs.msg import Point
from transformation import apply_transform

def parse_uav_goals(file_path):
    goals = []
    with open(file_path, 'r') as file:
        lines = file.readlines()[1:]
        for line in lines:
            line = line.strip()  # Remove leading/trailing whitespace and newline characters
            if line:  # Ignore empty lines and rendezvous points
                parts = line.split(',')
                x = float(parts[1].strip())
                y = float(parts[2].strip())
                goal_type = parts[3].strip()
                goal = {'x': x, 'y': y, 'type': goal_type}
                goals.append(goal)
    return goals

def get_uav_goals(uav_id):
    if uav_id == 1:
        uav_goals_file = '/home/experiment/catkin_ws/src/arl/goals/uav1_goals.txt'
    elif uav_id == 2:
        uav_goals_file = '/home/experiment/catkin_ws/src/arl/goals/uav2_goals.txt'
    elif uav_id == 3:
        uav_goals_file = '/home/experiment/catkin_ws/src/arl/goals/uav3_goals.txt'
    else:
        return []

    goals = parse_uav_goals(uav_goals_file)

    # Transform goals if needed
    origin = [0, 0, 0]  # Define the origin for transformations
    transformed_goals = []
    for goal in goals:
        position = apply_transform(Point(goal['x'], goal['y'], 3), origin)
        transformed_goal = {
            'position': position,
            'type': goal['type']
        }
        transformed_goals.append(transformed_goal)

    return transformed_goals

