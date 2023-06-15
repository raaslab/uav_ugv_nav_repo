#!/usr/bin/env python
from geometry_msgs.msg import Point
from transformation import apply_transform

def get_uav_goals(uav_id):
    

    if uav_id == 1:
        origin = [0, 0, 0]
        goals = [
            {
                "position": apply_transform(Point(5, -5, 3), origin),
                "type": "regular"
            },
            {
                "position": apply_transform(Point(10, 5, 3), origin),
                "type": "regular"
            },
            {
                "position": apply_transform(Point(10, 0, 3), origin),
                "type": "rendezvous"
            },
        ]
    elif uav_id == 2:
        origin = [0, 0, 0]
        goals = [
            {
                "position": apply_transform(Point(15, 0, 2), origin),
                "type": "regular"
            },
            {
                "position": apply_transform(Point(28, 10, 2), origin),
                "type": "regular"
            },
            {
                "position": apply_transform(Point(15, 15, 2), origin),
                "type": "rendezvous"
            },
        ]
    elif uav_id == 3:
        origin = [-2, 0, 0]
        goals = [
            {
                "position": apply_transform(Point(20, 0, 3), origin),
                "type": "rendezvous"
            },
            {
                "position": apply_transform(Point(10, 18, 3), origin),
                "type": "regular"
            },
        ]
    else:
        goals = []

    return goals
