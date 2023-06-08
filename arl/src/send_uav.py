#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import String
import numpy as np

rospy.set_param('/ugv_goal_reached', False)  # Initialize the UGV goal reached parameter

# def apply_transform(goal_position, drone_position):
#     goal_x = goal_position.x
#     goal_y = goal_position.y
#     goal_z = goal_position.z
#     drone_x = drone_position[0]
#     drone_y = drone_position[1]
#     drone_z = drone_position[2]

#     relative_position = np.array([goal_x - drone_x, goal_y - drone_y, goal_z - drone_z])
#     transformed_position = relative_position + np.array([0, -2, 0])  # Adjust the transformation based on the drone's initial position

#     transformed_goal_position = Point()
#     transformed_goal_position.x = transformed_position[0]
#     transformed_goal_position.y = transformed_position[1]
#     transformed_goal_position.z = transformed_position[2]

#     return transformed_goal_position

def uav_goal_callback(msg):
    # Callback function to receive UGV's goal status
    goal_status = msg.data

    if goal_status == "reached":
        rospy.set_param('/ugv_goal_reached', True)
    elif goal_status =="in progress":
        rospy.set_param('/ugv_goal_reached', False)

def uav_waypoint():
    rospy.init_node('uav_waypoint', anonymous=True)
    uav_goal_pub = rospy.Publisher('/uav_goal_status', String, queue_size=10)
    ugv_goal_sub = rospy.Subscriber('/ugv_goal_status', String, uav_goal_callback)
    uav_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    origin = np.array([0,0,0])
    # Specify the three goal positions
    # goals = [
        
    #     apply_transform(Point(1, 1, 2), origin),  # Apply transformation for UAV goal position
    #     apply_transform(Point(2, 2, 2), origin),
    #     apply_transform(Point(3, 3, 2), origin)
    # ]
    goals = [(2, 0, 2), (3, 2, 2), (5, 3, 2)]

    rate = rospy.Rate(10)  # 10 Hz

    for goal in goals:
        # Move UAV to the current goal position
        while not rospy.is_shutdown():
            # Publish UAV's current position
            #uav_position_pub.publish(create_pose(goal.x, goal.y, goal.z))
            uav_position_pub.publish(create_pose(goal[0], goal[1], goal[2]))

            rospy.loginfo("Published UAV position: %s", str(goal))

            # Continuously publish goal status
            uav_goal_pub.publish("in progress")
                        # Check if UAV has reached its own goal position
            uav_position = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped).pose.position
            if abs(uav_position.x - goal[0]) < 0.1 and abs(uav_position.y - goal[1]) < 0.1:
                break
            
            # Wait for some time
            rospy.sleep(1.0)

            # Check if UGV's goal status indicates it has reached the current goal position
            ugv_goal_status = rospy.get_param('/ugv_goal_reached')
            # If so, break the loop and proceed to the next goal
            if ugv_goal_status:
                break
        # Check if UAV has reached its own goal position
        uav_position = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped).pose.position
        if abs(uav_position.x - goal[0]) < 0.1 and abs(uav_position.y - goal[1]) < 0.1:
            break
        # Publish goal status as "reached"
        uav_goal_pub.publish("reached")
        rospy.loginfo("UAV reached goal: %s", str(goal))
        rate.sleep()

def create_pose(x, y, z):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose

if __name__ == '__main__':
    try:
        uav_waypoint()
    except rospy.ROSInterruptException:
        pass
