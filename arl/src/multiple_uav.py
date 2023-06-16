#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import numpy as np
from math import sqrt
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, SetModeRequest
from transformation import apply_transform
from uav_goals import get_uav_goals
import re
ugv_goal_status = False
uav_goal_pub = None

def uav_goal_callback(msg):
    # Callback function to receive UGV's goal status
    global ugv_goal_status
    goal_status = msg.data
    print("Received goal status:", goal_status)
    if goal_status == "rendezvous_reached":
        rospy.set_param('/ugv_goal_reached', True)
        print("UAV received 'reached' status from UGV")
        ugv_goal_status = True  # Set a global variable to track the UGV's goal status
    elif goal_status == "reached" or goal_status == "in progress":
        rospy.set_param('/ugv_goal_reached', False)

def state_callback(msg):
    global current_state
    current_state = msg

def uav_waypoint(uav_id):
    rospy.init_node('uav_waypoint', anonymous=True)
    ugv_goal_sub = rospy.Subscriber('/ugv_goal_status', String, uav_goal_callback)
    uav_position_pub = rospy.Publisher('/uav' + str(uav_id) + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    state_sub = rospy.Subscriber("/uav" + str(uav_id) + "/mavros/state", State, state_callback)
    arming_service = rospy.ServiceProxy("/uav" + str(uav_id) + "/mavros/cmd/arming", CommandBool)
    landing_service = rospy.ServiceProxy("/uav" + str(uav_id) + "/mavros/cmd/land", CommandTOL)
    set_mode_service = rospy.ServiceProxy("/uav" + str(uav_id) + "/mavros/set_mode", SetMode)

    global ugv_goal_status

    rate = rospy.Rate(20)  # 10 Hz

    goals = get_uav_goals(uav_id)

    for goal in goals:
        goal_position = goal["position"]
        goal_type = goal["type"]
        ugv_goal_status = False  # Initialize the UGV goal status
        rospy.set_param('/ugv_goal_reached', False)  # Reset the UGV goal reached parameter

        # Move UAV to the current goal position
        while not rospy.is_shutdown():
            # Publish UAV's current position
            uav_position_pub.publish(create_pose(goal_position.x, goal_position.y, goal_position.z))

            rospy.loginfo("Published UAV position: %s", str(goal_position))

            # Continuously publish goal status
            uav_goal_pub.publish("in progress")

            # Check if UAV has reached its own goal position
            uav_position = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped).pose.position
            distance_to_goal = sqrt((uav_position.x - goal_position.x) ** 2 + (uav_position.y - goal_position.y) ** 2)

            print("Distance to goal is:", distance_to_goal)
            if distance_to_goal < 0.4:
                # Publish goal status as "reached"
                if goal_type == "regular":
                    uav_goal_pub.publish("reached")
                    rospy.loginfo("UAV %d reached goal: %s", uav_id, str(goal_position))
                elif goal_type == "rendezvous":
                    uav_goal_pub.publish("rendezvous_reached")
                    rospy.loginfo("UAV %d published 'rendezvous_reached' status", uav_id)
                    rospy.wait_for_service('/uav' + str(uav_id) + '/mavros/cmd/arming')
                    try:
                        arming_service(True)
                        rospy.loginfo("UAV %d armed", uav_id)
                    except rospy.ServiceException as e:
                        rospy.logerr("Failed to arm UAV %d: %s", uav_id, str(e))

                    rospy.wait_for_service('/uav' + str(uav_id) + '/mavros/set_mode')
                    try:
                        offboard_mode = SetModeRequest()
                        offboard_mode.custom_mode = "OFFBOARD"
                        set_mode_service(0, offboard_mode.custom_mode)
                        rospy.loginfo("UAV %d offboard mode set", uav_id)
                    except rospy.ServiceException as e:
                        rospy.logerr("Failed to set offboard mode for UAV %d: %s", uav_id, str(e))

                    while not rospy.is_shutdown():
                        # Continuously publish rendezvous position until UGV goal is reached
                        if rospy.get_param('/ugv_goal_reached'):
                            ugv_goal_status = True
                            rospy.loginfo("UGV reached the rendezvous point")
                            break
                        else:
                            uav_position_pub.publish(create_pose(goal_position.x, goal_position.y, goal_position.z))
                            rospy.sleep(0.5)

                    while not rospy.is_shutdown() and not rospy.get_param('/ugv_goal_reached'):
                        rospy.loginfo("UAV %d hovering at the rendezvous point...", uav_id)
                        uav_position_pub.publish(create_pose(goal_position.x, goal_position.y, goal_position.z))
                        rospy.sleep(0.5)

                break

            rospy.sleep(0.4)

        rate.sleep()

def create_pose(x, y, z):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose

if __name__ == '__main__':
    node_name = rospy.get_name()  # Get the full node name    
    match = re.search(r'\d+', node_name)
    uav_id = int(match.group())
    uav_goal_pub = rospy.Publisher('/uav_goal_status/uav' + str(uav_id), String, queue_size=10)
    try:
        goals = get_uav_goals(uav_id)
        uav_waypoint(uav_id)
    except rospy.ROSInterruptException:
        pass

