#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import String
import numpy as np
from math import sqrt
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, SetModeRequest
from transformation import apply_transform
current_state = State()
FLIGHT_ALTITUDE=3
rospy.set_param('/ugv_goal_reached', False)  # Initialize the UGV goal reached parameter
ugv_goal_status = False  
uav_goal_pub = rospy.Publisher('/uav_goal_status', String, queue_size=10)
def uav_goal_callback(msg):
    # Callback function to receive UGV's goal status
    global ugv_goal_status
    goal_status = msg.data
    print("Received goal status:", goal_status)
    if goal_status == "rendezvous_reached":
        rospy.set_param('/ugv_goal_reached', True)
        print("UAV received 'reached' status from UGV")
        ugv_goal_status = True  # Set a global variable to track the UGV's goal status
    elif goal_status =="reached" or goal_status == "in progress":
        rospy.set_param('/ugv_goal_reached', False)

def state_callback(msg):
    global current_state
    current_state = msg

def uav_waypoint():
    rospy.init_node('uav_waypoint', anonymous=True)
    ugv_goal_sub = rospy.Subscriber('/ugv_goal_status', String, uav_goal_callback)
    uav_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    state_sub = rospy.Subscriber("mavros/state", State, state_callback)

    arming_service = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    landing_service = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
    set_mode_service = rospy.ServiceProxy("mavros/set_mode", SetMode)

    origin = np.array([0, 0, 0])
    global ugv_goal_status
    global current_state
    # Specify the goal positions with type
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
        {
            "position": apply_transform(Point(15, 15, 3), origin),
            "type": "regular"
        },
        {
            "position": apply_transform(Point(18, 10, 3), origin),
            "type": "regular"
        },
        {
            "position": apply_transform(Point(10, 18, 3), origin),
            "type": "regular"
        },
        {
            "position": apply_transform(Point(20, 0, 3), origin),
            "type": "rendezvous"
        },
        {
            "position": apply_transform(Point(10, 18, 3), origin),
            "type": "regular"
        },
    ]

    rate = rospy.Rate(20)  # 10 Hz

    for goal in goals:
        goal_position = goal["position"]
        goal_type = goal["type"]
        ugv_goal_status = False  # Initialize the UGV goal status
        rospy.set_param('/ugv_goal_reached', False)  # Reset the UGV goal reached parameter
    
        #ugv_goal_sub = rospy.Subscriber('/ugv_goal_status', String, uav_goal_callback)

        # Move UAV to the current goal position
        while not rospy.is_shutdown():
            # Publish UAV's current position
            uav_position_pub.publish(create_pose(goal_position.x, goal_position.y, goal_position.z))

            rospy.loginfo("Published UAV position: %s", str(goal_position))

            # Continuously publish goal status
            uav_goal_pub.publish("in progress")

            # Check if UAV has reached its own goal position
            uav_position = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped).pose.position
            distance_to_goal = sqrt((uav_position.x - goal_position.x) ** 2 + (uav_position.y - goal_position.y) ** 2)

            print("Distance to goal is:", distance_to_goal)
            if distance_to_goal < 0.4:
                # Publish goal status as "reached"
                if goal_type == "regular":
            # Publish goal status as "reached"
                    uav_goal_pub.publish("reached")
                    rospy.loginfo("UAV reached goal: %s", str(goal_position))

                elif goal_type == "rendezvous":
                    uav_goal_pub.publish("rendezvous_reached")
                    rospy.loginfo("UAV published 'rendezvous_reached' status")
                    rospy.wait_for_service('/mavros/cmd/arming')
                    try:
                        arming_service(True)
                        rospy.loginfo("UAV armed")
                    except rospy.ServiceException as e:
                        rospy.logerr("Failed to arm UAV: %s", str(e))

                    rospy.wait_for_service('/mavros/set_mode')
                    try:
                        offboard_mode = SetModeRequest()
                        offboard_mode.custom_mode = "OFFBOARD"
                        set_mode_service(0, offboard_mode.custom_mode)
                        rospy.loginfo("Offboard mode set")
                    except rospy.ServiceException as e:
                        rospy.logerr("Failed to set offboard mode: %s", str(e))

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
                        rospy.loginfo("Hovering at the rendezvous point...")
                        uav_position_pub.publish(create_pose(goal_position.x, goal_position.y, goal_position.z))
                        rospy.sleep(0.5)

                break

            rospy.sleep(0.4)

        rate.sleep()


def create_pose(x, y, z):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = FLIGHT_ALTITUDE
    return pose

if __name__ == '__main__':
    try:
        uav_waypoint()
    except rospy.ROSInterruptException:
        pass
