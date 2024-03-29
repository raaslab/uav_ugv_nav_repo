#!/usr/bin/env python

import rospy
import re
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from math import sqrt
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, SetModeRequest
from uav_goals import get_uav_goals
from mavros_msgs.msg import State 

# Global Variables
rospy.set_param('/ugv_goal_reached', False)
ugv_goal_status = False
uav_goal_publisher = None
ugv_goal_subscriber =None
current_state = None
current_pose = None

def uav_goal_callback(msg, uav_id):
    # Callback function to receive UGV's goal status
    global ugv_goal_status
    goal_status = msg.data
    print("Received goal status:", goal_status)
    if goal_status == "rendezvous_reached uav1":
        rospy.set_param('/ugv_goal_reached', True)
        print("UAV", uav_id, "received 'rendezvous_reached' status from UGV")
        ugv_goal_status = True  # Set a global variable to track the UGV's goal status
    elif goal_status == "reached" or goal_status == "in progress":
        rospy.set_param('/ugv_goal_reached', False)
        ugv_goal_status = False

def publish_uav_position(uav_id, position):
    topic = '/uav' + str(uav_id) + '/mavros/setpoint_position/local'
    uav_position_publisher = rospy.Publisher(topic, PoseStamped, queue_size=10)
    uav_position_publisher.publish(create_pose(position.x, position.y, position.z))


def publish_uav_goal_status(uav_id, status):
    global uav_goal_publisher
    topic = '/uav_goal_status/uav' + str(uav_id)
    uav_goal_publisher = rospy.Publisher(topic, String, queue_size=10)
    uav_goal_publisher.publish(status)


def arming(uav_id, arm):
    arming_service = rospy.ServiceProxy("/uav" + str(uav_id) + "/mavros/cmd/arming", CommandBool)
    try:
        arming_service(arm)
        rospy.loginfo("UAV %d %s", uav_id, "armed" if arm else "disarmed")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to %s UAV %d: %s", "arm" if arm else "disarm", uav_id, str(e))


def set_mode(uav_id, mode):
    set_mode_service = rospy.ServiceProxy("/uav" + str(uav_id) + "/mavros/set_mode", SetMode)
    try:
        offboard_mode = SetModeRequest()
        offboard_mode.custom_mode = mode
        set_mode_service(0, offboard_mode.custom_mode)
        rospy.loginfo("UAV %d set to %s mode", uav_id, mode)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to set %s mode for UAV %d: %s", mode, uav_id, str(e))


def create_pose(x, y, z):
    """Create a PoseStamped message with the given coordinates"""
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose

def set_home_position():
    position_home = PoseStamped()
    position_home.pose.position.x = 0
    position_home.pose.position.y = 0
    position_home.pose.position.z = 1
    return position_home.pose.position


def initialize_uav(uav_id):
    """Initialize the UAV node and subscribe to UGV goal status"""
    rospy.init_node('uav_waypoint', anonymous=True)
    state_sub = rospy.Subscriber('/uav' + str(uav_id) + '/mavros/state', State, state_cb)
    pose_sub = rospy.Subscriber('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped, pose_cb)

def state_cb(msg):
    # Callback function to update the current state
    global current_state
    current_state = msg

def pose_cb(msg):
# Callback function to update the current pose
    global current_pose
    current_pose = msg 

def waypoint_navigation(uav_id):
    """Perform waypoint navigation for the UAV"""
    initialize_uav(uav_id)
    rate = rospy.Rate(20)  # 10 Hz
    
    goals = get_uav_goals(uav_id)
    takeoff(uav_id,1.5)
    
    #land(uav_id,0)
    for goal in goals:
        goal_position = goal["position"]
        goal_type = goal["type"]
        ugv_goal_status = False
        rospy.set_param('/ugv_goal_reached', False)
        rospy.loginfo("Published UAV position: " + str(goal_position))
        while not rospy.is_shutdown():
            publish_uav_position(uav_id, goal_position)
            
            publish_uav_goal_status(uav_id, "in progress")
            uav_position = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/local_position/pose',
                                                  PoseStamped).pose.position
            distance_to_goal = sqrt((uav_position.x - goal_position.x) ** 2 + (uav_position.y - goal_position.y) ** 2)
            
            if distance_to_goal < 0.4:
                if goal_type == "rv":
                    handle_rendezvous_goal(uav_id, goal_position)
                    # takeoff(uav_id,1)
                    # takeoff(uav_id,1.5)
                    # land(uav_id,0)
                    #handle_hover(uav_id,duration=200)
                else:
                    publish_uav_goal_status(uav_id, "reached")
                    #handle_hover(uav_id)
                    rospy.loginfo("UAV " + str(uav_id) + " reached goal: " + str(goal_position))
                break            
            #rospy.set_param('/ugv_goal_reached', False)

            rate.sleep()
    land(uav_id,0)

def get_dist(pos1, pos2):
    return sqrt((pos1.x - pos2.x) ** 2 + 
                (pos1.y - pos2.y) ** 2+
                (pos1.z - pos2.z) ** 2)

        
def takeoff(uav_id,ALT):
    # Start the main loop 
    rate = rospy.Rate(20) # 20 Hz update rate
    pose_msg=PoseStamped()
    rospy.loginfo("UAV %d hover status", uav_id)
        # Take off first
    take_off_position = None  # Initialize take_off_position variable
    while take_off_position is None:
        try:
            current_pose = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped)
            take_off_position = current_pose.pose.position
            take_off_position.z = ALT  # Update the desired takeoff altitude
        except rospy.ROSInterruptException:
            pass

    # First take off to the desired altitude
    rospy.loginfo("Taking off to UAV position: " + str(take_off_position))
    while not rospy.is_shutdown():
        publish_uav_position(uav_id,take_off_position)
        if get_dist(take_off_position, current_pose.pose.position) < 0.2:
            break
        rate.sleep()

    # now hover for the desired amount of time
    rospy.loginfo("Hovering")
    for i in range(400):
        publish_uav_position(uav_id,take_off_position)
        rate.sleep()
    

def land(uav_id, ALT):
    rospy.loginfo("UAV %d landing status", uav_id)
    
    landing_position = None  # Initialize landing position variable
    
    while landing_position is None:
        try:
            current_pose = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped)
            landing_position = current_pose.pose.position
            landing_position.z = ALT  # Update the desired landing altitude
        except rospy.ROSInterruptException:
            pass
    
    rate = rospy.Rate(20)  # 20 Hz update rate
    
    # First take off to the desired altitude
    rospy.loginfo("Landing to UAV position: " + str(landing_position))
    while not rospy.is_shutdown():
        publish_uav_position(uav_id,landing_position)
        if get_dist(landing_position, current_pose.pose.position) < 0.1:
            break
        rate.sleep()


def handle_hover(uav_id, duration=200):
    """Hover the UAV at the current position for a specified duration"""
    rate = rospy.Rate(20)  # 20 Hz update rate

    for _ in range(duration):
        current_pose = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped)
        hover_position = current_pose.pose.position
        rospy.loginfo("Published UAV position: " + str(take_off_position))
        publish_uav_position(uav_id, hover_position)
        rate.sleep()



def handle_rendezvous_goal(uav_id, goal_position):
    """Handle the logic for rendezvous goals"""
    while not rospy.is_shutdown():
        if rospy.get_param('/ugv_goal_reached'):
            rospy.loginfo("UGV reached the rendezvous point")
            takeoff(uav_id,5.0)
            takeoff(uav_id,2.0)
            publish_uav_goal_status(uav_id, "rendezvous_reached" + str(uav_id))
            rospy.loginfo("UAV %d published 'rendezvous_reached' status", uav_id)
            break
        else:
            publish_uav_position(uav_id, goal_position)
            rospy.sleep(0.5)

    # takeoff(uav_id, goal_position.z-1)
    # takeoff(uav_id, goal_position.z)

    # while not rospy.is_shutdown() and not rospy.get_param('/ugv_goal_reached'):
    #     rospy.loginfo("UAV %d hovering at the rendezvous point...", uav_id)
    #     publish_uav_position(uav_id, goal_position)
    #     rospy.sleep(0.5)


def main():
    # node_name = rospy.get_name()  # Get the full node name
    # match = re.search(r'\d+', node_name)
    # uav_id = int(match.group())
    uav_id=1
    rospy.Subscriber('/ugv_goal_status', String, lambda msg, uav_id=uav_id: uav_goal_callback(msg, uav_id))
    try:
        waypoint_navigation(uav_id)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    #rospy.sleep(5)
    main()


