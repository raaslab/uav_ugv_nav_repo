#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
import math
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from uav_goals import get_uav_goals
import re

# Declare global variables
waypoints_received = False
waypoints = []
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False
global ugv_goal_status
ugv_goal_status = False
current_position = None

def position_callback(msg):
    global current_position
    current_position = msg



def publish_uav_goal_status(uav_id, status):
    #print("********bas kya*******")
    topic = '/uav_goal_status/uav' + str(uav_id)
    uav_goal_publisher = rospy.Publisher(topic, String, queue_size=10)
    rospy.sleep(2)
    uav_goal_publisher.publish(status)
# New callback function for UGV goal status
def uav_goal_callback(msg, uav_id):
    global ugv_goal_status
    goal_status = msg.data
    if goal_status == "rendezvous_reached":
        ugv_goal_status = True
    elif goal_status == "reached" or goal_status == "in progress":
        ugv_goal_status = False
	


def waypoint_callback(data):
	# print("\n----------waypoint_callback----------")
	global last_waypoint
	# rospy.loginfo("Got waypoint: %s", data)
	if len(data.waypoints) != 0:							# If waypoint list is not empty
		rospy.loginfo("is_current: %s", data.waypoints[len(data.waypoints)-1].is_current)
		last_waypoint = data.waypoints[len(data.waypoints)-1].is_current	# Checks status of "is_current" for last waypoint

def globalPosition_callback(data):
	# print("\n----------globalPosition_callback----------")
	global latitude
	global longitude
	global altitude
	latitude = data.latitude
	longitude = data.longitude
	altitude = data.altitude



def clear_pull(uav_id):
	print("\n----------clear_pull----------")
	# Clearing waypoints
	rospy.wait_for_service('/uav'+ str(uav_id) + '/mavros/mission/clear')
	waypoint_clear = rospy.ServiceProxy('/uav'+ str(uav_id) + '/mavros/mission/clear', WaypointClear)
	resp = waypoint_clear()
	rospy.sleep(5)
	# Call waypoints_pull
	rospy.wait_for_service('/uav'+ str(uav_id) + '/mavros/mission/pull')
	waypoint_pull = rospy.ServiceProxy('/uav'+ str(uav_id) + '/mavros/mission/pull', WaypointPull)
	resp = waypoint_pull()
	rospy.sleep(5)
	return

def finishWaypoints():
	print("\n----------finishwaypoints----------")
	while True:						# Waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		# Waiting for last_waypoint to be true
		if last_waypoint == True:			# If last_waypoint is in the process of being visited
			while True:
				rospy.sleep(2)
				# Waiting for last_waypoint to be false
				if last_waypoint == True:	# If last_waypoint has been visited (due to previous constraint)
					break
			break
	return

def armingCall(uav_id):
	print("\n----------armingCall----------")
	rospy.wait_for_service('/uav'+ str(uav_id) + '/mavros/cmd/arming')
	uav_arm = rospy.ServiceProxy('/uav'+ str(uav_id) + '/mavros/cmd/arming', CommandBool)
	resp = uav_arm(1)
	rospy.sleep(5)

def pushingWaypoints(uav_id,poi):
	print("\n----------pushingWaypoints----------")
	rospy.wait_for_service('/uav'+ str(uav_id) + '/mavros/mission/push')
	waypoint_push = rospy.ServiceProxy('/uav'+ str(uav_id) + '/mavros/mission/push', WaypointPush)
	resp = waypoint_push(0, poi)
	rospy.sleep(5)
	return

def takeoff_call(uav_id,lat, long, alt):
	print("\n----------takeoff_call----------")
	takeoff = rospy.ServiceProxy('/uav'+ str(uav_id) + '/mavros/cmd/takeoff', CommandTOL)
	resp = takeoff(0,0,lat,long, alt)
	rospy.sleep(5)
	return

def switch_modes(uav_id,current_mode, next_mode): # current_mode: int, next_mode: str (http://docs.ros.org/jade/api/mavros_msgs/html/srv/SetMode.html)
	print("\n----------switch_modes----------")
	rospy.wait_for_service('/uav'+ str(uav_id) + '/mavros/set_mode')
	modes = rospy.ServiceProxy('/uav'+ str(uav_id) + '/mavros/set_mode', SetMode)
	resp = modes(current_mode, next_mode)
	rospy.sleep(5)
	return

# ##################################
# # basic modes from MAV_MODE
# uint8 MAV_MODE_PREFLIGHT = 0
# uint8 MAV_MODE_STABILIZE_DISARMED = 80
# uint8 MAV_MODE_STABILIZE_ARMED = 208
# uint8 MAV_MODE_MANUAL_DISARMED = 64
# uint8 MAV_MODE_MANUAL_ARMED = 192
# uint8 MAV_MODE_GUIDED_DISARMED = 88
# uint8 MAV_MODE_GUIDED_ARMED = 216
# uint8 MAV_MODE_AUTO_DISARMED = 92
# uint8 MAV_MODE_AUTO_ARMED = 220
# uint8 MAV_MODE_TEST_DISARMED = 66
# uint8 MAV_MODE_TEST_ARMED = 194
# ##################################3

def land_call(uav_id,lat, long, alt):
	print("\n----------land_call----------")
	land = rospy.ServiceProxy('/uav'+ str(uav_id) + '/mavros/cmd/land', CommandTOL)
	resp = land(0,0,lat,long, alt)
	rospy.sleep(5)
	return

def create_waypoint(lat, lon, alt, is_relative=True):
    wp = Waypoint()
    wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT if is_relative else Waypoint.FRAME_GLOBAL
    wp.command = 16
    wp.is_current = False
    wp.autocontinue = True
    wp.param1 = 0 # hold time
    wp.param2 = 0 # Acceptance radius in meters
    wp.param3 = 0 # Pass or skip the waypoint
    wp.param4 =float('nan') # Desired yaw angle at waypoint (rotary wing)
    wp.x_lat = lat
    wp.y_long = lon
    wp.z_alt = alt
    return wp


def main(uav_id):
    rospy.init_node('wayPoint')
    rospy.Subscriber('/uav'+ str(uav_id) + '/mavros/mission/waypoints', WaypointList, waypoint_callback)
    rospy.Subscriber('/uav'+ str(uav_id) + '/mavros/global_position/raw/fix', NavSatFix, globalPosition_callback)
    readyBit = rospy.Publisher('/uav'+ str(uav_id) + '/mavros/ugv/ready', String, queue_size=10) # Flag topic
    rospy.Subscriber('/uav' + str(uav_id) + '/mavros/global_position/global', NavSatFix, position_callback)

    clear_pull(uav_id)

    armingCall(uav_id)

    # Take off command
    takeoff_call(uav_id,latitude, longitude, 3)
    
    waypoints=[]
    goals=get_uav_goals(uav_id)

    for goal in goals:
        goal_position = goal["position"]
        goal_type = goal["type"]

        waypoint = Waypoint(
            frame=3,
            command=16,
            is_current=0,
            autocontinue=True,
            param1=5,
            x_lat=goal_position.x,
            y_long=goal_position.y,
            z_alt=goal_position.z
        )

        waypoints.append(waypoint)

        # If 'rv' goal is encountered, push waypoints
        if goal_type == 'rv':
            publish_uav_goal_status(uav_id,"progress")
            pushingWaypoints(uav_id,waypoints) # Pushes waypoints to UAV

            #switch_modes(216, "auto.mission")
            switch_modes(uav_id,0, "auto.mission")
            
            finishWaypoints() # Checks if waypoints are finished
	    
            # Inform UGV about UAV's rendezvous position and wait for UGV's response
            publish_uav_goal_status(uav_id,"rendezvous_reached")

            while not ugv_goal_status:
                rospy.sleep(1)

            # Create a new waypoint at the current 'rv' position but 2 meters lower
            waypoint_hover = Waypoint(
                frame=3,
                command=19,  # MAV_CMD_NAV_LOITER_TIME, the UAV will hover at this waypoint for a specified time
                is_current=0,
                autocontinue=True,
                param1=5,  # Duration to hover in seconds
                x_lat=goal_position.x,
                y_long=goal_position.y,
                z_alt=goal_position.z - 2  # Lower the altitude by 2 meters
            )
            waypoints.append(waypoint_hover)

            # Push the new waypoint and switch to auto.mission
            pushingWaypoints(uav_id,waypoints)
            switch_modes(uav_id,0, "auto.mission")

            # Wait until the new waypoint is reached
            finishWaypoints()

            # Clear waypoints for next set
            waypoints = []


    # If there are waypoints left after last 'rv', push them
    if waypoints:
        pushingWaypoints(uav_id,waypoints) # Pushes waypoints to UAV

        #switch_modes(216, "auto.mission")
        switch_modes(uav_id,0, "auto.mission")
        
        finishWaypoints() # Checks if waypoints are finished

    # Land
    land_call(uav_id,latitude, longitude, 0)

    print("EVERYTHING WORKED AS PLANNED!!!")
    rospy.spin()


if __name__ == '__main__':
	uav_id=1
	rospy.Subscriber('/ugv_goal_status', String, lambda msg, uav_id=uav_id: uav_goal_callback(msg, uav_id))
	main(uav_id)
    # node_name = rospy.get_name()  # Get the full node name
    # match = re.search(r'\d+', node_name)
    # uav_id = int(match.group())

