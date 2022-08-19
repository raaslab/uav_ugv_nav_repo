#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

#global variables
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False

def waiter(condition):
	while True:
		if condition:
			return
		else:
			rospy.sleep(2)

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

def waiting_ugv(lat, long, alt):
	print("\n----------waiting_ugv----------")
	while True:
		# TODO: add listener to the UGV flag here
		# checker = UGV publisher
		checker = 1
		if checker == 1:
			waypoints = [Waypoint(frame = 3, command = 21, is_current = 1, autocontinue = True, param1 = 5, x_lat = lat, y_long = long, z_alt = alt)]			
			waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
			resp = waypoint_push(0, waypoints)
			rospy.sleep(5)			
			return

def clear_pull():
	print("\n----------clear_pull----------")
	# Clearing waypoints
	rospy.wait_for_service("/mavros/mission/clear")
	waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
	resp = waypoint_clear()
	rospy.sleep(5)
	# Call waypoints_pull
	rospy.wait_for_service("/mavros/mission/pull")
	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
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

def armingCall():
	print("\n----------armingCall----------")
	rospy.wait_for_service("/mavros/cmd/arming")
	uav_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = uav_arm(1)
	rospy.sleep(5)

def pushingWaypoints(poi):
	print("\n----------pushingWaypoints----------")
	rospy.wait_for_service("/mavros/mission/push")
	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(0, poi)
	rospy.sleep(5)
	return

def takeoff_call(lat, long, alt):
	print("\n----------takeoff_call----------")
	takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
	resp = takeoff(0,0,lat,long, alt)
	rospy.sleep(5)
	return

def switch_modes(current_mode, next_mode): # current_mode: int, next_mode: str (http://docs.ros.org/jade/api/mavros_msgs/html/srv/SetMode.html)
	print("\n----------switch_modes----------")
	rospy.wait_for_service("/mavros/set_mode")
	modes = rospy.ServiceProxy("/mavros/set_mode", SetMode)
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


def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	readyBit = rospy.Publisher("/mavros/ugv/ready", String, queue_size=10) # Flag topic
    
	clear_pull()
	
	armingCall()	

	# switch_modes(208, "guided")

	# Take off command through service call
	takeoff_call(47.3977419, 8.5455938, 5)
	
	# Sending waypoints_push
	waypoints = [
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 47.3977419, y_long = 8.5455938, z_alt = 5),
		Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 47.3977425, y_long = 8.5455952, z_alt = 10)
	]

	# print(waypoints)
	pushingWaypoints(waypoints) # Pushes waypoints to UAV

	#switch_modes(216, "auto.mission")
	switch_modes(0, "auto.mission")
	
	# TEST3
	finishWaypoints() # Checks if waypoints are finished
	# clear_pull() # Logistic house keeping

	# TEST4
	waiting_ugv(47.3977425, 8.5458982, 5) # Checks if ugv is at lat long
	
	# TEST5
	# while True:
	# 	rospy.sleep(2)
	# 	print("Waiting for UAV to be close to next takeoff point")
	# 	if (latitude-37.1973420)<0.0001 and (longitude-(-80.5798929))<0.0001 and (altitude-529)<2:
	# 		# TODO: Check if this works
	# 		# Take off command through service call
	# 		takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
	# 		res = takeoff(0, 0, 37.1973420, -80.5798929, 10)

	# 		waypoints = [
	# 			Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 37.1973420, y_long = -80.5798929, z_alt = 10),
	# 			Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.1972726, y_long = -80.5799733, z_alt = 5),
	# 			Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 37.1971499, y_long = -80.5801173, z_alt = 5)
	# 		]
	# 		pushingWaypoints(waypoints)
	# 		break

	# TEST6
	# finishWaypoints() # Checks if waypoints are finished
	# clear_pull() # Logistic house keeping
	# waiting_ugv(37.1971499, -80.5801173, 0) # Checks if ugv is there yet
	
	# DONE
	print("EVERYTHING WORKED AS PLANNED!!!")
	rospy.spin()


if __name__ == '__main__':
	main()