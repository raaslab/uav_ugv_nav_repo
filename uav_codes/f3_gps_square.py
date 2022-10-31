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
flight_altitude = 3   # Check altitude value before experiments

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

def land_call(lat, long, alt):
	print("\n----------land_call----------")
	land = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
	resp = land(0,0,lat,long, alt)
	rospy.sleep(5)
	return


def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	readyBit = rospy.Publisher("/mavros/ugv/ready", String, queue_size=10) # Flag topic
    
	clear_pull()
	
	armingCall()	

	# # Take off command for simulation
	# takeoff_call(47.3977419, 8.5455938, 5)

	# Take off command 
	takeoff_call(latitude, longitude, flight_altitude)
	
	# # Take off command inside F3 cage
	# takeoff_call(38.973593863069446, -76.92190286766103, 1)

	# Sending waypoints_push
	# waypoints = [
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 47.3977419, y_long = 8.5455938, z_alt = 2),
	# 	Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 47.3977419, y_long = 8.5456938, z_alt = 2)
	# ]


	# Sending waypoints_push inside F3 cage
	waypoints = [
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973568, y_long = -76.921832, z_alt = 3),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973630, y_long = -76.921709, z_alt = 3),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973899, y_long = -76.921993, z_alt = 3),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973749, y_long = -76.922084, z_alt = 3),
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973568, y_long = -76.921832, z_alt = 3)
		#Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973757, y_long = -76.922117, z_alt = 1),
		#Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973595, y_long = -76.921888, z_alt = 1)
	]

	# print(waypoints)
	pushingWaypoints(waypoints) # Pushes waypoints to UAV

	#switch_modes(216, "auto.mission")
	switch_modes(0, "auto.mission")
	
	# TEST3
	finishWaypoints() # Checks if waypoints are finished
	# clear_pull() # Logistic house keeping

	# Hover
	rospy.sleep(10)

	# Land
	land_call(latitude, longitude, 0)

	# # Land inside F3 cage
	# land_call(38.973593863069446, -76.92190286766103, 0)
	
	# DONE
	print("EVERYTHING WORKED AS PLANNED!!!")
	rospy.spin()

if __name__ == '__main__':
	main()
