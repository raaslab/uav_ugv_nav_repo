#!/usr/bin/env python

from apriltag_ros import msg
import rospy
# import mavros
# import sensor_msgs
# import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from apriltag_ros.msg import *

#global variables
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False

tag_det = False
tag_det_id = 100

land_check = False

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

def tag_callback(msg):
    # for i in range(10):
    if not msg.detections:
        print("No Tag Detected!!!!!!")
    else:
        print("-------------Tag Detected!--------------")
        tag_det = True
        tag_det_id = msg.detections[0].id
        
        if tag_det:
            print(tag_det_id)
            land_call(latitude, longitude, 0)
            land_check = True
        #     break
            # rospy.sleep(1)
        # if tag_det:
    #     print(tag_det_id)
    #     land_call(latitude, longitude, 0)
        
    # rospy.sleep(10)
    # land_call(latitude, longitude, 0)
    # break
    return

def tag_check():
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    # if tag_det:
    #     print(tag_det_id)
    #     land_call(latitude, longitude, 0)
    return 

def main():
    rospy.init_node('wayPoint')
    rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
    

    ######################### STEP 1 ###########################################################33
    clear_pull()
    armingCall()	

    start_lat = latitude
    start_long = longitude
    

	# # Take off command for simulation
	# takeoff_call(47.3977419, 8.5455938, 5)

	# Take off command 
    takeoff_call(latitude, longitude, 2)
	
	# # Take off command inside F3 cage
	# takeoff_call(38.973593863069446, -76.92190286766103, 1)

	# Sending waypoints_push
    waypoints = [
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 47.3977419, y_long = 8.5455938, z_alt = 2),
		Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 47.3977419, y_long = 8.5456938, z_alt = 2)
	]


	# Sending waypoints_push inside F3 cage
	# waypoints = [
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973550, y_long = -76.921744, z_alt = 2),
	# 	Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 38.973640, y_long = -76.921758, z_alt = 3),
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973816, y_long = -76.921915, z_alt = 3),
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973757, y_long = -76.922117, z_alt = 3),
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973595, y_long = -76.921888, z_alt = 3)
	# ]

	# print(waypoints)
    pushingWaypoints(waypoints) # Pushes waypoints to UAV

	#switch_modes(216, "auto.mission")
    switch_modes(0, "auto.mission")
	
	# TEST3
    finishWaypoints() # Checks if waypoints are finished
	# clear_pull() # Logistic house keeping

	# Hover
    # rospy.sleep(5)
    # rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    # print(tag_msgs)

    for i in range(10):
        tag_check()
        if land_check:
            break
        rospy.sleep(1)

    # rospy.sleep(20)
    print("--------------- Step 1 Complete --------------")

    ######################### STEP 2 ###########################################################33
    clear_pull()
    armingCall()	

	# # Take off command for simulation
	# takeoff_call(47.3977419, 8.5455938, 5)

	# Take off command 
    takeoff_call(latitude, longitude, 2)
	
	# # Take off command inside F3 cage
	# takeoff_call(38.973593863069446, -76.92190286766103, 1)

	# Sending waypoints_push
    waypoints2 = [
		Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = start_lat, y_long = start_long, z_alt = 2),
        Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = start_lat, y_long = start_long, z_alt = 2)
	]


	# Sending waypoints_push inside F3 cage
	# waypoints = [
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973550, y_long = -76.921744, z_alt = 2),
	# 	Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = 38.973640, y_long = -76.921758, z_alt = 3),
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973816, y_long = -76.921915, z_alt = 3),
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973757, y_long = -76.922117, z_alt = 3),
	# 	Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = 38.973595, y_long = -76.921888, z_alt = 3)
	# ]

	# print(waypoints)
    pushingWaypoints(waypoints2) # Pushes waypoints to UAV

	#switch_modes(216, "auto.mission")
    switch_modes(0, "auto.mission")
	
	# TEST3
    finishWaypoints() # Checks if waypoints are finished
	# clear_pull() # Logistic house keeping

	# Hover
    rospy.sleep(10)

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    # # print(tag_msgs)
    # for i in range(100):
    #     if tag_det:
    #         print(tag_det_id)
    #         land_call(latitude, longitude, 0)
    #         break
    #     rospy.sleep(1)

    # rospy.sleep(20)
    print("--------------- Step 2 Complete --------------")

    rospy.spin()

if __name__ == '__main__':
	main()