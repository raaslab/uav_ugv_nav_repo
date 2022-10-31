""" Four Point Navigation for UAV (m500 drone) using GPS with Apriltag Detection.
"""

#!/usr/bin/env python
from apriltag_ros import msg
import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from apriltag_ros.msg import *
from mavros_msgs.msg import State

#global variables
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False
current_state = State()
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
	# print("I am in callback..")
	global tag_det
	global tag_det_id

	if not msg.detections:
		print("No Tag Detected!!!!!!")
		tag_det = False
	else:
        # print("-------------Tag Detected!--------------")
		tag_det = True
		tag_det_id = msg.detections[0].id
	return

def tag_check():
    # rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    for i in range(10):
        if tag_det:
            print("-------------Tag Detected!--------------")
            print(tag_det_id)
            land_call(latitude, longitude, 0)
            break
            # rospy.sleep(1)

def state_cb(msg):
    global current_state
    current_state = msg
    print('current state is {}'.format(current_state))

def main():
    rospy.init_node('wayPoint')
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)


    # Define waypoints --
    _lat_1 = 47.3977419
    _long_1 = 8.5456938

    _lat_2 = 47.397743
    _long_2 = 8.5456789
    
    _lat_3 = 47.3977782
    _long_3 = 8.5456333

    ######################### STEP 1 ########################
    clear_pull()
    armingCall()	

    start_lat = latitude
    start_long = longitude

    # Take off command 
    takeoff_call(latitude, longitude, 2)

    # Sending waypoints_push inside F3 cage
    waypoints1 = [
        Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = _lat_1, y_long = _long_1, z_alt = 2),
        Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = _lat_1, y_long = _long_1, z_alt = 2)
        ]

    # print(waypoints)
    pushingWaypoints(waypoints1) # Pushes waypoints to UAV

    #switch_modes(216, "auto.mission")
    switch_modes(0, "auto.mission")

    finishWaypoints() # Checks if waypoints are finished

    # tag_check()

    print("--------------- Step 1 Complete --------------")

    ######################### STEP 2 #########################

    while not current_state.armed:	
        armingCall()	
        takeoff_call(latitude, longitude, 2)

    # Sending waypoints_push
    waypoints2 = [
        Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = _lat_2, y_long = _long_2, z_alt = 2),
        Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = _lat_2, y_long = _long_2, z_alt = 2)
    ]

    # print(waypoints)
    pushingWaypoints(waypoints2) # Pushes waypoints to UAV

    #switch_modes(216, "auto.mission")
    switch_modes(0, "auto.mission")

    # TEST3
    finishWaypoints() # Checks if waypoints are finished
    # clear_pull() # Logistic house keeping

    # Hover
    rospy.sleep(10)

    tag_check()
    # land_call(latitude, longitude, 0)

    print("--------------- Step 2 Complete --------------")

    # rospy.spin()

    ######################### STEP 3 #####################
    clear_pull()

    while not current_state.armed:	
        armingCall()	
        takeoff_call(latitude, longitude, 2)

    # Sending waypoints_push
    waypoints3 = [
        Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = _lat_3, y_long = _long_3, z_alt = 2),
        Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = _lat_3, y_long = _long_3, z_alt = 2)
    ]

    # print(waypoints)
    pushingWaypoints(waypoints3) # Pushes waypoints to UAV

    #switch_modes(216, "auto.mission")
    switch_modes(0, "auto.mission")

    # TEST3
    finishWaypoints() # Checks if waypoints are finished
    # clear_pull() # Logistic house keeping

    # Hover
    rospy.sleep(10)

    tag_check()
    # land_call(latitude, longitude, 0)

    print("--------------- Step 3 Complete --------------")

    ######################### STEP 4 #####################

    while not current_state.armed:	
        armingCall()	
        takeoff_call(latitude, longitude, 2)

    # Sending waypoints_push
    waypoints4 = [
        Waypoint(frame = 3, command = 16, is_current = 0, autocontinue = True, param1 = 5, x_lat = start_lat, y_long = start_long, z_alt = 2),
        Waypoint(frame = 3, command = 16, is_current = 1, autocontinue = True, param1 = 5, x_lat = start_lat, y_long = start_long, z_alt = 2)
    ]

    # print(waypoints)
    pushingWaypoints(waypoints4) # Pushes waypoints to UAV

    #switch_modes(216, "auto.mission")
    switch_modes(0, "auto.mission")

    # TEST3
    finishWaypoints() # Checks if waypoints are finished
    # clear_pull() # Logistic house keeping

    # Hover
    rospy.sleep(10)

    # tag_check()
    land_call(latitude, longitude, 0)

    print("--------------- Step 4 Complete --------------")


if __name__ == '__main__':
	main()