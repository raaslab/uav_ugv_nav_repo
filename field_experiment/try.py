#!/usr/bin/env python
import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped

#global variables
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False

def armingCall():
	print("\n----------armingCall----------")
	rospy.wait_for_service("mavros/cmd/arming")
	uav_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = uav_arm(1)
	rospy.sleep(5)

def takeoff_call(lat, long, alt):
	print("\n----------takeoff_call----------")
	takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
	resp = takeoff(0,0,lat,long, alt)
	rospy.sleep(5)
	return

def waypoint_callback(data):
	global last_waypoint00
	#print("\n----------\nwaypoint_callback")
	rospy.loginfo("Got waypoint: %s", data)
	if len(data.waypoints) != 0:							#if waypoint list is not empty
		rospy.loginfo("is_current: %s", data.waypoints[len(data.waypoints)-1].is_current)
		last_waypoint = data.waypoints[len(data.waypoints)-1].is_current	#checks status of "is_current" for last waypoint

def globalPosition_callback(data):
	#print("\n----------\nglobalPosition_callback")
	global latitude
	global longitude
	global altitude
	latitude = data.latitude
	longitude = data.longitude
	altitude = data.altitude

def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)

	rate = rospy.Rate(20.0)
	sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

	armingCall()	

	# Takeoff command for Gazebo environment -
	takeoff_call(47.3977415, 8.5455952, 2)
	# takeoff_call(47.3977415, 8.5455952, 2)

	#Clearing waypoints
	print("\n----------CLEARING----------")
	rospy.wait_for_service("/mavros/mission/clear")
	print("Clearing Waypoints!!!")
	waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
	resp = waypoint_clear()
	print(resp)
	rospy.sleep(5)
	#Call waypoints_pull
	print("\n----------PULLING----------")
	rospy.wait_for_service("/mavros/mission/pull")
	print("Calling Waypoint_pull Service")
	waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
	resp = waypoint_pull()
	print(resp)
	rospy.sleep(5)
	#Sending waypoints_push
	print("\n----------PUSHING----------")
	print("Waiting for MAVROS service...")
	rospy.wait_for_service("/mavros/mission/push")
		
	waypoints = [ 
		Waypoint(frame = 3, command = 22, is_current = True, autocontinue = True, param1 = 5, x_lat = 47.3977415, y_long = 8.5455952, z_alt = 5),
		Waypoint(frame = 3, command = 22, is_current = True, autocontinue = True, param1 = 5, x_lat = 47.3978000, y_long = 8.5455952, z_alt = 5),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 47.3979000, y_long = 8.5455952, z_alt = 5)]

	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(0, waypoints)
	print(resp)
	rospy.sleep(5)

	while True:						#waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		print("WAITING for last_waypoint == True")
		if last_waypoint == True:			#if last_waypoint is in the process of being visited
			while True:
				rospy.sleep(2)
				print("WAITING for last_waypoint == False")
				if last_waypoint == False:	#if last_waypoint has been visited (due to previous constraint)
					break
			break

	# k=0
	# while k < 10:
	# 	sp_pub.publish(cnt.sp)
	# 	rate.sleep()
	# 	k=k+1

	# setOffboardMode(self=True)


if __name__ == '__main__':
	main()
