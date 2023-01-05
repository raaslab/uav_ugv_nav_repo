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

def armingCall():
	print("\n----------armingCall----------")
	rospy.wait_for_service("mavros/cmd/arming")
	uav_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	resp = uav_arm(1)
	rospy.sleep(5)

def pushingWaypoints(poi):
	print("\n----------pushingWaypoints----------")
	rospy.wait_for_service("/mavros/mission/push")
	waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
	resp = waypoint_push(poi)
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

# ---------------------------------------------------------------------------------------
def land_call(lat, long, alt):
	print("\n----------land_call----------")
	land = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
	resp = land(0,0,lat,long, alt)
	rospy.sleep(5)
	return

def setOffboardMode(self):
	rospy.wait_for_service('mavros/set_mode')
	try:
		flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
		flightModeService(custom_mode='OFFBOARD')
	except (rospy.ServiceException):
		print ("service set_mode call failed: %s. Offboard Mode could not be set.")
	rospy.sleep(10)
		
		
# =======================================================================================
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0


def main():
	# rospy.init_node('wayPoint')
	# rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	# rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)
	# readyBit = rospy.Publisher("/mavros/ugv/ready", String, queue_size=10) # Flag topic
    
	#clear_pull()
	
	rospy.init_node('setpoint_node', anonymous=True)
	cnt = Controller()

	rate = rospy.Rate(20.0)
	sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

	armingCall()	

	# Takeoff command for Gazebo environment -
	# takeoff_call(47.3977415, 8.5455952, 2)	

	k=0
	while k < 10:
		sp_pub.publish(cnt.sp)
		rate.sleep()
		k=k+1
	
	setOffboardMode(self=True)

	# switch_modes(208, "OFFBOARD")

	# # Take off command through service call
	# #takeoff_call(-35.363238, 149.164230, 10)

	# takeoff_call(47.3977415, 8.5455952, 2)

	# switch_modes(208, "OFFBOARD")

	# print("\n---------Hovering------------")
	# rospy.sleep(20)

	# land_call(47.3977415, 8.5455952, 2)

	# print(latitude)
	# print(longitude)
	
	# print("Everything Worked Aa Planned!!")
	# rospy.spin()


if __name__ == '__main__':
	main()
