#!/usr/bin/env python3
import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped
import re
# Global variables
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

def clear_pull(uav_id):
    print("\n----------clear_pull----------")
    # Clearing waypoints
    rospy.wait_for_service("/uav{}/mavros/mission/clear".format(uav_id))
    waypoint_clear = rospy.ServiceProxy("/uav{}/mavros/mission/clear".format(uav_id), WaypointClear)
    resp = waypoint_clear()
    rospy.sleep(5)
    # Call waypoints_pull
    rospy.wait_for_service("/uav{}/mavros/mission/pull".format(uav_id))
    waypoint_pull = rospy.ServiceProxy("/uav{}/mavros/mission/pull".format(uav_id), WaypointPull)
    resp = waypoint_pull()
    rospy.sleep(5)
    return

def armingCall(uav_id):
    print("\n----------armingCall----------")
    rospy.wait_for_service("/uav{}/mavros/cmd/arming".format(uav_id))
    uav_arm = rospy.ServiceProxy("/uav{}/mavros/cmd/arming".format(uav_id), CommandBool)
    resp = uav_arm(1)
    rospy.sleep(5)

def takeoff_call(uav_id, lat, long, alt):
    print("\n----------takeoff_call----------")
    takeoff = rospy.ServiceProxy("/uav{}/mavros/cmd/takeoff".format(uav_id), CommandTOL)
    resp = takeoff(0, 0, lat, long, alt)
    rospy.sleep(5)
    return

def switch_modes(uav_id, current_mode, next_mode):
    # current_mode: int, next_mode: str (http://docs.ros.org/jade/api/mavros_msgs/html/srv/SetMode.html)
    print("\n----------switch_modes----------")
    rospy.wait_for_service("/uav{}/mavros/set_mode".format(uav_id))
    modes = rospy.ServiceProxy("/uav{}/mavros/set_mode".format(uav_id), SetMode)
    resp = modes(custom_mode=next_mode)
    rospy.sleep(5)
    return

def land_call(uav_id, lat, long, alt):
    print("\n----------land_call----------")
    land = rospy.ServiceProxy("/uav{}/mavros/cmd/land".format(uav_id), CommandTOL)
    resp = land(0, 0, lat, long, alt)
    rospy.sleep(5)
    return

def setOffboardMode(uav_id):
    rospy.wait_for_service('/uav{}/mavros/set_mode'.format(uav_id))
    try:
        flightModeService = rospy.ServiceProxy('/uav{}/mavros/set_mode'.format(uav_id), mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='OFFBOARD')
    except rospy.ServiceException:
        print("service set_mode call failed: %s. Offboard Mode could not be set.")
    rospy.sleep(10)

class Controller:
    def __init__(self):
        self.state = State()
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)
        self.sp.coordinate_frame = 1
        self.ALT_SP = 3.0
        self.sp.position.z = self.ALT_SP
        self.STEP_SIZE = 2.0
        self.FENCE_LIMIT = 5.0
        self.local_pos = Point(0.0, 0.0, 3.0)
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0


def main(uav_id):
    rospy.init_node('multi_uav_setpoint_node_' + str(uav_id), anonymous=True)

    # Arm all UAVs
    
    armingCall(uav_id)

    # Switch all UAVs to OFFBOARD mode
    
    switch_modes(uav_id, 208, "OFFBOARD")

    # Wait for a common trigger message to start all UAVs together
    rospy.wait_for_message('/start_uavs', String)

    # Set OFFBOARD mode for all UAVs simultaneously
    
    setOffboardMode(uav_id)

    rospy.spin()

if __name__ == '__main__':
    node_name = rospy.get_name()  # Get the full node name
    match = re.search(r'\d+', node_name)
    uav_id = match.group()
    main(uav_id)
