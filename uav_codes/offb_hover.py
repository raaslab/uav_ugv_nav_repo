#!/usr/bin/env python
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
import numpy as np

FLIGHT_ALTITUDE = 1.0
RATE = 20  # loop rate
RADIUS = 0.5  # radius of the circle
CYCLE_S = 8  # time to complete a circle in seconds
STEPS = CYCLE_S * RATE
current_state = State()
path = []  # a list of [PositionTarget()]
# generate a simple circle using parametric equation
# note this is in ENU coordinates since mavros will convert to NED
# x right, y forward, z up.
def init_path():
    global path
    dt = 1.0 / RATE
    #print('dt is {}'.format(dt))  # check division for python 2
    dadt = 2.0 * np.pi / CYCLE_S  # omega
    r = RADIUS
    for i in range(STEPS):
        way_point = PositionTarget()
        way_point.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        way_point.type_mask = 0
        # calculate the parameter a which is an angle sweeping from -pi/2 to 3pi/2
        a = (-np.pi / 2.0) + i * (2 * np.pi / STEPS)
        sin_a = np.math.sin(a)
        cos_a = np.math.cos(a)
        # position
        way_point.position.x = r * cos_a
        way_point.position.y = r * sin_a
        way_point.position.z = FLIGHT_ALTITUDE
        # velocity
        way_point.velocity.x = -dadt * r * sin_a
        way_point.velocity.y = dadt * r * cos_a
        way_point.velocity.z = 0
        # acceleration
        way_point.acceleration_or_force.x = -dadt ** 2 * r * cos_a
        way_point.acceleration_or_force.y = -dadt ** 2 * r * sin_a
        way_point.acceleration_or_force.z = 0
        way_point.yaw = np.math.atan2(-way_point.velocity.x, way_point.velocity.y) + np.pi / 2 + np.pi / 2
        print("x: {}, y: {}, Z: {}, yaw: {}".format(way_point.position.x, way_point.position.y, way_point.position.z,
                                                    way_point.yaw / np.pi * 180))
        path.append(way_point)
    for i in range(STEPS):
        next_yaw = path[(i + 1) % STEPS].yaw
        current_yaw = path[i].yaw
        # account for wrap around +- PI
        if ((next_yaw - current_yaw) < -np.pi):
            next_yaw += 2 * np.pi
        if ((next_yaw - current_yaw) > np.pi):
            next_yaw -= 2 * np.pi
        path[i].yaw_rate = (next_yaw - current_yaw) / dt
def state_cb(msg):
    global current_state
    current_state = msg
    print('current state is {}'.format(current_state))
def main():
    rospy.init_node('tracking_circle_node', anonymous=True)
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    target_local_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(RATE)
    # wait for FCU connection
    # while ((not rospy.is_shutdown()) and (not current_state.connected)):
    #     rate.sleep()
    #     rospy.loginfo("connecting to FCU...")
    # keep this pose constant, home position
    position_home = PositionTarget()
    position_home.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_home.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                              PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                              PositionTarget.IGNORE_YAW_RATE
    position_home.position.x = 0
    position_home.position.y = 0
    position_home.position.z = FLIGHT_ALTITUDE
    position_home.velocity.x = 0
    position_home.velocity.y = 0
    position_home.velocity.z = 0
    position_home.acceleration_or_force.x = 0
    position_home.acceleration_or_force.y = 0
    position_home.acceleration_or_force.z = 0
    # path starts pointing 0 degrees right of forward (y axis in ENU)
    # ENU yaw is angle left (CCW) of X axis which is to the right.
    # hence yaw here is 90 degrees
    # plus 90 gets us from x axis as 0 to y axis as 0
    position_home.yaw = 3.14/2
    position_home.yaw_rate = 0
    
    ######
    position_dir = PositionTarget()
    position_dir.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_dir.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                              PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                              PositionTarget.IGNORE_YAW_RATE
    position_dir.position.x = 0
    position_dir.position.y = 0
    position_dir.position.z = FLIGHT_ALTITUDE
    position_dir.velocity.x = 0
    position_dir.velocity.y = 0
    position_dir.velocity.z = 0
    position_dir.acceleration_or_force.x = 0
    position_dir.acceleration_or_force.y = 0
    position_dir.acceleration_or_force.z = 0
    # path starts pointing 0 degrees right of forward (y axis in ENU)
    # ENU yaw is angle left (CCW) of X axis which is to the right.
    # hence yaw here is 90 degrees
    # plus 90 gets us from x axis as 0 to y axis as 0
    position_dir.yaw = 3.14/2
    position_dir.yaw_rate = 0
    
    ######
    
    init_path()
    # send a few setpoints before starting
    for i in range(100):
        target_local_pub.publish(position_home)
        rate.sleep()
    rospy.loginfo("waiting for offboard mode")
    while not rospy.is_shutdown():
        target_local_pub.publish(position_home)
        rate.sleep()
        if (current_state.mode == "OFFBOARD") and current_state.armed:
            break
    # give the system 2 seconds to get to home position
    i = 2 * RATE
    rospy.loginfo("going home")
    while (not rospy.is_shutdown()) and (i > 0):
        i -= 1
        target_local_pub.publish(position_home)
        rate.sleep()
    # now begin circle
    i = 0
    rospy.loginfo("following path")
    while not rospy.is_shutdown():
        # Update timestamp and publish position target
        path[i].header.stamp = rospy.Time.now()
        # target_local_pub.publish(path[i])
        target_local_pub.publish(position_dir)
        i += 1
        if i >= STEPS:
            i = 0
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

