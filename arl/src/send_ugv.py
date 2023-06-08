#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

rospy.set_param('/uav_goal_reached', False)  # Initialize the UAV goal reached parameter
current_goal_index = 0

# Callback function to receive UAV's goal status
def ugv_goal_callback(msg):
    goal_status = msg.data

    if goal_status == "reached":
        rospy.set_param('/uav_goal_reached', True)
    elif goal_status == "in progress":
        rospy.set_param('/uav_goal_reached', False)

# Callback function to receive UGV's position
# def ugv_position_callback(msg,goals):
#     ugv_position = msg.pose.pose.position
#     for goal in goals:
#         goal_x, goal_y, goal_z = goal
#         # Check if UGV has reached the goal position (for simplicity, using a fixed threshold of 0.1)
#         if abs(ugv_position.x - goal_x) < 0.1 and abs(ugv_position.y - goal_y) < 0.1:
#             rospy.set_param('/ugv_goal_reached', True)
#         else:
#             rospy.set_param('/ugv_goal_reached', False)
def ugv_position_callback(msg, goals,ugv_velocity_pub):
    global current_goal_index
    ugv_position = msg.pose.pose.position
    goal = goals[current_goal_index]
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    inc_x = goal[0] - ugv_position.x
    inc_y = goal[1] - ugv_position.y

    angle_to_goal = atan2(inc_y, inc_x)
    distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)

    if abs(angle_to_goal - theta) > 0.1:
        # Rotate towards the goal
        ugv_velocity_pub.publish(create_twist(0.0, 0.3))
    elif distance_to_goal > 0.2:
        # Move towards the goal
        ugv_velocity_pub.publish(create_twist(0.7, 0.0))
    else:
        # Stop
        ugv_velocity_pub.publish(create_twist(0.0, 0.0))

    # Check if UGV has reached the goal position
    if distance_to_goal < 0.1:
        rospy.set_param('/ugv_goal_reached', True)
    else:
        rospy.set_param('/ugv_goal_reached', False)


def ugv_waypoint():
    rospy.init_node('ugv_waypoint', anonymous=True)
    ugv_goal_pub = rospy.Publisher('/ugv_goal_status', String, queue_size=10)
    uav_goal_sub = rospy.Subscriber('/uav_goal_status', String, ugv_goal_callback)
    ugv_velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    ugv_position_sub = rospy.Subscriber('/odometry/filtered', Odometry, lambda msg: ugv_position_callback(msg, goals,ugv_velocity_pub))

    # Specify the three goal positions
    goals = [(2, 2, 0), (3, 4, 0), (5, 5, 0)]

    rate = rospy.Rate(10)  # 10 Hz

    for goal in goals:
        # Move UGV to the current goal position
        while not rospy.is_shutdown():
            # Publish UGV's current velocity
            ugv_velocity_pub.publish(create_twist(0.2, 0.5))
            rospy.loginfo("Published UGV velocity: Linear=%.2f, Angular=%.2f", 0.2, 0.5)
            
            # Continuously publish goal status
            ugv_goal_pub.publish("in progress")
            
            # Wait for some time
            rospy.sleep(1.0)

            # Check if UAV's goal status indicates it has reached the current goal position
            uav_goal_status = rospy.get_param('/uav_goal_reached')
            # If so, break the loop and proceed to the next goal
            if uav_goal_status:
                break

        # Publish goal status as "reached"
        ugv_goal_pub.publish("reached")
        rospy.loginfo("UGV reached goal: %s", str(goal))
        rate.sleep()

def create_twist(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    return twist

if __name__ == '__main__':
    try:
        ugv_waypoint()
    except rospy.ROSInterruptException:
        pass
