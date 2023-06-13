#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

rospy.set_param('/uav_goal_reached', False)  # Initialize the UAV goal reached parameter
current_goal_index = 0
uav_goal_status = False  # Initialize the UAV goal status

# Callback function to receive UAV's goal status
def ugv_goal_callback(msg):
    global uav_goal_status  # Update the global uav_goal_status variable
    goal_status = msg.data
    print("Received goal status:", goal_status)
    if goal_status == "reached":
        rospy.set_param('/uav_goal_reached', True)
        print("UGV received 'reached' status from UAV")
        uav_goal_status = True  # Set the UAV goal status to True
    elif goal_status == "in progress":
        rospy.set_param('/uav_goal_reached', False)
        uav_goal_status = False  # Set the UAV goal status to False

# Callback function to receive UGV's position
def ugv_position_callback(msg, goals, ugv_velocity_pub, ugv_goal_pub):
    global current_goal_index
    global uav_goal_status  # Add this line to access the global uav_goal_status variable

    ugv_position = msg.pose.pose.position
    print("UGV current position: x={}, y={}, z={}".format(ugv_position.x, ugv_position.y, ugv_position.z))
    
    goal = goals[current_goal_index]
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    inc_x = goal[0] - ugv_position.x
    inc_y = goal[1] - ugv_position.y
    print("current_goal_index:", current_goal_index)
    print("goals:", goals)
    angle_to_goal = atan2(inc_y, inc_x)
    distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)

    if abs(angle_to_goal - theta) > 0.5:
        # Rotate towards the goal
        ugv_velocity_pub.publish(create_twist(0.0, 0.1))
    elif distance_to_goal > 0.2:
        # Move towards the goal
        velocity = distance_to_goal * 0.8  # Adjust the scaling factor as desired
        ugv_velocity_pub.publish(create_twist(velocity, 0.0))
    else:
        # Stop
        ugv_velocity_pub.publish(create_twist(0.0, 0.0))

        # Print the goal position
    print("###############Current Goal: x={}, y={}".format(goal[0], goal[1]))
    print("********************UGV distance to goal is: {}".format(distance_to_goal))

    # Check if UGV has reached the goal position
    if sqrt((ugv_position.x - goal[0]) ** 2 + (ugv_position.y - goal[1]) ** 2) < 0.5:
        rospy.set_param('/ugv_goal_reached', True)
        ugv_goal_pub.publish("reached")  # Publish goal status as "reached"

        # Wait for UAV to reach the goal position
        while not uav_goal_status:
            rospy.sleep(0.1)

        current_goal_index += 1  # Update the current goal index
        uav_goal_status = False  # Reset the UAV goal status
        if current_goal_index >= len(goals):
            current_goal_index = 0

    else:
        rospy.set_param('/ugv_goal_reached', False)
        ugv_goal_pub.publish("in progress")  # Publish goal status as "in progress"


def ugv_waypoint():
    global uav_goal_status
    rospy.init_node('ugv_waypoint', anonymous=True)
    ugv_goal_pub = rospy.Publisher('/ugv_goal_status', String, queue_size=10)
    uav_goal_sub = rospy.Subscriber('/uav_goal_status', String, ugv_goal_callback)
    ugv_velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Specify the three goal positions
    goals = [(5, 0, 0), (8, 0, 0), (10, 0, 0)]

    ugv_position_sub = rospy.Subscriber('/odometry/filtered', Odometry, lambda msg: ugv_position_callback(msg, goals,ugv_velocity_pub,ugv_goal_pub))

    rate = rospy.Rate(0.20)  # 5 Hz

    for goal in goals:
        # Move UGV to the current goal position
        while not rospy.is_shutdown():
            # Publish UGV's current velocity
            ugv_velocity_pub.publish(create_twist(0.8, 0.1))
            rospy.loginfo("Published UGV velocity: Linear=%.2f, Angular=%.2f", 0.8, 0.1)
            
            # Continuously publish goal status
            ugv_goal_pub.publish("in progress")
            
            # # Wait for some time
            # rospy.sleep(1.0)

            # Check if UAV's goal status indicates it has reached the current goal position
            # If so, break the loop and proceed to the next goal
            if uav_goal_status:
                break
            rate.sleep() # Control the publishing frequency
        rospy.loginfo("UGV reached goal: %s", str(goal))
        # Publish goal status as "reached"
        ugv_goal_pub.publish("reached")
        
        # Add a delay between iterations to control the publishing frequency
        rospy.sleep(5.0)
    
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
