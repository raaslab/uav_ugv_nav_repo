#!/usr/bin/env python


import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

x = 0.0
y = 0.0
theta = 0.0
goal_reached = False

def newOdom(msg):
    global x, y, theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def goal_reached_callback(msg):
    global goal_reached
    goal_reached = msg.data

rospy.init_node("ugv_waypoint_publisher")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
goal_reached_pub = rospy.Publisher("/ugv_goal_reached", Bool, queue_size=1)  # Add publisher for goal reached
rospy.Subscriber("/uav_goal_reached", Bool, goal_reached_callback)

speed = Twist()

r = rospy.Rate(20)

goal_positions = [
    #SSSPoint(0,0,0),
    Point(10, 10, 0),
    Point(3, 2, 0),
    Point(10, 4, 0)
]
current_goal_index = 0

while not rospy.is_shutdown():
    if goal_reached:
        goal = goal_positions[current_goal_index]

        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2(inc_y, inc_x)
        distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)

        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif distance_to_goal > 0.2:
            speed.linear.x = 0.7
            speed.angular.z = 0.0
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            goal_reached_pub.publish(Bool(True))  # Publish goal reached message
            goal_reached = False
            current_goal_index += 1

            if current_goal_index >= len(goal_positions):
                rospy.loginfo("All goals reached")
                break

            goal_reached_pub.publish(Bool(True))  # Publish goal reached message
            rospy.loginfo("Moving to next goal...")
            goal_reached = False

    pub.publish(speed)
    r.sleep()

pub.publish(speed)
r.sleep()


