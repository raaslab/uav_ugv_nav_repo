#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from ugv_goals import get_ugv_goals
from math import atan2, sqrt

# Publishers
ugv_goal_pub = rospy.Publisher('/ugv_goal_status', String, queue_size=10)
ugv_velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Parameters
rospy.set_param('/uav_goal_reached', False)

# Global Variables
current_goal_index = 0
uav_goal_status = {}


def ugv_goal_callback(msg, uav_id):
    global uav_goal_status
    goal_status = msg.data
    print("Received goal status:", goal_status)
    if goal_status.startswith("rendezvous_reached") and goal_status.endswith(str(uav_id)):
        rospy.set_param('/uav_goal_reached', True)
        print("UGV received 'reached' status from UAV", uav_id)
        uav_goal_status[uav_id] = True
    elif goal_status == "reached" or goal_status == "in progress":
        rospy.set_param('/uav_goal_reached', False)
        uav_goal_status[uav_id] = False


def calculate_distance_and_angle(ugv_position, goal):
    inc_x = goal[0] - ugv_position.x
    inc_y = goal[1] - ugv_position.y
    angle_to_goal = atan2(inc_y, inc_x)
    distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)
    return angle_to_goal, distance_to_goal


def move_towards_goal(angle_to_goal, distance_to_goal, theta):
    if abs(angle_to_goal - theta) > 0.3:
        publish_ugv_velocity(0.0, 0.8)
    elif distance_to_goal > 0.5:
        velocity = distance_to_goal * 0.8
        publish_ugv_velocity(velocity, 0.0)


def check_goal_reached():
    rospy.set_param('/uav_goal_reached', True)
    


def handle_rendezvous(goal):
    rendezvous_uav_id = goal[4]
    while not uav_goal_status[rendezvous_uav_id]:
        rospy.sleep(0.1)

    uav_goal_status[rendezvous_uav_id] = False
    publish_goal_status("rendezvous_reached")


def update_goal_index():
    global current_goal_index
    current_goal_index += 1


def process_next_goal(goals):
    if current_goal_index >= len(goals):
        rospy.loginfo("UGV reached the last goal. Shutting down...")
        rospy.signal_shutdown("UGV reached the last goal")
    else:
        rospy.set_param('/uav_goal_reached', False)
        publish_goal_status("rendezvous_reached")


def ugv_position_callback(msg, goals):
    global current_goal_index

    ugv_position = msg.pose.pose.position
    goal = goals[current_goal_index]
    goal_type = goal[3]
    rot_q = msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    angle_to_goal, distance_to_goal = calculate_distance_and_angle(ugv_position, goal)

    move_towards_goal(angle_to_goal, distance_to_goal, theta)

    if distance_to_goal < 0.5:
        check_goal_reached()
        if goal_type=='rv':
            handle_rendezvous(goal)
        update_goal_index()
        process_next_goal(goals)


def initialize_node():
    rospy.init_node('ugv_waypoint', anonymous=True)


def run_ugv_waypoint():
    initialize_node()
    subscribe_uav_goal_status()
    goals = get_ugv_goals()
    subscribe_ugv_position(goals)

    rate = rospy.Rate(0.20)

    for goal in goals:
        while not rospy.is_shutdown():
            publish_ugv_velocity(0.8, 0.1)
            rospy.loginfo("Published UGV velocity: Linear=%.2f, Angular=%.2f", 0.8, 0.1)
            publish_goal_status("in progress")

            if goal[3] == "rv":
                wait_for_uav_goal_reached()

                publish_goal_status("rendezvous_reached")
                break

            if goal[3] == "wp":
                break

        rospy.loginfo("UGV reached goal: %s", str(goal))
        publish_goal_status("reached")
        rospy.sleep(0.1)

    rate.sleep()


def subscribe_uav_goal_status():
    uav_ids = [1, 2]
    uav_goal_subs = []  # Define the list to hold the UAV goal subscribers
    for uav_id in uav_ids:
        topic = '/uav_goal_status/uav{}'.format(uav_id)
        uav_goal_sub = rospy.Subscriber(topic, String, lambda msg, uav_id=uav_id: ugv_goal_callback(msg, uav_id))
        uav_goal_subs.append(uav_goal_sub)
    return uav_goal_subs


def subscribe_ugv_position(goals):
    rospy.Subscriber('/odometry/filtered', Odometry, lambda msg: ugv_position_callback(msg, goals))


def wait_for_uav_goal_reached():
    while not rospy.get_param('/uav_goal_reached'):
        rospy.sleep(0.1)


def publish_goal_status(goal_status):
    ugv_goal_pub.publish(goal_status)


def publish_ugv_velocity(linear_x, angular_z):
    ugv_velocity_pub.publish(create_twist(linear_x, angular_z))


def create_twist(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    return twist


if __name__ == '__main__':
    try:
        run_ugv_waypoint()
    except rospy.ROSInterruptException:
        pass
