#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from ugv_goals import get_ugv_goals
from math import atan2, sqrt
ugv_goal_pub = rospy.Publisher('/ugv_goal_status', String, queue_size=10)
rospy.set_param('/uav_goal_reached', False)  # Initialize the UAV goal reached parameter
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


# Callback function to receive UGV's position
def ugv_position_callback(msg, goals, ugv_velocity_pub, ugv_goal_pub):
    global current_goal_index
    global uav_goal_status  # Add this line to access the global uav_goal_status variable

    ugv_position = msg.pose.pose.position
    #print("UGV current position: x={}, y={}, z={}".format(ugv_position.x, ugv_position.y, ugv_position.z))
    goal = goals[current_goal_index]
    goal_type = goal[3]  # Get the type of the current goal
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    inc_x = goal[0] - ugv_position.x
    inc_y = goal[1] - ugv_position.y
    angle_to_goal = atan2(inc_y, inc_x)
    distance_to_goal = sqrt(inc_x ** 2 + inc_y ** 2)

    if abs(angle_to_goal - theta) > 0.3:
        # Rotate towards the goal
        ugv_velocity_pub.publish(create_twist(0.0, 0.8))
    elif distance_to_goal > 0.5:
        # Move towards the goal
        velocity = distance_to_goal * 0.8  # Adjust the scaling factor as desired
        ugv_velocity_pub.publish(create_twist(velocity, 0.0))

    # Check if UGV has reached the goal position
    if sqrt((ugv_position.x - goal[0]) ** 2 + (ugv_position.y - goal[1]) ** 2) < 0.5:
        rospy.set_param('/ugv_goal_reached', True)
        ugv_goal_pub.publish("reached")  # Publish goal status as "reached"

        if goal_type == "rv":
            # Wait for UAV to reach the goal position
            rendezvous_uav_id = goal[4]
            while not uav_goal_status[rendezvous_uav_id]:
                rospy.sleep(0.1)

            # Reset the UAV goal status
            uav_goal_status[rendezvous_uav_id] = False
            ugv_goal_pub.publish("rendezvous_reached") 

        current_goal_index += 1  # Update the current goal index

        if current_goal_index >= len(goals):
            rospy.loginfo("UGV reached the last goal. Shutting down...")
            rospy.signal_shutdown("UGV reached the last goal")
        else:
            rospy.set_param('/ugv_goal_reached', False)
            ugv_goal_pub.publish("in progress")  # Publish goal status as "in progress"

def ugv_waypoint():
    global uav_goal_status
    rospy.init_node('ugv_waypoint', anonymous=True)
    
    uav_ids = [1, 2]  # UAV IDs
    uav_goal_subs = []
    for uav_id in uav_ids:
        topic = '/uav_goal_status/uav{}'.format(uav_id)
        uav_goal_sub = rospy.Subscriber(topic, String, lambda msg, uav_id=uav_id: ugv_goal_callback(msg, uav_id))
        uav_goal_subs.append(uav_goal_sub)

    ugv_velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Specify the three goal positions with types
    goals = get_ugv_goals()
    
    ugv_position_sub = rospy.Subscriber('/odometry/filtered', Odometry, lambda msg: ugv_position_callback(msg, goals, ugv_velocity_pub, ugv_goal_pub))

    rate = rospy.Rate(0.20)  # 5 Hz

    for goal in goals:
        # Move UGV to the current goal position
        while not rospy.is_shutdown():
            # Publish UGV's current velocity
            ugv_velocity_pub.publish(create_twist(0.8, 0.1))
            rospy.loginfo("Published UGV velocity: Linear=%.2f, Angular=%.2f", 0.8, 0.1)
            
            # Continuously publish goal status
            ugv_goal_pub.publish("in progress")
            
            # Check if the goal type is "rendezvous"
            if goal[3] == "rv":
                # Wait for some time
                while not rospy.get_param('/uav_goal_reached'):
                    rospy.sleep(0.1)

                ugv_goal_pub.publish("rendezvous_reached")
            # Check if the goal type is "regular"
            if goal[3] == "wp":
                break

        rospy.loginfo("UGV reached goal: %s", str(goal))
        # Publish goal status as "reached"
        ugv_goal_pub.publish("reached")
        
        # Add a delay between iterations to control the publishing frequency
        rospy.sleep(0.1)
    
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

