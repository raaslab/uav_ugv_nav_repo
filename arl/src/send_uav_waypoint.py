#!/usr/bin/env python


# import rospy
# from geometry_msgs.msg import PoseStamped
# import numpy as np

# def apply_transform(goal_position, drone_position):
#     relative_position = goal_position - drone_position
#     transformed_position = relative_position + np.array([0, -1, 0])  # Adjust the transformation based on the drone's initial position
#     return transformed_position

# def send_uav_waypoint():
#     rospy.init_node('uav_waypoint_publisher', anonymous=True)
#     waypoint_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
#     rate = rospy.Rate(10)  # Publish at 10 Hz

#     drone_position = np.array([0, 0, 0])  # Current drone position
#     goal_position = np.array([5, 5, 2])  # Desired goal position

#     while not rospy.is_shutdown():
#         transformed_goal_position = apply_transform(goal_position, drone_position)

#         # Create a PoseStamped message for the waypoint
#         waypoint_msg = PoseStamped()
#         waypoint_msg.pose.position.x = transformed_goal_position[0]  # Set the transformed x-coordinate
#         waypoint_msg.pose.position.y = transformed_goal_position[1]  # Set the transformed y-coordinate
#         waypoint_msg.pose.position.z = transformed_goal_position[2]  # Set the transformed z-coordinate

#         waypoint_msg.header.stamp = rospy.Time.now()
#         waypoint_publisher.publish(waypoint_msg)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         send_uav_waypoint()
#     except rospy.ROSInterruptException:
#         pass

# import rospy
# from std_msgs.msg import Bool
# from geometry_msgs.msg import PoseStamped, Point
# import numpy as np

# def apply_transform(goal_position, drone_position):
#     goal_x = goal_position.x
#     goal_y = goal_position.y
#     goal_z = goal_position.z
#     drone_x = drone_position[0]
#     drone_y = drone_position[1]
#     drone_z = drone_position[2]
    
#     relative_position = np.array([goal_x - drone_x, goal_y - drone_y, goal_z - drone_z])
#     transformed_position = relative_position + np.array([0, -1, 0])  # Adjust the transformation based on the drone's initial position
    
#     transformed_goal_position = Point()
#     transformed_goal_position.x = transformed_position[0]
#     transformed_goal_position.y = transformed_position[1]
#     transformed_goal_position.z = transformed_position[2]
    
#     return transformed_goal_position



# def send_uav_waypoint():
#     rospy.init_node('uav_waypoint_publisher', anonymous=True)
#     waypoint_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
#     goal_reached_pub = rospy.Publisher("/uav_goal_reached", Bool, queue_size=1)
#     rate = rospy.Rate(10)  # Publish at 10 Hz

#     drone_position = np.array([0, 1, 0])  # Current drone position

#     # List of goal positions
#     goal_positions = [
#         apply_transform(Point(5, 5, 2), drone_position),  # Apply transformation for UAV goal position
#         apply_transform(Point(3, 2, 1), drone_position),
#         apply_transform(Point(1, 4, 3), drone_position)
#     ]

#     for goal_position in goal_positions:
#         waypoint_msg = PoseStamped()
#         waypoint_msg.header.stamp = rospy.Time.now()
#         waypoint_msg.header.frame_id = 'map'  # Set the frame ID according to your setup
#         waypoint_msg.pose.position = goal_position

#         while not rospy.is_shutdown():
#             waypoint_publisher.publish(waypoint_msg)
#             if reached_goal(goal_position, drone_position):
#                 goal_reached_pub.publish(Bool(True))
#                 break
#             rate.sleep()

# def reached_goal(goal_position, drone_position):
#     # Add your logic to check if the drone has reached the goal position
#     return False

# if __name__ == '__main__':
#     try:
#         send_uav_waypoint()
#     except rospy.ROSInterruptException:
#         pass

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point, Twist
import numpy as np
goal_reached = False
def apply_transform(goal_position, drone_position):
    goal_x = goal_position.x
    goal_y = goal_position.y
    goal_z = goal_position.z
    drone_x = drone_position[0]
    drone_y = drone_position[1]
    drone_z = drone_position[2]

    relative_position = np.array([goal_x - drone_x, goal_y - drone_y, goal_z - drone_z])
    transformed_position = relative_position + np.array([0, -2, 0])  # Adjust the transformation based on the drone's initial position

    transformed_goal_position = Point()
    transformed_goal_position.x = transformed_position[0]
    transformed_goal_position.y = transformed_position[1]
    transformed_goal_position.z = transformed_position[2]

    return transformed_goal_position

def goal_reached_callback(msg):
    global goal_reached
    if msg.data:
        goal_reached = True
        
        pass
# def ugv_cmd_vel_callback(msg):
#     global ugv_started_moving
#     if msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.linear.z != 0.0 or msg.angular.x != 0.0 or msg.angular.y != 0.0 or msg.angular.z != 0.0:
#         ugv_started_moving = True

def send_uav_waypoint():
    rospy.init_node('uav_waypoint_publisher', anonymous=True)
    waypoint_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    #cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, ugv_cmd_vel_callback)  # Subscribe to UGV cmd_vel topic
    goal_reached_publisher = rospy.Publisher('/uav_goal_reached', Bool, queue_size=1)  # Publisher for UAV goal reached status
    rospy.Subscriber('/ugv_goal_reached', Bool, goal_reached_callback)  # Subscribe to UGV goal reached topic
    rate = rospy.Rate(20)  # Publish at 10 Hz
    global goal_reached
    origin = np.array([0, 0, 0])  # Current drone position
    drone_position= np.array([0,0,0])
    ugv_started_moving = False

    #     # Wait for UGV to start moving
    # rospy.loginfo("Waiting for UGV to start moving...")
    # while not rospy.is_shutdown():
    #     if ugv_started_moving:
    #         break
    #     rate.sleep()

    # rospy.loginfo("UGV and UAV started together!")
    # List of goal positions
    goal_positions = [
        #SSapply_transform(Point(0, 0, 2), origin),
        apply_transform(Point(10, 10, 2), origin),  # Apply transformation for UAV goal position
        apply_transform(Point(3, 2, 2), origin),
        apply_transform(Point(10, 4, 2), origin)
    ]

    for goal_position in goal_positions:
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = 'map'  # Set the frame ID according to your setup
        waypoint_msg.pose.position = goal_position

        while not rospy.is_shutdown():
            waypoint_publisher.publish(waypoint_msg)
            if goal_reached_condition(goal_position, drone_position):
                goal_reached_publisher.publish(Bool(True))  # Publish UAV goal reached status
                break
            rate.sleep()

            # Get the current drone position from the topic 'mavros/local_position/pose'
            try:
                current_pose = rospy.wait_for_message('mavros/local_position/pose', PoseStamped, timeout=1)
                drone_position[0] = current_pose.pose.position.x
                drone_position[1] = current_pose.pose.position.y
                #drone_position[2] = current_pose.pose.position.z
            except rospy.exceptions.ROSException:
                rospy.logwarn('Timeout occurred while waiting for current drone position')

        # Wait for the UGV to reach the current goal position
        rospy.loginfo("Waiting for UGV to reach the goal position...")
        while not rospy.is_shutdown():
            if goal_reached:  # Subscribe to goal_reached status from UGV
                break
            rate.sleep()

        goal_reached = False  # Reset the UGV goal reached status

    rospy.loginfo("All goals reached")

def goal_reached_condition(goal_position, drone_position):
    # Compare the drone's current position with the goal position
    return (
        abs(drone_position[0] - goal_position.x) < 0.1 and
        abs(drone_position[1] - goal_position.y) < 2.1
    )

if __name__ == '__main__':
    try:
        send_uav_waypoint()
    except rospy.ROSInterruptException:
        pass

