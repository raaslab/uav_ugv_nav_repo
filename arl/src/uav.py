#!/usr/bin/env python

import rospy
import actionlib
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from math import sqrt


class UAVWaypointNode:
    def __init__(self):
        rospy.init_node('uav_waypoint_node')

        # Define the goal positions
        self.goal_positions = [(5, 3, 3), (8, 6, 3), (10, 8, 3)]
        self.current_goal_idx = 0

        # Create a publisher for the UAV's current position
        self.position_publisher = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Create an action client for the UAV
        self.client = actionlib.SimpleActionClient('mavros/setpoint_raw/local', PositionTarget)

        # Subscribe to the UGV's goal status topic
        rospy.Subscriber('/ugv_goal_status', String, self.ugv_goal_callback)

        # Create a publisher for UAV's goal status
        self.goal_status_pub = rospy.Publisher('/uav_goal_status', String, queue_size=10)

        # Connect to the MAVROS services
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        self.arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Arm and set the UAV to OFFBOARD mode
        self.arm_client(True)
        self.set_mode_client(custom_mode='OFFBOARD')

    def ugv_goal_callback(self, data):
        # Check if the UGV has reached the current goal position
        if data.data == 'Goal {}/{}'.format(self.current_goal_idx + 1, len(self.goal_positions)):
            # Check if the UAV has also reached the current goal position
            if self.check_uav_reached_goal():
                # Proceed to the next goal position
                self.current_goal_idx += 1

                # Check if all goal positions have been reached
                if self.current_goal_idx >= len(self.goal_positions):
                    rospy.loginfo('UAV and UGV have reached all goal positions.')
                    # Perform any necessary cleanup or shutdown here
                else:
                    # Publish the new goal status for the UAV
                    self.publish_goal_status()

    def check_uav_reached_goal(self):
        # Placeholder logic (Euclidean distance check)
        current_position = self.get_uav_current_position()  # Implement this method to get the UAV's current position
        goal_position = self.goal_positions[self.current_goal_idx]
        distance_threshold = 0.5  # Adjust this threshold as needed

        distance = sqrt((current_position[0] - goal_position[0]) ** 2 +
                        (current_position[1] - goal_position[1]) ** 2 +
                        (current_position[2] - goal_position[2]) ** 2)

        if distance <= distance_threshold:
            return True
        else:
            return False

    def get_uav_current_position(self):
        # UAV's current position from mavros
        # Return the current position as a tuple (x, y, z)
        current_pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        z = current_pose.pose.position.z

        return x, y, z

    def publish_goal_status(self):
        # Publish the UAV's current goal status
        goal_status = String()
        goal_status.data = 'Goal {}/{}'.format(self.current_goal_idx + 1, len(self.goal_positions))
        self.goal_status_pub.publish(goal_status)

    def send_uav_goals(self):
        # Start sending goal positions to the UAV
        rate = rospy.Rate(10)  # Adjust the publishing rate as needed

        while not rospy.is_shutdown():
            # Check if the current goal position has been reached
            if self.check_uav_reached_goal():
                # Increment the goal index and check if all goal positions have been reached
                self.current_goal_idx += 1
                if self.current_goal_idx >= len(self.goal_positions):
                    rospy.loginfo('UAV and UGV have reached all goal positions.')
                    # Perform any necessary cleanup or shutdown here
                    break

            # Create a new goal message
            goal = PositionTarget()
            goal.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            goal.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                             PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                             PositionTarget.IGNORE_YAW_RATE

            # Set the goal position
            goal.position.x = self.goal_positions[self.current_goal_idx][0]
            goal.position.y = self.goal_positions[self.current_goal_idx][1]
            goal.position.z = self.goal_positions[self.current_goal_idx][2]

            # Send the goal to the UAV
            self.client.send_goal(goal)

            # Wait for the goal to be completed or preempted
            self.client.wait_for_result()

            rate.sleep()


if __name__ == '__main__':
    uav_node = UAVWaypointNode()
    uav_node.send_uav_goals()
