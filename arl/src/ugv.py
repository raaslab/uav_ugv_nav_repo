#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import sqrt
from move_base_msgs.msg import MoveBaseActionFeedback

class UGVWaypointNode:
    def __init__(self):
        rospy.init_node('ugv_waypoint_node')
        
        # Define the goal positions
        self.goal_positions = [(5, 5), (8, 8), (10, 10)]
        self.current_goal_idx = 0
        
        # Create an action client for the UGV
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Subscribe to the UAV's goal status topic
        rospy.Subscriber('/uav_goal_status', String, self.uav_goal_callback)

        # Subscribe to the move_base feedback topic
        rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.move_base_feedback_callback)
 
        # Create a publisher for UGV's goal status
        self.goal_status_pub = rospy.Publisher('/ugv_goal_status', String, queue_size=10)
        
    def uav_goal_callback(self, data):
        # Check if the UAV has reached the current goal position
        if data.data == 'reached':
            # Check if the UGV has also reached the current goal position
            if self.check_ugv_reached_goal():
                # Proceed to the next goal position
                self.current_goal_idx += 1
                
                # Check if all goal positions have been reached
                if self.current_goal_idx >= len(self.goal_positions):
                    rospy.loginfo('UAV and UGV have reached all goal positions.')
                    # Perform any necessary cleanup or shutdown here
                else:
                    # Publish the new goal status for the UGV
                    self.publish_goal_status()
    
    def check_ugv_reached_goal(self):
        # Placeholder logic (Euclidean distance check)
        current_position = self.get_ugv_current_position()  # Implement this method to get the UGV's current position
        goal_position = self.goal_positions[self.current_goal_idx]
        distance_threshold = 0.1  # Adjust this threshold as needed
        
        distance = sqrt((current_position[0] - goal_position[0])**2 +
                    (current_position[1] - goal_position[1])**2)
        
        if distance <= distance_threshold:
            return True
        else:
            return False
    
    def move_base_feedback_callback(self, feedback):
        # UGV's current position from the move_base feedback
        self.current_position = (feedback.feedback.base_position.pose.position.x, feedback.feedback.base_position.pose.position.y)
    
    def get_ugv_current_position(self):
        # Return the UGV's current position obtained from the move_base feedback
        return self.current_position
    
    def publish_goal_status(self):
        # Publish the UGV's current goal status
        goal_status = String()
        goal_status.data = 'Goal {}/{}'.format(self.current_goal_idx + 1, len(self.goal_positions))
        self.goal_status_pub.publish(goal_status)
        
    def send_ugv_goals(self):
        # Start sending goal positions to the UGV
        rate = rospy.Rate(10)  # Adjust the publishing rate as needed
        
        while not rospy.is_shutdown():
            # Check if the current goal position has been reached
            if self.check_ugv_reached_goal():
                # Increment the goal index and check if all goal positions have been reached
                self.current_goal_idx += 1
                if self.current_goal_idx >= len(self.goal_positions):
                    rospy.loginfo('UAV and UGV have reached all goal positions.')
                    # Perform any necessary cleanup or shutdown here
                    break
            
            # Create a new goal message
            goal = MoveBaseGoal()
            
            # Set the goal position
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.goal_positions[self.current_goal_idx][0]
            goal.target_pose.pose.position.y = self.goal_positions[self.current_goal_idx][1]
            goal.target_pose.pose.orientation.w = 1.0
            
            # Send the goal to the UGV
            self.client.send_goal(goal)
            
            # Wait for the goal to be completed or preempted
            self.client.wait_for_result()
            
            rate.sleep()

if __name__ == '__main__':
    ugv_node = UGVWaypointNode()
    ugv_node.send_ugv_goals()
