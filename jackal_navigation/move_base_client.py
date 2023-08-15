#! /usr/bin/env python

import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

"""
class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

"""
# We create some constants with the corresponing vaules from the SimpleGoalState class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

"""
/move_base/goal
### PYTHON MESSAGE

rosmsg show move_base_msgs/MoveBaseGoal                                                          
geometry_msgs/PoseStamped target_pose                                                                                          
  std_msgs/Header header                                                                                                       
    uint32 seq                                                                                                                 
    time stamp                                                                                                                 
    string frame_id                                                                                                            
  geometry_msgs/Pose pose                                                                                                      
    geometry_msgs/Point position                                                                                               
      float64 x                                                                                                                
      float64 y                                                                                                                
      float64 z                                                                                                                
    geometry_msgs/Quaternion orientation                                                                                       
      float64 x                                                                                                                
      float64 y                                                                                                                
      float64 z                                                                                                                
      float64 w                                                                                                                
               


/move_base/cancel                                                                                                                                                         
/move_base/cmd_vel                                                                                                                                                        
/move_base/current_goal                                                                                                                                                   
/move_base/feedback
"""

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    rospy.loginfo(str(feedback))

# initializes the action client node
rospy.init_node('move_base_gps_node')

action_server_name = '/move_base'
client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)

# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

# creates a goal to send to the action server
goal = MoveBaseGoal()

goal.target_pose.header.frame_id = "/map"
goal.target_pose.header.stamp    = rospy.get_rostime()
goal.target_pose.pose.position.x = 0.0
goal.target_pose.pose.orientation.z = 0.0
goal.target_pose.pose.orientation.w = 1.0

client.send_goal(goal, feedback_cb=feedback_callback)


# You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
#Its a variable, better use a function like get_state.
#state = client.simple_state
# state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
state_result = client.get_state()

rate = rospy.Rate(1)

rospy.loginfo("state_result: "+str(state_result))

while state_result < DONE:
    rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    
rospy.loginfo("[Result] State: "+str(state_result))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")

#rospy.loginfo("[Result] State: "+str(client.get_result()))