
#!/usr/bin/python

import subprocess

# Start a new ROS bag recording
rosbag_process = subprocess.Popen(['rosbag', 'record', '-a'])

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback,MoveBaseActionFeedback

import math
from ugv_goals import get_ugv_goals

rendezvous_complete = False
rospy.init_node('goal_publisher', anonymous=True)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
uav_status_pub = rospy.Publisher('/ugv_goal_status', String, queue_size=1)

def ugv_goal_callback(msg, uav_id):
    global rendezvous_complete
    goal_status = msg.data
    if goal_status.startswith("rendezvous_reached") and goal_status.endswith(str(uav_id)):
        rendezvous_complete = True
        #rospy.set_param('/uav_goal_reached', True)

def publish_goal(x, y, goal_type, uav_id):
    global rendezvous_complete
    rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, pos_callback)
    
    rate = rospy.Rate(1)  # Set the rate at which to publish the goal (1 Hz in this example)
    
    while not rospy.is_shutdown():
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # No orientation required for a 2D goal
        goal_pub.publish(goal_msg)
        
        distance_to_goal = math.sqrt((current_pos[0]-x)**2 + (current_pos[1]-y)**2)
        print(distance_to_goal)
        if distance_to_goal < 0.25:
            if goal_type == 'rv':
                uav_status_pub.publish("rendezvous_reached uav{}".format(uav_id))
                while not rendezvous_complete:  # Wait until the rendezvous is completed
                    rate.sleep()
                rendezvous_complete = False
            return  # Return if we are within the tolerance of the goal
        rate.sleep()

def pos_callback(data):
    global current_pos
    current_pos = (data.feedback.base_position.pose.position.x, data.feedback.base_position.pose.position.y)


def follow_waypoints(waypoints):
    uav_ids = [1, 2]
    for uav_id in uav_ids:
        topic = '/uav_goal_status/uav{}'.format(uav_id)
        rospy.Subscriber(topic, String, lambda msg, uav_id=uav_id: ugv_goal_callback(msg, uav_id))

    for waypoint in waypoints:
        x_goal = waypoint[0]
        y_goal = waypoint[1]
        goal_type = waypoint[2]
        uav_id = waypoint[3]

        print(x_goal, y_goal, goal_type, uav_id)
        publish_goal(x_goal, y_goal, goal_type, uav_id)  # Use the coordinates of the current waypoint
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        current_pos=(0,0)
        waypoints = get_ugv_goals()
        follow_waypoints(waypoints)
        rosbag_process.terminate()
    except rospy.ROSInterruptException:
        pass
