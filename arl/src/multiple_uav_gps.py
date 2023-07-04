import rospy
import re
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from math import sqrt
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, SetModeRequest
from uav_goals import get_uav_goals
from mavros_msgs.msg import State 
from sensor_msgs.msg import NavSatFix
# Global Variables
rospy.set_param('/ugv_goal_reached', False)
ugv_goal_status = False
uav_goal_publisher = None
ugv_goal_subscriber =None
current_state = None
current_pose = None
use_gps = False  # Set this flag to True if using GPS

def uav_goal_callback(msg, uav_id):
    # Callback function to receive UGV's goal status
    global ugv_goal_status
    goal_status = msg.data
    print("Received goal status:", goal_status)
    if goal_status == "rendezvous_reached":
        rospy.set_param('/ugv_goal_reached', True)
        print("UAV", uav_id, "received 'rendezvous_reached' status from UGV")
        ugv_goal_status = True  # Set a global variable to track the UGV's goal status
    elif goal_status == "reached" or goal_status == "in progress":
        rospy.set_param('/ugv_goal_reached', False)
        ugv_goal_status = False

def publish_uav_goal_position(uav_id, position):
    if use_gps:
        topic = '/uav' + str(uav_id) + '/mavros/global_position/global'
    else:
        topic = '/uav' + str(uav_id) + '/mavros/setpoint_position/local'
    uav_position_publisher = rospy.Publisher(topic, PoseStamped, queue_size=10)
    uav_position_publisher.publish(create_pose(position.x, position.y, position.z))

def publish_uav_goal_status(uav_id, status):
    topic = '/uav_goal_status/uav' + str(uav_id)
    uav_goal_publisher = rospy.Publisher(topic, String, queue_size=10)
    uav_goal_publisher.publish(status)


def arming(uav_id, arm):
    arming_service = rospy.ServiceProxy("/uav" + str(uav_id) + "/mavros/cmd/arming", CommandBool)
    try:
        arming_service(arm)
        rospy.loginfo("UAV %d %s", uav_id, "armed" if arm else "disarmed")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to %s UAV %d: %s", "arm" if arm else "disarm", uav_id, str(e))


def set_mode(uav_id, mode):
    set_mode_service = rospy.ServiceProxy("/uav" + str(uav_id) + "/mavros/set_mode", SetMode)
    try:
        offboard_mode = SetModeRequest()
        offboard_mode.custom_mode = mode
        set_mode_service(0, offboard_mode.custom_mode)
        rospy.loginfo("UAV %d set to %s mode", uav_id, mode)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to set %s mode for UAV %d: %s", mode, uav_id, str(e))

def create_pose(x, y, z):
    """Create a PoseStamped message with the given coordinates"""
    pose = PoseStamped()
    if use_gps:
        pose.pose.position.latitude = x
        pose.pose.position.longitude = y
        pose.pose.position.altitude = z
    else:
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
    return pose

def initialize_uav(uav_id):
    """Initialize the UAV node and subscribe to UGV goal status"""
    rospy.init_node('uav_waypoint', anonymous=True)
    rospy.Subscriber('/uav' + str(uav_id) + '/mavros/state', State, state_cb)
    rospy.Subscriber('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped, pose_cb)

def state_cb(msg):
    # Callback function to update the current state
    global current_state
    current_state = msg

def pose_cb(msg):
# Callback function to update the current pose
    global current_pose
    current_pose = msg 

def waypoint_navigation(uav_id):
    """Perform waypoint navigation for the UAV"""
    initialize_uav(uav_id)
    rate = rospy.Rate(20)  # 10 Hz
    
    goals = get_uav_goals(uav_id)
    takeoff(uav_id, 2)
    
    #land(uav_id, 0)
    for goal in goals:
        goal_position = goal["position"]
        goal_type = goal["type"]
        ugv_goal_status = False
        rospy.set_param('/ugv_goal_reached', False)
        rospy.loginfo("Published UAV position: " + str(goal_position))
        while not rospy.is_shutdown():
            publish_uav_goal_position(uav_id, goal_position)
            if use_gps:
                uav_position = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/global_position/global', NavSatFix).position
            else:
                uav_position = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped).pose.position
            
            distance_to_goal = sqrt((uav_position.x - goal_position.x) ** 2 + (uav_position.y - goal_position.y) ** 2)
            
            if distance_to_goal < 0.4:
                if goal_type == "rv":
                    handle_rendezvous_goal(uav_id, goal_position)
                    #land(uav_id, 0)
                    #handle_hover(uav_id)
                else:
                    publish_uav_goal_status(uav_id, "reached")
                    #handle_hover(uav_id)
                    rospy.loginfo("UAV " + str(uav_id) + " reached goal: " + str(goal_position))
                break

            rate.sleep()
            
def takeoff(uav_id,ALT):
    # Start the main loop 
    rate = rospy.Rate(20) # 20 Hz update rate
    pose_msg=PoseStamped()
    rospy.loginfo("UAV %d hover status", uav_id)
        # Take off first
    take_off_position = None  # Initialize take_off_position variable
    while take_off_position is None:
        try:
            current_pose = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped)
            take_off_position = current_pose.pose.position
            take_off_position.z = ALT  # Update the desired takeoff altitude
        except rospy.ROSInterruptException:
            pass

    for i in range(200):
        print(take_off_position)
        #if current_state is not None and current_state.mode == 'OFFBOARD':
            #print("hover initiate")
        rospy.loginfo("Published UAV position: " + str(take_off_position))
        publish_uav_position(uav_id,take_off_position)

        # if sqrt((take_off_position.x - current_pose.pose.position.x) ** 2 + 
        #         (take_off_position.y - current_pose.pose.position.y) ** 2+
        #         (take_off_position.z - current_pose.pose.position.z) ** 2)< 0.2:
        print("reached_take_off)")
        rate.sleep()

def land(uav_id, ALT):
    rospy.loginfo("UAV %d landing status", uav_id)
    
    landing_position = None  # Initialize landing position variable
    
    while landing_position is None:
        try:
            current_pose = rospy.wait_for_message('/uav' + str(uav_id) + '/mavros/local_position/pose', PoseStamped)
            landing_position = current_pose.pose.position
            landing_position.z = ALT  # Update the desired landing altitude
        except rospy.ROSInterruptException:
            pass
    
    rate = rospy.Rate(20)  # 20 Hz update rate
    
    for i in range(100) :

        publish_uav_goal_position(uav_id, landing_position)
        
        # # Check distance between current pose and landing position
        distance_to_landing = sqrt((landing_position.x - current_pose.pose.position.x) ** 2 +
                                   (landing_position.y - current_pose.pose.position.y) ** 2 +
                                   (landing_position.z - current_pose.pose.position.z) ** 2)
        
        # # Exit the loop if the UAV has reached the landing position
        if distance_to_landing < 0.1:
            arming(uav_id, False)
        
        rate.sleep()

def handle_hover(uav_id):
    # Start the main loop
 
    rate = rospy.Rate(20) # 20 Hz update rate
    rospy.loginfo("UAV %d hover status", uav_id)
    while not rospy.is_shutdown():
        #print(current_state)
        if current_state is not None and current_state.mode == 'OFFBOARD':
            
            if current_pose is not None:
                #print("hover initiate")
                publish_uav_goal_position(uav_id, current_pose.pose.position)
    
        rate.sleep() 

def handle_rendezvous_goal(uav_id, goal_position):
    """Handle the logic for rendezvous goals"""
    publish_uav_goal_status(uav_id, "rendezvous_reached" + str(uav_id))
    rospy.loginfo("UAV %d published 'rendezvous_reached' status", uav_id)

    while not rospy.is_shutdown():
        if rospy.get_param('/ugv_goal_reached'):
            rospy.loginfo("UGV reached the rendezvous point")
            break
        else:
            publish_uav_goal_position(uav_id, goal_position)
            rospy.sleep(0.5)

    while not rospy.is_shutdown() and not rospy.get_param('/ugv_goal_reached'):
        rospy.loginfo("UAV %d hovering at the rendezvous point...", uav_id)
        publish_uav_goal_position(uav_id, goal_position)
        rospy.sleep(0.5)

# Modify other relevant functions to handle GPS data if use_gps flag is set

def main():
    # node_name = rospy.get_name()  # Get the full node name
    # match = re.search(r'\d+', node_name)
    # uav_id = int(match.group())
    uav_id=1
    rospy.Subscriber('/ugv_goal_status', String, lambda msg, uav_id=uav_id: uav_goal_callback(msg, uav_id))
    try:
        waypoint_navigation(uav_id)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    #rospy.sleep(5)
    main()
