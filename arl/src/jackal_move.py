import math
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ugv_goals import get_ugv_goals

ORIGIN_LATITUDE = 49.899915221
ORIGIN_LONGITUDE = 8.90001002309
EARTH_RADIUS = 6378137  # in meters

current_pos = (0, 0)


def gps_to_map_frame(lat, lon):
    """Convert GPS coordinates to map frame."""
    x = EARTH_RADIUS * (math.radians(lon) - math.radians(ORIGIN_LONGITUDE)) * math.cos(math.radians(ORIGIN_LATITUDE))
    y = EARTH_RADIUS * (math.radians(lat) - math.radians(ORIGIN_LATITUDE))
    return x, y


def create_goal_message(x, y):
    """Create a PoseStamped message for the goal."""
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = 'odom'
    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.w = 1.0  # No orientation required for a 2D goal
    return goal_msg


def publish_goal(goal_pub, rate, x, y):
    """Publish the goal."""
    while not rospy.is_shutdown():
        distance = math.sqrt((current_pos[0] - x) ** 2 + (current_pos[1] - y) ** 2)
        if distance < 0.25:  # Goal has been reached
            break
        goal_msg = create_goal_message(x, y)
        goal_pub.publish(goal_msg)
        print(distance)
        rate.sleep()


def pos_callback(data):
    """Update the current position."""
    global current_pos
    current_pos = (data.pose.pose.position.x, data.pose.pose.position.y)


def follow_waypoints(goal_pub, rate, waypoints):
    """Follow the waypoints."""
    for waypoint in waypoints:
        x_goal, y_goal = gps_to_map_frame(*waypoint)
        print(x_goal, y_goal)
        publish_goal(goal_pub, rate, x_goal, y_goal)


if __name__ == '__main__':
    try:
        rospy.init_node('goal_publisher', anonymous=True)
        rospy.Subscriber("/odometry/filtered", Odometry, pos_callback)
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rate = rospy.Rate(1)  # Set the rate at which to publish the goal (1 Hz in this example)
        
        waypoints = get_ugv_goals()
        while not rospy.is_shutdown():
            follow_waypoints(goal_pub, rate, waypoints)
            rospy.signal_shutdown('All waypoints have been visited')
    except rospy.ROSInterruptException:
        pass
