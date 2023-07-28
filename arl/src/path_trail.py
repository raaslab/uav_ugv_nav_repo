#!/usr/bin/python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from math import sqrt
from ugv_goals import get_ugv_goals

rospy.init_node('custom_marker_trail')
marker_pub = rospy.Publisher('/custom_marker', Marker, queue_size=10)


def distance_between_points(point1, point2):
    # Calculate Euclidean distance between two points
    return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def publish_trail(trail_points):
    marker = Marker()
    marker.header.frame_id = "odom"  # Replace "map" with the frame you want the marker to be attached to
    marker.header.stamp = rospy.Time.now()
    marker.ns = "custom_marker_trail"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.05  # Adjust the width of the trail line
    marker.color.a = 1.0
    marker.color.r = 1.0  # Replace with the desired color (red in this example)
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()  # Marker will persist until manually removed

    for point in trail_points:
        p = Point()
        p.x, p.y, p.z = point
        marker.points.append(p)

    # Publish the marker
    marker_pub.publish(marker)

def odometry_callback(odom_msg):
    global trail_points
    # Get the robot's position from the odometry message
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    z = odom_msg.pose.pose.position.z  # Usually 0 in 2D
    print(x,y,z)
    if not trail_points or distance_between_points(trail_points[-1], (x, y)) > 0.1:
        trail_points.append((x, y, z))

    # Ensure the trail has a limited length (e.g., the last 100 points)

    publish_trail(trail_points)


def read_coordinates_from_file(file_name):
    coordinates = []
    with open(file_name, 'r') as file:
        lines = file.readlines()[1:]  # Skip the header line
        for line in lines:
            line = line.strip()
            if line:
                parts = line.split(',')
                x = float(parts[1])
                y = float(parts[2]) 
            coordinates.append((x, y))
    return coordinates

def publish_fixed_coordinates(coordinates):
    rate = rospy.Rate(20)  # Adjust the rate to control the publishing frequency
    for i, (x, y) in enumerate(coordinates):
        marker = Marker()
        marker.header.frame_id = "odom"  # Replace "map" with the frame you want the marker to be attached to
        marker.header.stamp = rospy.Time.now()
        marker.ns = "custom_marker_wp"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # Replace with the Z position of the marker
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = .2  # Replace with the desired scale of the marker
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.lifetime = rospy.Duration()
        # Publish the marker
        marker_pub.publish(marker)
        if i > 0:
            line_marker = Marker()
            line_marker.header.frame_id = "odom"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "line_markers"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.05  # Adjust the width of the line
            line_marker.color.a = 1.0
            line_marker.color.r = 0.0  # Replace with the desired color (blue in this example)
            line_marker.color.g = 0.0
            line_marker.color.b = 1.0
            line_marker.lifetime = rospy.Duration()  # Marker will persist until manually removed

            # Add the current coordinate as the first point of the line
            p1 = Point()
            p1.x, p1.y = coordinates[i - 1]
            p1.z = 0
            line_marker.points.append(p1)

            # Add the next coordinate as the second point of the line
            p2 = Point()
            p2.x, p2.y, p2.z = x, y, 0.0
            line_marker.points.append(p2)
            #print(line_marker)
            #print("publishing fixed markers")
            marker_pub.publish(line_marker)
        rate.sleep()

if __name__ == '__main__':
    rate = rospy.Rate(1)  # Adjust the rate to control the publishing frequency

    # for the fixed markers
    #file_name = "/home/experiment/catkin_ws/src/arl/src/experiments/ugv_goals_arl_small_area.txt"
    #coordinates = read_coordinates_from_file(file_name)
    coordinates = get_ugv_goals()
    # for the trail of the path traversed
    trail_points = []

    # Subscribe to the "/odom" topic to get odometry data
    rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback)

    while not rospy.is_shutdown():
        publish_fixed_coordinates(coordinates)
        rate.sleep()