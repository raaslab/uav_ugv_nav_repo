#!/usr/bin/env python 

# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc
reload(gc)
# Import AlvinXY transformation module
import alvinxy.alvinxy as axy
reload(axy)
import rospy
import tf
from nav_msgs.msg import Odometry


def get_xy_based_on_lat_long(lat,lon, name):
    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin
    olat = 49.9
    olon = 8.9
    
    xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
    utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    xa,ya = axy.ll2xy(lat,lon,olat,olon)

    rospy.loginfo("#########  "+name+"  ###########")  
    rospy.loginfo("LAT COORDINATES ==>"+str(lat)+","+str(lon))  
    rospy.loginfo("COORDINATES XYZ ==>"+str(xg2)+","+str(yg2))
    rospy.loginfo("COORDINATES AXY==>"+str(xa)+","+str(ya))
    rospy.loginfo("COORDINATES UTM==>"+str(utmx)+","+str(utmy))

    return xg2, yg2

if __name__ == '__main__':
    rospy.init_node('gps_to_xyz_node')
    xg2, yg2 = get_xy_based_on_lat_long(lat=49.9,lon=8.9, name="MAP")
    xg2, yg2 = get_xy_based_on_lat_long(lat=50.9,lon=8.9, name="MAP")
    