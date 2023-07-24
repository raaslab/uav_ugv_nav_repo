#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from threading import Timer

def publish_topic():
    pub = rospy.Publisher('/uav_goal_status/uav1', String, queue_size=10)
    rospy.init_node('fake_uav', anonymous=True)
    
    rate = rospy.Rate(0.1)  # Publish every 10 seconds
    while not rospy.is_shutdown():
        msg = "rendezvous_reached1"
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_topic()
    except rospy.ROSInterruptException:
        pass