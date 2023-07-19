#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from threading import Timer

def publish_topic():
    pub = rospy.Publisher('/ugv_goal_status', String, queue_size=100)
    rospy.init_node('fake_ugv', anonymous=True)
    
    rate = rospy.Rate(0.1)  # Publish every 10 seconds
    while not rospy.is_shutdown():
        msg = "rendezvous_reached"
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_topic()
    except rospy.ROSInterruptException:
        pass