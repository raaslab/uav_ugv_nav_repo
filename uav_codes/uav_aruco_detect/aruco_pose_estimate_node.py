#!/usr/bin/env python3

from aruco_estimator import ArucoMarkerEstimator
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import os
import rospkg
import rospy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image


class ArucoPoseEstimatorNode:

    def __init__(self, paramfilepath, family_name, marker_side_len,
        frame_of_marker="/camera"):
        self.cvbridge = CvBridge()

        # Set up the marker estimator.
        self.marker_estimator = ArucoMarkerEstimator(
            paramfilepath=paramfilepath, family_name=family_name,
            marker_side_len=marker_side_len)
        self.__frame_of_marker = frame_of_marker
    #end def

    def rcvd_image_callback(self, image_msg):
        """
        Callback function used to estimate the marker pose and predicted
        uncertainty.
        """
        # Convert the color image to a CV image, and then get the data.
        cv_image = self.cvbridge.imgmsg_to_cv2(image_msg,
            desired_encoding='bgr8')

        # Perform pose estimation on the detected markers.
        detections = self.marker_estimator.detect(cv_image)
        pose_estimates = self.marker_estimator.estimate_pose(
            detections)
        if pose_estimates is None or len(pose_estimates) == 0:
            return

        # Extract the poses, transform them from camera to global coords,
        # and publish the messages.
        rospy.loginfo('Time @ ' + str(image_msg.header.stamp))
        for i in range(len(pose_estimates)):
            msg = PoseStamped()
            msg.header.stamp = image_msg.header.stamp
            msg.header.frame_id = self.__frame_of_marker

            tx, ty, tz = pose_estimates[i]['tvec'].flatten()
            msg.pose.position.x = tx
            msg.pose.position.y = ty
            msg.pose.position.z = tz

            rotations = R.from_rotvec(
                pose_estimates[i]['rvec'].flatten())
            qx, qy, qz, qw = rotations.as_quat()
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw

            self.__pose_pub.publish(msg)

            logstr = '\n- Marker #{}:'
            logstr += '\n --- pos [xyz in m]:    ({:.2f}, {:.2f}, {:.2f}), '
            logstr += '\n --- rot [rpy in degs]: ({:.1f}, {:.1f}, {:.1f})\n'
            roll, pitch, yaw = rotations.as_rotvec()
            rospy.loginfo(logstr.format(pose_estimates[i]['mid'],
                tx, ty, tz, np.degrees(roll), np.degrees(pitch),
                np.degrees(yaw)))
        #end for
    #end def

    def setup_node(self, camera_image_sub_topic, pose_publisher_topic,
        queue_len):
        self.camera_image_sub = rospy.Subscriber(
            camera_image_sub_topic, Image, callback=self.rcvd_image_callback)

        self.__pose_pub = rospy.Publisher(pose_publisher_topic,
                PoseStamped, queue_size=queue_len)
    #end def
#end class


if __name__ == '__main__':
    queue_len = 60
    rospy.init_node('aruco_pose_estimator', anonymous=True)
    
    # Configure the node.
    rospack = rospkg.RosPack()
    PX4_FPV_CAM_PATH = os.path.join(rospack.get_path('sense_aruco'),
        "calib/gazebo_cams/px4_fpv_cam.yaml")
    paramfilepath = rospy.get_param("~parampath", PX4_FPV_CAM_PATH)
    family_name = rospy.get_param("~familyname", "DICT_4X4_1000")
    marker_side_len = float(rospy.get_param("~marker_len", 0.18))
    image_sub_topic = rospy.get_param("~image_topic", '/iris/usb_cam/image_raw')
    publisher_topic = rospy.get_param("~pose_topic", "aruco_marker")

    aruco_estimator_node = ArucoPoseEstimatorNode(paramfilepath=paramfilepath,
        family_name=family_name, marker_side_len=marker_side_len)
    
    aruco_estimator_node.setup_node(image_sub_topic, publisher_topic,
        queue_len)
    
    rospy.spin()
#end if