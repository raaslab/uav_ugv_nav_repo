from __future__ import print_function
import cv2
import numpy as np
import os
import yaml


class ArucoMarkerEstimator:

    def __init__(self, paramfilepath=None, family_name=None,
        marker_side_len=None):
        self.__calib_mat = None
        self.__calib_dst = None
        self.__aruco_dict = None
        self.__aruco_params = None
        self.__marker_side_len = None

        if paramfilepath is not None:
            self.load_camera_params(paramfilepath)

        if family_name is not None:
            self.set_marker_search_family(family_name)

        if marker_side_len is not None:
            self.set_marker_side_len(marker_side_len)
    #end def

    def detect(self, image):
        """
        Estimates the IDs and four corners of each detected aruco marker in an 
        image. If no marker was not found, this function returns an empty list.
        """
        # Sanity check.
        if image is None:
            return None
        
        # Estimate the corners and IDs of the markers. Ignore rejected markers.
        corners, marker_ids, _ = cv2.aruco.detectMarkers(image=image,
            dictionary=self.__aruco_dict, parameters=self.__aruco_params)

        if len(corners) == 0:
            return None

        dtypes = [('mid',int), ('corners',float,(4,2))]
        detections = np.empty((len(corners,)), dtype=dtypes)
        detections['mid'] = marker_ids
        detections['corners'] = np.array(corners).reshape(-1,4,2)
        return detections        
    #end def

    def estimate_pose(self, detections):
        # Sanity checks.
        if detections is None:
            return None
        if len(detections) == 0:
            return None
        if self.has_calib_params() == False:
            return None

        # Estimate the pose of all the seen markers.
        rvecs, tvecs, objpts = cv2.aruco.estimatePoseSingleMarkers(
            corners=detections['corners'],
            markerLength=self.__marker_side_len,
            cameraMatrix=self.__calib_mat, distCoeffs=self.__calib_dst)

        # Sanity check.
        if len(detections) != len(rvecs):
            raise Exception('Failed to estimate pose of all detected markers.')

        # Create an array of estimates.
        dtypes =  [('mid',int), ('tvec',float,(1,3)), ('rvec',float,(1,3))]
        estimates = np.empty((len(detections)), dtype=dtypes)
        estimates['mid'] = detections['mid'].copy()
        estimates['tvec'] = np.array(tvecs).reshape(-1,1,3)
        estimates['rvec'] = np.array(rvecs).reshape(-1,1,3)
        
        return estimates
    #end def

    @staticmethod
    def get_family_dictionaries():
        """
        Returns a dictionary containing the Aruco families.
        """
        return dict({
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        })
    #end def

    def has_calib_params(self):
        return self.__calib_dst is not None and self.__calib_mat is not None

    def load_camera_params(self, paramfilepath):
        """
        Loads the camera calibration parameters from the specifed 
        """
        if not os.path.exists(paramfilepath):
            raise Exception("The calibration parameters file path " \
                + "({}) does not exist.".format(paramfilepath))
        #end if

        # Load the camera parameters from the saved file
        with open(paramfilepath, "r") as f:
            yamlfile = yaml.safe_load(f)
            self.__calib_mat = np.array(yamlfile['K']['data']).reshape((3,3))
            self.__calib_dst = np.array(yamlfile['D']['data']).reshape((1,5))
    #end def

    def set_marker_side_len(self, marker_side_len):
        """
        Sets the side length of the markers to observe.
        """
        if isinstance(marker_side_len, float) and marker_side_len > 0.:
            self.__marker_side_len = marker_side_len
        else:
            raise Exception("Marker side length must be a positive float.")
    #end def

    def set_marker_search_family(self, family_name):
        """
        Sets the marker family to search for in images.
        """
        family_dict = ArucoMarkerEstimator.get_family_dictionaries()
        if family_dict.get(family_name, None) is None:
            raise Exception("ArUco family '{}' is not supported!".format(
                family_name))
        #end if

        self.__aruco_dict = cv2.aruco.Dictionary_get(family_dict[family_name])
        self.__aruco_params = cv2.aruco.DetectorParameters_create()
    #end def
#end classsudo apt-get install python-rospkg