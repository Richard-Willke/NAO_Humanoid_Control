#!/usr/bin/env python
import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_1.srv import MoveJoints, MoveJointsResponse
import numpy as np
import cv2 
import cv_bridge
from sensor_msgs.msg import JointState, Image

motionProxy =0



class CameraClient:

    def __init__(self):
        self.image_subscriber = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.image_callback_top, queue_size = 10)
        self.image_publisher = rospy.Publisher("/nao_custom_image", Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()
        self.aruco_center = None #np.zeros(2)
        self.response = MoveJointsResponse()
        self.img_msg = np.zeros((320,240))
        self.ids = None
        

    def arucoFunction(self):

        camera_matrix = np.array([551.543059, 0.0, 327.382898, 0.0, 553.736023, 225.02638, 0.0, 0.0, 1]).reshape(3, 3)
        dist_coeffs = np.array([-0.066494, 0.095481, -0.000279, 0.002292, 0.0]).reshape(1, 5)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL) 
        parameters = cv2.aruco.DetectorParameters_create() 
        gray = cv2.cvtColor(self.img_msg, cv2.COLOR_BGR2GRAY)
        self.cor, self.ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if self.ids is not None:
            center_x = 0
            center_y = 0
            for corners in self.cor:
                marker_corners = corners[0]
                center_x += int(np.mean(marker_corners[:,0]))
                center_y += int(np.mean(marker_corners[:,1]))

            self.aruco_center = np.array([center_x/len(self.cor), center_y/len(self.cor)])

        else:

            self.ids = None

        self.img_msg = cv2.aruco.drawDetectedMarkers(self.img_msg, self.cor, self.ids, (0, 255, 0))
        cv2.namedWindow("Aruco Marker", cv2.WINDOW_AUTOSIZE) 
        cv2.circle(self.img_msg, (self.aruco_center[0],self.aruco_center[1]), 5, (255,0,0), 5)
        cv2.imshow('Aruco Marker', self.img_msg)
        cv2.waitKey(1)


    def image_callback_top(self, msg):
        self.img_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.arucoFunction()


    def call_service(self):

        rospy.wait_for_service('/move_joints')
        try:
            if self.aruco_center is not None:
                proxy = rospy.ServiceProxy('/move_joints', MoveJoints)
                # here mode stays at 3, since it is only executed for task 3
                resp = proxy(list(self.aruco_center), True, 3 ) 
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        

if __name__ == '__main__':

    rospy.init_node('client_node')
    client_ = CameraClient()

    while not rospy.is_shutdown():
        try:
            client_.call_service()
        except:
            pass

    #rospy.spin()
    cv2.destroyAllWindows()    
        