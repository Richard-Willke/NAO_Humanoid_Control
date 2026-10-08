#!/usr/bin/env python2
import sys
sys.path.append('/workspaces/hrs/hrs_ws/src/config_libs/src/config_wrap')
import config_wrapper
import cv2 
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import numpy as np
from config_wrap.config_wrapper import *
from utils.utils import *
from collections import deque
import imutils
import tf
from visualization_msgs.msg import Marker, MarkerArray
from naoqi import ALBroker
from std_msgs.msg import Float64MultiArray


class NaoCamera:

    """ A class that saves the Goal and Objects in Torso frame of the Nao Robot """

    def __init__(self):


        self.br_top = tf.TransformBroadcaster() 
        self.br_bottom = tf.TransformBroadcaster() 
        self.ls_top = tf.TransformListener()
        self.ls_bottom = tf.TransformListener()

        self.image_publisher = rospy.Publisher("/nao_custom_image", Image, queue_size=10)

        self.obstacle_publisher = rospy.Publisher("/nao_torso_obstacle", Float64MultiArray, queue_size=10)
        self.goal_publisher = rospy.Publisher("/nao_torso_goal", Float64MultiArray, queue_size=10)



        self.saved_obstacles = []
        self.obstacle_msg = Float64MultiArray()
        self.goal_msg = Float64MultiArray()
        self.bridge = cv_bridge.CvBridge()
        self.world_frame_id    = None
        self.torso_position    = None
        self.torso_orientation = None
        self.nao_params = config_wrapper.NAO_Params()
        self.obstacles = MarkerArray()
        self.obstacle_size = self.nao_params.obstacle_size
        self.obstacle_color = self.nao_params.obstacle_color

        self.bottom_image = self.nao_params.bottom_image_dim
        self.top_image = self.nao_params.top_image_dim


        """ Variables Required for Rohan's Implementation """
        self.goal_pose = []
        self.goal_detected = False
        # ostacle_dict = ((pos),(x_y_z_w_quaternion))
        self.obstacle_dict = {}
        self.obstacle_new_detected = False
        self.new_marker_id = None
        self.rvec = np.zeros(3)
        self.tvec = np.zeros(3)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL) 
        self.parameters = cv2.aruco.DetectorParameters_create() 
        self.bottom_img = None
        self.top_img = None


    def image_callback_bottom(self, msg):

        """ A callback for the bottom camera subscriber of Main File """

        self.img_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.bottom_img = self.img_msg.copy()
        

    def bottom_image_viewer(self):

        """ Aruco Detection for bottom camera """
        
        if self.bottom_img is not None:

            gray = cv2.cvtColor(self.bottom_img, cv2.COLOR_BGR2GRAY)
            cor, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters= self.parameters)

            if ids is not None:

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(cor, 0.05, self.nao_params.camera_matrix, self.nao_params.dist_coeffs)
                rvecs = np.squeeze(rvecs).reshape(len(ids), -1)
                tvecs = np.squeeze(tvecs).reshape(len(ids), -1)


                for i, marker_id in enumerate(ids):

                    cv2.aruco.drawAxis(self.bottom_img, self.nao_params.camera_matrix, self.nao_params.dist_coeffs, rvecs[i], tvecs[i], 0.05)

                    """ Only publish arucos which are allowed by config_wrapper file  """

                    if marker_id[0].item() not in self.saved_obstacles and marker_id[0].item() in self.nao_params.valid_ids:

                        self.saved_obstacles.append(marker_id[0].item())
                    
                        t = rospy.Time.now()

                        self.br_bottom.sendTransform(tvecs[i], tf.transformations.quaternion_from_euler(rvecs[i][0], rvecs[i][1], rvecs[i][2]), t,
                                        "marker_" + str(marker_id), "CameraBottom_optical_frame")

                        id = marker_id[0].item()

                        self.ls_bottom.waitForTransform("torso", "marker_" + str(marker_id) , rospy.Time(), rospy.Duration(4.0))
                        (trans_t,rot_t) = self.ls_bottom.lookupTransform("torso", "marker_" + str(marker_id), rospy.Time())

                        self.obstacle_dict["marker_" + str(marker_id)] = [trans_t, rot_t]                     

                        self.obstacle_msg.data = [float(marker_id), trans_t[0], trans_t[1], trans_t[2], rot_t[0], rot_t[1],rot_t[2], rot_t[3]]
                        self.obstacle_publisher.publish(self.obstacle_msg) 
                
            
            cv2.imshow('Bottom Image', self.bottom_img)



    def image_callback_top(self, msg):

        """ A callback for rop camera """

        self.img_msg_goal = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.top_img = self.img_msg_goal.copy()


            
    def top_image_viewer(self):
         
        """ Aruco Detection for top camera """
        if self.top_img is not None:

            gray = cv2.cvtColor(self.top_img, cv2.COLOR_BGR2GRAY)
            cor, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters= self.parameters)

            if ids is not None:

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(cor, 0.05, self.nao_params.camera_matrix, self.nao_params.dist_coeffs)
                rvecs = np.squeeze(rvecs).reshape(len(ids), -1)
                tvecs = np.squeeze(tvecs).reshape(len(ids), -1)


                for i, marker_id in enumerate(ids):

                    """ If goal already detected, do not publish it again """ 
                    if not self.goal_detected:

                        if (marker_id[0].item() not in self.saved_obstacles) and (marker_id[0].item() in self.nao_params.goal_id):
                            
                            cv2.aruco.drawAxis(self.top_img, self.nao_params.camera_matrix, self.nao_params.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                            
                            t = rospy.Time.now()
                            self.br_top.sendTransform(tvecs[i], tf.transformations.quaternion_from_euler(rvecs[i][0], rvecs[i][1], rvecs[i][2]), t,
                                                     "marker_Goal",  "CameraTop_optical_frame")

                            self.saved_obstacles.append(marker_id[0].item())

                            self.goal_rvec = rvecs[i]
                            self.goal_tvec = tvecs[i]
                            self.ls_top.waitForTransform("torso", "marker_Goal" , rospy.Time(), rospy.Duration(4.0))
                            (trans_t,rot_t) = self.ls_top.lookupTransform("torso", "marker_Goal", rospy.Time())

                            self.obstacle_dict["marker_" + str(marker_id)] = [trans_t, rot_t]
                            self.goal_detected = True
                            self.rvec = rvecs[i]
                            self.tvec = tvecs[i]

                            self.goal_msg.data = [0, trans_t[0], trans_t[1], trans_t[2], rot_t[0], rot_t[1],rot_t[2], rot_t[3]]
                            self.goal_publisher.publish(self.goal_msg) 


        if self.top_img is not None:

            cv2.aruco.drawAxis(self.top_img, self.nao_params.camera_matrix, self.nao_params.dist_coeffs, self.rvec, self.tvec, 0.05)

            cv2.imshow('Top Image', self.top_img)


    def process_camera(self):

        self.bottom_image_viewer()
        self.top_image_viewer()
        cv2.waitKey(1)




def main():

    robotIP = "10.152.246.156" 
    PORT = 9559
    broker = ALBroker("NaoBroker",
                            "0.0.0.0",   # listen to anyone
                            0,           # find a free port and use it
                            robotIP,          # parent broker IP
                            PORT)        # parent broker port
    camera = NaoCamera()
    rospy.Subscriber("/nao_robot/camera/bottom/camera/image_raw", Image, camera.image_callback_bottom, queue_size = 10)
    rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, camera.image_callback_top, queue_size = 10)


    while not rospy.is_shutdown():
        
        try:  
            camera.process_camera()

        except KeyboardInterrupt: 
            broker.shutdown()
            break

if __name__ == "__main__":

    rospy.init_node("Camera")
    
    main()






