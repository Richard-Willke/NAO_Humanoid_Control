#!/usr/bin/env python
import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_2.srv import MoveJoints, MoveJointsResponse
import numpy as np
import cv2 
import cv_bridge
import tf
import math
from sensor_msgs.msg import JointState, Image
motionProxy =0


class CameraClient:

    def __init__(self):
        self.image_subscriber = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.image_callback_top, queue_size = 10)
        self.image_publisher = rospy.Publisher("/nao_custom_image", Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()
        self.aruco_3D_pos = None #np.zeros(2)
        self.run_once = True
        self.aruco_3D_ori = None
        self.response = MoveJointsResponse()
        self.top_img = np.zeros((320,240))
        self.frame_id = None
        self.ids = None
        self.top_img = None
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.trans_marker_torso = None
        self.pi = 3.14159
        self.aruco_pixel = None


    def to_quat(self, roll, pitch, yaw):
        
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return np.array([qw, qx, qy, qz])


    def quaternion_to_euler(self, q = None):
        (x, y, z, w) = (q[0], q[1], q[2], q[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]

    def arucoFunction(self):
        # task 10 arucoFunction
        camera_matrix = np.array([551.543059, 0.0, 327.382898, 0.0, 553.736023, 225.02638, 0.0, 0.0, 1]).reshape(3, 3)
        dist_coeffs = np.array([-0.066494, 0.095481, -0.000279, 0.002292, 0.0]).reshape(1, 5)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL) 
        parameters = cv2.aruco.DetectorParameters_create() 
        gray = cv2.cvtColor(self.top_img, cv2.COLOR_BGR2GRAY)
        cor, self.ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        
        if self.ids is not None:
            self.aruco_pixel = cor[0]
            rvecs, tvecs, trash= cv2.aruco.estimatePoseSingleMarkers(cor, 0.05, camera_matrix, dist_coeffs)
            for i, marker_id in enumerate(self.ids):
                cv2.aruco.drawAxis(self.top_img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
                self.aruco_3D_pos = tvecs[i][0]
                self.aruco_3D_ori = rvecs[i][0]


                

                Rx = np.array([[1, 0, 0], 
                               [0, math.cos(self.aruco_3D_ori [2]), -math.sin(self.aruco_3D_ori [2])], 
                               [0, math.sin(self.aruco_3D_ori [2]),  math.cos(self.aruco_3D_ori [2])]])


                Ry = np.array([[math.cos(self.aruco_3D_ori [1]),  0, math.sin(self.aruco_3D_ori [1])], 
                                [0,                                1,                              0], 
                                [-math.sin(self.aruco_3D_ori [1]), 0, math.cos(self.aruco_3D_ori [1])]])

                
                Rz = np.array([[math.cos(self.aruco_3D_ori [0]), -math.sin(self.aruco_3D_ori [0]), 0], 
                               [math.sin(self.aruco_3D_ori [0]),  math.cos(self.aruco_3D_ori [0]), 0], 
                               [0,                                              0,                 1]])
                
                R_ = np.array([[0, 0, 1],
                          [-1, 0, 0],
                          [0, -1, 0]])

                R = np.dot(R_, np.dot(Rx,np.dot(Ry,Rz)))
                # T_aru_op = np.eye(4)
                # T_aru_op[:3,:3] = R
                # T_aru_op[:3,3] = self.aruco_3D_pos 

                # T_op_tcam = np.eye(4)
                # T_op_tcam[:3,:3] = np.array([[0, 0, 1],
                #                              [-1, 0, 0],
                #                              [0, -1, 0]])


                # T_ar_tcam = np.dot(T_op_tcam,T_aru_op)

                yaw = math.atan2(R[1,0],R[0,0])
                pitch = math.atan2(-R[2,0],math.sqrt(R[2,1]**2 + R[2,2]**2))
                roll = math.atan2(R[2,1],R[2,2])     
                quat = self.to_quat(roll, pitch, yaw)

                try:
                    self.br.sendTransform((self.aruco_3D_pos[0], self.aruco_3D_pos[1],self.aruco_3D_pos[2]),
                                          list(quat),
                                          rospy.Time.now(),
                                          "marker_in_eye",
                                          self.frame_id)
                    (self.trans_marker_torso, self.rot_marker_torso) = self.listener.lookupTransform('torso','marker_in_eye',rospy.Time(0))
                    # self.call_service(self.trans_marker_torso)
                except:
                    pass


        else:
            self.aruco_3D_pos = None
            self.aruco_3D_ori = None
            self.ids = None
            self.trans_marker_torso = None
        
        # content = cv2.imread('path_to_virtual_content.png')
        self.top_img = cv2.aruco.drawDetectedMarkers(self.top_img, cor, self.ids, (0, 255, 0))
        cv2.imshow('Aruco Marker', self.top_img)
        cv2.waitKey(1)
        


    def image_callback_top(self, msg):
        self.top_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.frame_id = msg.header.frame_id

        self.arucoFunction()


    def call_service(self, pos):
        rospy.wait_for_service('/move_joints_2')

        try:
           
            if self.aruco_3D_pos is not None:

                proxy = rospy.ServiceProxy('/move_joints_2', MoveJoints)

                resp = None
                # resp = proxy("LArm", [0.2, 0.2, 0.2], None, 1.0) # initial values. need to replace 
                if pos[1] >= 0.08:
                    print('Moving Left')

                    resp = proxy(["LArm"], [pos[0], pos[1], pos[2], 0.0, 0.0, 0.0], 0.4, None) # initial values. need to replace
                    #resp = proxy("LArm", [pos[0], pos[1], pos[2]], None, 0.5) # initial values. need to replace 
                    
                elif pos[1] <= -0.08:

                    print('Moving Right')
                    resp = proxy(["RArm"], [pos[0], pos[1], pos[2], 0.0, 0.0, 0.0], 0.4, None)

                else: 
                    print('Moving Both')
                    resp = proxy(["LArm","RArm"], [pos[0], pos[1], pos[2], 0.0, 0.0, 0.0], 0.8, None) # initial values. need to replace 

                   
               
                #print("Note: ", resp.pose_6D)
                return resp.pose_6D

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        

if __name__ == '__main__':

    rospy.init_node('client_node')
    client_ = CameraClient()

    rpy = client_.quaternion_to_euler(q = [0.39, 0.46, 0.66, -0.42])

    while not rospy.is_shutdown():
        try:
            if client_.aruco_3D_pos is not None and client_.trans_marker_torso is not None: # and client_.run_once == True: 
                client_.call_service(client_.trans_marker_torso)
                #client_.run_once = False
                rospy.sleep(0.5)

        except:
            pass

        # rospy.spin()

    cv2.destroyAllWindows()    
        