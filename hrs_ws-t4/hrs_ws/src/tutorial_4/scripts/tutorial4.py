#!/usr/bin/env python2
import cv2 
import rospy
from sensor_msgs.msg import Image
import cv_bridge
# from cv_bridge import CvBridge
import numpy as np
from collections import deque
import imutils
import matplotlib.pyplot as plt

class Nao:
    def __init__(self):
        self.image_subscriber = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.image_callback_top, queue_size = 10)
        
        self.image_publisher = rospy.Publisher("/nao_custom_image", Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()

        """
        color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
              'white': [[180, 18, 255], [0, 0, 231]],
              'red1': [[180, 255, 255], [159, 50, 70]],
              'red2': [[9, 255, 255], [0, 50, 70]],
              'green': [[89, 255, 255], [36, 50, 70]],
              'blue': [[128, 255, 255], [90, 50, 70]],
              'yellow': [[35, 255, 255], [25, 50, 70]],
              'purple': [[158, 255, 255], [129, 50, 70]],
              'orange': [[24, 255, 255], [10, 50, 70]],
              'gray': [[180, 18, 230], [0, 0, 40]]}
        """

        self.bottom_image = np.zeros((320,240))
        self.cropped_image = np.zeros((640,480))
        self.hsv_pic = np.zeros((640,480))
        self.B = np.zeros(4)
        self.track_image= np.zeros((640,480))
        self.hist_value = None
        self.h = np.zeros((640,480))
        self.s = np.zeros((640,480))
        self.v = np.zeros((640,480))

        self.r = np.zeros(4)
        self.initialized_flag = False
        self.previous_img = np.zeros((640,480))

        self.termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 ) 

        self.lower_bound = 50 
        self.upper_bound = 255

        self.max_corners = 100 
        self.quality_level = 0.01 
        self.min_distance = 10
    
    def initialize_templates(self):
        # Select ROI 
        self.r = cv2.selectROI("select the area", self.img_msg) 
        
        # Crop image 
        self.cropped_image = self.img_msg[int(self.r[1]):int(self.r[1]+self.r[3]),  
                            int(self.r[0]):int(self.r[0]+self.r[2])] 

        self.cropped_image_hsv = cv2.cvtColor(self.cropped_image, cv2.COLOR_BGR2HSV)

        self.h, self.s,self.v = cv2.split(self.cropped_image_hsv)

        mask = cv2.inRange(self.s, self.lower_bound, self.upper_bound)

        self.hist_value = cv2.calcHist([self.h], [0], mask, [181],  [0, 181])

        self.track_window_mean = (int(self.r[0]), int(self.r[1]), int(self.r[2]), int(self.r[3]))
        self.track_window_cam = (int(self.r[0]), int(self.r[1]), int(self.r[2]), int(self.r[3]))

        #new_hist = []
        #for i in range(len(hist_value)):
        #    new_hist.append(hist_value[i][0])

        
        #normed_hist = [(float(i)-min(new_hist))/(max(new_hist)-min(new_hist)) for i in new_hist]

        #fig, ax = plt.subplots()
        #ax.plot(normed_hist)
        #ax.set_title('Normalized Histogram')
        #plt.show()
        
        cv2.imwrite('/workspaces/hrs/hrs_ws/templateImage.png', self.img_msg)
        cv2.imwrite('/workspaces/hrs/hrs_ws/3.png', self.cropped_image)
        cv2.imwrite('/workspaces/hrs/hrs_ws/4.png', self.hsv_pic)
        cv2.imwrite('/workspaces/hrs/hrs_ws/5_B.png', self.B)
        cv2.imwrite('/workspaces/hrs/hrs_ws/6.png', self.track_image)
        cv2.destroyAllWindows()

        self.initialized_flag = True

    def meanShift(self):
        # task 6 meanshift
        new_img = self.img_msg.copy()
        ret, self.track_window_mean = cv2.meanShift(self.B, 
                                        self.track_window_mean, 
                                        self.termination) 
        x,y,w,h = self.track_window_mean
        color = (255, 0, 0)
        self.track_image = cv2.rectangle(new_img, (x,y), (x+w,y+h), color,2)
        cv2.imshow('Mean Shift', new_img)

    def camShift(self):
        #task 7 camshift
        new_img = self.img_msg.copy()
        ret, self.track_window_cam = cv2.CamShift(self.B, self.track_window_cam, self.termination) 
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)
        color = (0, 0, 255)
        self.track_image = cv2.polylines(new_img,[pts],True, color,2)
        cv2.imshow('Cam Shift', new_img)

    def opticalFlow(self):
        # task 9 opticalFlow
        new_img = self.img_msg.copy()
        previous_gray = cv2.cvtColor(self.previous_img, cv2.COLOR_BGR2GRAY)
        next_gray = cv2.cvtColor(new_img, cv2.COLOR_BGR2GRAY)
        
        corners = cv2.goodFeaturesToTrack(next_gray, self.max_corners, self.quality_level, self.min_distance) 
        corners = np.float32(corners).reshape(-1, 1, 2)

        new_points, status, error = cv2.calcOpticalFlowPyrLK(previous_gray, next_gray, corners, None, criteria=self.termination)

        for i, (new, old) in enumerate(zip(new_points, corners)):
            a, b = new.ravel()
            c, d = old.ravel()
            # Draw a line connecting the old position to the new position (motion trace)
            cv2.line(new_img, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
            # Draw a circle at the new position
            cv2.circle(new_img, (int(a), int(b)), 5, (255, 0, 255), -1)

        cv2.imshow('Optical Flow', new_img)

    def arucoFunction(self):
        # task 10 arucoFunction
        camera_matrix = np.array([551.543059, 0.0, 327.382898, 0.0, 553.736023, 225.02638, 0.0, 0.0, 1]).reshape(3, 3)
        dist_coeffs = np.array([-0.066494, 0.095481, -0.000279, 0.002292, 0.0]).reshape(1, 5)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL) 
        parameters = cv2.aruco.DetectorParameters_create() 
        gray = cv2.cvtColor(self.top_img, cv2.COLOR_BGR2GRAY)
        cor, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            rvecs, tvecs, trash= cv2.aruco.estimatePoseSingleMarkers(cor, 0.05, camera_matrix, dist_coeffs)
            for i, marker_id in enumerate(ids):
                cv2.aruco.drawAxis(self.top_img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)
                print('3D space position = ', tvecs[i])
        
        content = cv2.imread('path_to_virtual_content.png')
        projected_content = cv2.aruco.drawDetectedMarkers(self.top_img, cor, ids, (0, 255, 0))
        cv2.imshow('Aruco Marker', projected_content)


    def image_callback_top(self, msg):
        
        # NOTE: 
        # If you want to access the full functionality of the code, first initialize by pressing "a" on the keyboard and choosing an object as a template. 
        # Confirm with "ENTER".

        self.img_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.top_img = self.img_msg.copy()
        cv2.imshow('Top Camera', self.top_img)
  
        self.hsv_pic = cv2.cvtColor(self.img_msg, cv2.COLOR_BGR2HSV)

        if cv2.waitKey(33) == ord('a'):
            self.initialize_templates()

        if self.hist_value is not None:

            hist_value_norm = cv2.normalize(self.hist_value,None,0,255,cv2.NORM_MINMAX)
            self.B = cv2.calcBackProject([self.hsv_pic],[0], hist_value_norm,[0,180],1)
    
        if self.initialized_flag == True:
            cv2.imshow('cropped_image', self.cropped_image)
            cv2.imshow('hsv', self.hsv_pic)
            cv2.imshow('BackProject', self.B)

            ### NOTE: uncomment either one of the four to run the code of the desired functions 
            
            self.camShift()
            self.meanShift() 
            self.opticalFlow()
            self.arucoFunction()
            
            ###################################################################################

        self.previous_img = self.img_msg
        cv2.waitKey(1)

if __name__ == "__main__":

    nao = Nao()

    rospy.init_node('Tutorial_2')

    rospy.spin()
    

        