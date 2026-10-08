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
    
    def initialize_templates(self):
        # Select ROI 
        self.r = cv2.selectROI("select the area", self.img_msg) 
        
        # Crop image 
        self.cropped_image = self.img_msg[int(self.r[1]):int(self.r[1]+self.r[3]),  
                            int(self.r[0]):int(self.r[0]+self.r[2])] 

        #track_window = (int(self.r[0]), int(self.r[1]), int(self.r[3]), int(self.r[2]))
        #print("track window:", track_window)

        self.cropped_image_hsv = cv2.cvtColor(self.cropped_image, cv2.COLOR_BGR2HSV)

        self.h, self.s,self.v = cv2.split(self.cropped_image_hsv)

        lower_bound = 50 
        upper_bound = 255
        mask = cv2.inRange(self.s, lower_bound, upper_bound)

        self.hist_value = cv2.calcHist([self.h], [0], mask, [181],  [0, 181])

        self.track_window = (int(self.r[0]), int(self.r[1]), int(self.r[2]), int(self.r[3]))

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


    def image_callback_top(self, msg):
        self.img_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # TASK 2
        cv2.imshow("templateImage", cv2.WINDOW_NORMAL)
        cv2.imshow("templateImage", self.img_msg)   
    
        self.hsv_pic = cv2.cvtColor(self.img_msg, cv2.COLOR_BGR2HSV)

        if cv2.waitKey(33) == ord('a'):
            self.initialize_templates()

        
        
        

        '''
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250) 
        parameters = cv2.aruco.DetectorParameters_create() 
        gray = cv2.cvtColor(self.img_msg, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is not None:
            rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        
        content = cv2.imread('path_to_virtual_content.png')
        projected_content = cv2.aruco.drawDetectedMarkers(self.img_msg, corners, ids, (0, 255, 0))
        cv2.imshow('Projected Content', projected_content)

        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        '''

        if self.hist_value is not None:

            hist_value_norm = cv2.normalize(self.hist_value,None,0,255,cv2.NORM_MINMAX)
            self.B = cv2.calcBackProject([self.hsv_pic],[0], hist_value_norm,[0,180],1)
            # termination criteria, either 15 iteration or by at least 2 pt 
            
            termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 ) 

            #ret, self.track_window = cv2.meanShift(self.B, 
            #                                self.track_window, 
            #                                termination) 


            x,y,w,h = self.track_window
            #self.track_image = cv2.rectangle(self.img_msg, (x,y), (x+w,y+h), 255,2)


                                
            ret, self.track_window = cv2.CamShift(self.B, self.track_window, termination)

 
            pts = cv2.boxPoints(ret)
            pts = np.int0(pts)
            self.track_image = cv2.polylines(self.img_msg,[pts],True, 255,2)

            # self.track_image = cv2.rectangle(self.img_msg, 
            #                             (int(self.track_window[0]),int(self.track_window[1])), 
            #                             (int(self.track_window[0])+int(self.track_window[2]),int(self.track_window[1])+int(self.track_window[3])), 
            #                             255, 2)


            # task 9 (missing calcOpticalFlowPyrLK)
            previous_gray = cv2.cvtColor(self.previous_img, cv2.COLOR_BGR2GRAY)
            next_gray = cv2.cvtColor(self.img_msg, cv2.COLOR_BGR2GRAY)
            max_corners = 100 
            quality_level = 0.01 
            min_distance = 10
            corners = cv2.goodFeaturesToTrack(next_gray, max_corners, quality_level, min_distance) 
            feature_points = np.float32(feature_points).reshape(-1, 1, 2)

            # for corner in corners: 
            #     x, y = corner.ravel() 
            #     cv2.circle(self.img_msg, (x, y), 3, 255, -1) 
            # cv2.imshow('Corners', self.img_msg)

            #next_image = self.img_msg #self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            #next_gray_image = cv2.cvtColor(next_image, cv2.COLOR_BGR2GRAY)
            #roi_gray = gray[y:y+h,x:x+w]

            new_points, status, error = cv2.calcOpticalFlowPyrLK(previous_gray, next_gray, corners, None, criteria=termination)

            for i, (new_point, old_point) in enumerate(zip(new_points, roi_points)):
                a, b = new_point.ravel()
                cv2.circle(next_image, (int(a), int(b)), 5, (0, 255, 0), 2)
            
            cv2.imshow('Optical Flow', next_image)

        cv2.imshow('/workspaces/hrs/hrs_ws/templateImage.png', self.img_msg)
        if self.initialized_flag == True:
            cv2.imshow('cropped_image.png', self.cropped_image)
            cv2.imshow('hsv.png', self.hsv_pic)
            cv2.imshow('BackProject.png', self.B)
            cv2.imshow('camshift.png', self.track_image)


        self.previous_img = self.img_msg
        cv2.waitKey(1)

if __name__ == "__main__":

    nao = Nao()

    rospy.init_node('Tutorial_2')

    rospy.spin()
    

        