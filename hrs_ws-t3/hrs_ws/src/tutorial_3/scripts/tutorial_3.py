#!/usr/bin/env python2
import cv2 
import rospy
from sensor_msgs.msg import Image
import cv_bridge
# from cv_bridge import CvBridge
import numpy as np
from collections import deque
import imutils

class Nao:
    def __init__(self):
       
        self.image_subscriber = rospy.Subscriber("/nao_robot/camera/bottom/camera/image_raw", Image, self.image_callback, queue_size = 10)
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

        self.bottom_image = None
          
    def erode_and_dilate(self, threshold):
        # Creating kernel 
        kernel = np.ones((5, 5), np.uint8) 
        
        # Using cv2.erode() method  
        eroded_image = cv2.erode(threshold, kernel)  
        
        # Displaying the image  
        cv2.imshow("Eroded Image", eroded_image)  
        
        # Using cv2.erode() method  
        dilated_image = cv2.dilate(eroded_image, kernel)  
        
        # Displaying the image  
        cv2.imshow("Dilated Image", dilated_image)  

    def extract_colors(self, hsv_pic):
        lower_bound_red = np.array([0, 104, 151])
        upper_bound_red = np.array([80, 255, 255])
        hsv_red = cv2.inRange(hsv_pic, lower_bound_red, upper_bound_red)
        cv2.imshow("Color Extraction Red", cv2.WINDOW_NORMAL)
        cv2.imshow("Color Extraction Red", hsv_red)

        lower_bound_blue = np.array([90, 100, 90])
        upper_bound_blue = np.array([128, 255, 255])
        hsv_blue = cv2.inRange(hsv_pic, lower_bound_blue, upper_bound_blue)
        cv2.imshow("Color Extraction Blue", cv2.WINDOW_NORMAL)
        cv2.imshow("Color Extraction Blue", hsv_blue)

        lower_bound_green = np.array([36, 50, 70]) 
        upper_bound_green = np.array([89, 255, 255]) 
        hsv_green = cv2.inRange(hsv_pic, lower_bound_green, upper_bound_green)
        cv2.imshow("Color Extraction Green", cv2.WINDOW_NORMAL)
        cv2.imshow("Color Extraction Green", hsv_green)

    def blob_extraction(self, img_msg, threshold):
        points = deque(maxlen=30)
        contours = cv2.findContours(threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(contours)
        center = None

        if len(cnts) > 0:
            c = max(cnts, key = cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            print("Pixel Coordinates of Tracked Ob ject are x: %s and y %s"%(x,y))
            if radius >10:
                cv2.circle(img_msg, (int(x),int(y)), 30, (0,0,255), 2)
        points.append(center)
        for i in range(1, len(points)):
            if points[i-1] is None or points[i] is None:
                continue
            thickness = int(np.sqrt(30/float(i+1))*2.5)
            cv2.line(img_msg, points[i-1], points[i], (0,0,255), thickness)
        cv2.imshow("Blob Extraction", img_msg)

    def image_callback(self, msg):
        
        img_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        self.bottom_image = img_msg

    

    def image_callback_top(self, msg):
        # tasks 2 (TOP CAMERA)
        img_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        gray_image = cv2.cvtColor(img_msg, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.medianBlur(gray_image,5)

        blurerd_img = cv2.GaussianBlur(img_msg, (11,11), 0)
        hsv_pic = cv2.cvtColor(blurerd_img, cv2.COLOR_BGR2HSV)
        
        lower_bound = np.array([36, 50, 70]) #np.array([100, 150, 0]) #np.array([0, 104, 171])
        upper_bound = np.array([89, 255, 255]) #np.array([140, 255, 255]) #np.array([75, 255, 255])

        threshold = cv2.inRange(hsv_pic, lower_bound, upper_bound)

        # TASK 1
        cv2.imshow("Top Camera", cv2.WINDOW_NORMAL)
        cv2.imshow("Top Camera", img_msg)   

        # TASK 2 and 3 (Color extraction for Red Green and Blue)
        self.extract_colors(hsv_pic)
        
        # TASK 4
        self.erode_and_dilate(threshold)

        # TASK 5
        self.blob_extraction(img_msg, threshold)

        # TASK 6 
        if self.bottom_image is not None:

            gray_image = cv2.cvtColor(self.bottom_image, cv2.COLOR_BGR2GRAY)
            gray_blur = cv2.medianBlur(gray_image,5)

            blurerd_img = cv2.GaussianBlur(self.bottom_image, (11,11), 0)
            hsv_pic = cv2.cvtColor(blurerd_img, cv2.COLOR_BGR2HSV)
            
            lower_bound = np.array([15, 144, 171]) # np.array([36, 50, 70]) #np.array([100, 150, 0]) #np.array([0, 104, 171])
            upper_bound = np.array([80, 255, 255]) # np.array([89, 255, 255]) #np.array([140, 255, 255]) #np.array([75, 255, 255])

            threshold = cv2.inRange(hsv_pic, lower_bound, upper_bound)
            
            cv2.imshow("Bottom Camera", cv2.WINDOW_NORMAL)
            cv2.imshow("Bottom Camera", self.bottom_image)

            # TASK 7
            circles = cv2.HoughCircles(gray_blur,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=30,maxRadius=50)

            if circles is not None: 
                #circles = np.uint16(np.around(circles))
                for i in circles[0,:]:
                    # draw the outer circle
                    cv2.circle(self.bottom_image,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),2)
                    # draw the center of the circle
                    cv2.circle(self.bottom_image,(int(i[0]),int(i[1])),2,(0,0,255),3)
            
            cv2.imshow('Circular Shapes', self.bottom_image)
    

        cv2.waitKey(1)

if __name__ == "__main__":

    nao = Nao()

    rospy.init_node('Tutorial_2')

    rospy.spin()
    
    #while not rospy.is_shutdown():

        