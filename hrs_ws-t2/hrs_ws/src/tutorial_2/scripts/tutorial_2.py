#!/usr/bin/env python2
import cv2 
import rospy
from sensor_msgs.msg import Image
import cv_bridge
# from cv_bridge import CvBridge

class Nao:
    def __init__(self):
        self.image_subscriber = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.image_callback, queue_size = 10)
        self.image_publisher = rospy.Publisher("/nao_custom_image", Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()
          
    def image_callback(self, msg):
        img_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray_pic = cv2.cvtColor(img_msg, cv2.COLOR_BGR2GRAY)
    
        cv2.imshow("Original image", cv2.WINDOW_NORMAL)
        cv2.imshow("Original image", img_msg)

        cv2.imshow("Greyscale image", cv2.WINDOW_NORMAL)
        cv2.imshow("Greyscale image", gray_pic)

        thresh, binary_pic = cv2.threshold(gray_pic, 120, 255, cv2.THRESH_BINARY)
        cv2.imshow("Binary image", cv2.WINDOW_NORMAL)
        cv2.imshow("Binary image", binary_pic)

        cv2.waitKey(3)


if __name__ == "__main__":

    nao = Nao()

    rospy.init_node('Tutorial_2')

    while not rospy.is_shutdown():

        rospy.spin()