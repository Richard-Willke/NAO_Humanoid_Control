#!/usr/bin/env python2
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from naoqi import ALProxy
from naoqi import ALBroker
import tf
from utils.utils import *



""" Implementation of Kalman Filter to track the COM of Nao """

class KalmanFilter:
    '''
    Usage: Create an object of this class and call update() method

    Return: A tuple, ([x, y, z], [r, p, y])
    '''
    # State vector: [x, y, dx, dy]
    # Measurement vector: [x, y]
    # Control input: [ddx, ddy]
    
    def __init__(self):
        # flags
        self.wait_logged = False
        self.proc_logged = False
        self.has_imu = False
        
        # for nao apis
        # global mem
        # mem = ALProxy("ALMemory")

        # initial vals
        self.imu = np.zeros(2)
        self.imu_cov = np.zeros([2, 2])

        # ROS
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.pred_pub = rospy.Publisher("/pred_position", Marker, queue_size=10)

        self.listener = tf.TransformListener()

        # kalman filter params
        self.kf = cv2.KalmanFilter(dynamParams=4, measureParams=2, controlParams=2)
        dt = 1.0/20
        # A
        self.kf.transitionMatrix = np.array([[1, 0, dt,  0],
                                             [0, 1,  0, dt],
                                             [0, 0,  1,  0],
                                             [0, 0,  0,  1]], dtype=np.float32)
        # B
        self.kf.controlMatrix = np.array([[0.5 * (dt**2), 0],
                                          [0, 0.5 * (dt**2)],
                                          [dt, 0],
                                          [0, dt]], np.float32)
        # H
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                              [0, 1, 0, 0]], np.float32)
        # Q 
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05
        # R
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.02
        # P
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        # State
        self.kf.statePost = np.zeros(4, dtype=np.float32)

    def update(self):
        if self.has_imu:
            if not self.proc_logged:
                rospy.loginfo("imu data received, start processing")
                self.proc_logged = True

            try:

                trans_odom_torso = None
                while trans_odom_torso == None:
                    self.listener.waitForTransform("odom", "torso", rospy.Time(), rospy.Duration(4.0))
                    (trans_odom_torso, quat_odom_torso) = self.listener.lookupTransform('odom','torso', rospy.Time())

                blur_posi = np.array(trans_odom_torso[0:2], dtype=np.float32)
                corr_posi = self.correct(blur_posi)

                #pred_posi has a shape 2x2 each row has position and velocity
                pred_posi = self.predict(self.imu.reshape(2,1))
                self.publish(pred_posi)
                
                euler_odom_torso = quaternion_to_euler(quat_odom_torso)#euler is [ roll, pitch, yaw]
                predicted_position = [pred_posi[0][0], pred_posi[1][0], trans_odom_torso[2]]

                return predicted_position, quat_odom_torso


            except:

                rospy.logerr("something bad happened")
        
                pass

            pass

        elif not self.wait_logged:

            rospy.logwarn("no command data received, waiting")

            self.wait_logged = True

        self.has_imu = False


    def correct(self, measurement):
        '''
        Correct the measurement
        
        Args:
            - measurement: numpy.ndarray, dtype=numpy.float32
        '''
        self.kf.correct(measurement)

    def predict(self, control_input):
        '''
        Predict the next state
        
        Args:
            - control_input: numpy.ndarray, dtype=numpy.float32
        '''
        return self.kf.predict(control = control_input)

    def publish(self, prediction):
        position = Marker()
        position.header.frame_id = "odom"
        position.header.stamp = rospy.Time.now()
        position.ns = "my_namespace"
        position.id = 0
        position.type = Marker.SPHERE
        position.action = Marker.ADD
        position.pose.position.x = prediction[0][0]
        position.pose.position.y = prediction[1][0]
        position.pose.position.z = 0.0
        position.pose.orientation.x = 0.0
        position.pose.orientation.y = 0.0
        position.pose.orientation.z = 0.0
        position.pose.orientation.w = 1.0
        position.scale.x = 0.1
        position.scale.y = 0.1
        position.scale.z = 0.1
        position.color.a = 1.0
        position.color.r = 0.0
        position.color.g = 0.0
        position.color.b = 1.0
        position.lifetime = rospy.Duration.from_sec(3.0)

        self.pred_pub.publish(position)

    def imu_callback(self, msg):
        self.has_imu = True
        self.imu = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y], dtype=np.float32)
        

def main():

    motionProxy =0

    ROBOT_IP = '10.152.246.156'
    PORT = 9559
    mem = None
    myBroker = ALBroker("myBroker",
        "0.0.0.0",   # listen to anyone
        0,           # find a free port and use it
        ROBOT_IP,          # parent broker IP
        PORT)        # parent broker port

    rospy.init_node('kalman_filter_node')
    rate = rospy.Rate(20)
    kf = KalmanFilter()
    while not rospy.is_shutdown():
        try: 
            kf.update()
        except KeyboardInterrupt:
            myBroker.shutdown()
        pass
        rate.sleep()



if __name__ == '__main__':
    main()