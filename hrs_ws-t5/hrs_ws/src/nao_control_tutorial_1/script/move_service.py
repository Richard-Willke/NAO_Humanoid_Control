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

'''
[
HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw,
LHand, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch,
RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll,
RElbowYaw, RElbowRoll, RWristYaw, RHand
]
'''

#TODO: create service handler
class NaoRobot():
    def __init__(self,  IP, Port):

        self.robotIP = IP #"10.152.246.114"
        self.PORT = Port #9559

        self.fractionMaxSpeed  = 0.1
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.setJointState, queue_size = 10)
        self.currentPose = JointState()
        self.response = MoveJointsResponse()

        self.max_values = np.array([320, 240])
        self.imageCenter = np.array([self.max_values[0]/2, self.max_values[1]/2])
        self.min_values = np.array([0, 0])

        self.motionProxy = ALProxy("ALMotion", self.robotIP, 9559)
        

    def setJointState(self, msg):
        self.currentPose = np.array([msg.position[0], msg.position[1]])

    def limiterFunction(self, inputs):
        output1 =  max(-0.5, min(0.5, inputs[0]/self.max_values[0]))
        output2 =  max(-0.5, min(0.5, inputs[1]/self.max_values[1]))
        return output1, output2

    def setAngles(self,req):

        if req.mode == 1:
            # NOTE: The right arm does not work for the robot used in our tests. The program does work without errors nevertheless 
            names = ["LShoulderPitch", "LShoulderRoll","RShoulderPitch","RShoulderRoll"]
            self.motionProxy.setStiffnesses(names, 1.0)
            self.motionProxy.setAngles(names, [0.2, 0.2, 0.2, 0.2], self.fractionMaxSpeed)            
            self.response.current_pose = list(req.target_pose)
            return self.response
        
        elif req.mode == 2:
            if req.set_angles:
                names = ["LShoulderPitch", "LShoulderRoll"]
                self.motionProxy.setStiffnesses(names, 1.0)
                self.motionProxy.angleInterpolation(names,[np.radians(30), 0],[2.0, 1.0],True) 

            else:
                names = ["LShoulderPitch", "LShoulderRoll"]
                self.motionProxy.setStiffnesses(names, 1.0)
                self.motionProxy.setAngles(names, [np.radians(30), 0], self.fractionMaxSpeed)            
            
            self.response.current_pose = list(req.target_pose)
            return self.response
        
        elif req.mode == 3:
            if req.set_angles:
                names = ["HeadYaw", "HeadPitch"]
                self.motionProxy.setStiffnesses(names, 1.0)
                angle_diff1, angle_diff2 = self.limiterFunction(np.array([(self.imageCenter[0] - req.target_pose[0]),(req.target_pose[1] - self.imageCenter[1])]))
                self.motionProxy.angleInterpolation(names, 
                                                    [max(-1.7, min(1.7, self.currentPose[0] + angle_diff1)), 
                                                     max(-0.5, min(0.4, self.currentPose[1] + angle_diff2))], 
                                                    [0.2, 0.2],
                                                    True) 
                taskList = self.motionProxy.getTaskList()
                try:
                    uiMotion = taskList[0][1]
                    motion_service.killTask(uiMotion)
                    self.motionProxy.setStiffnesses(names, 0.0)
                except:
                    pass

            else:
                names = ["HeadYaw", "HeadPitch"]
                self.motionProxy.setStiffnesses(names, 1.0)
                angle_diff1, angle_diff2 = self.limiterFunction(np.array([(self.imageCenter[0] - req.target_pose[0]),(req.target_pose[1] - self.imageCenter[1])]))
                self.motionProxy.setAngles(names, 
                                           [max(-2.0, min(2.0, self.currentPose[0] + angle_diff1)), 
                                            max(-0.6, min(0.5, self.currentPose[1] + angle_diff2))],  
                                           self.fractionMaxSpeed)            
            
            self.response.current_pose = list(req.target_pose)
            return self.response
        
        else:
            rospy.ServiceException("Incorrect mode, mode can only take values from 1 to 3")
            return None

if __name__ == '__main__':
    
    rospy.init_node('server_move_joints')
    naoBot = NaoRobot(IP = "10.152.246.194", Port = 9559)
    server = rospy.Service('/move_joints', MoveJoints, naoBot.setAngles)

    while not rospy.is_shutdown():
        pass

    rospy.spin()
    
    
			
		
