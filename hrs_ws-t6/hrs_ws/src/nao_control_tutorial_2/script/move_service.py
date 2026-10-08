#!/usr/bin/env python
import rospy
import tf
import time
import motion
import numpy
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_2.srv import MoveJoints, MoveJointsResponse
from sensor_msgs.msg import JointState, Image
import numpy as np

motionProxy =0

class NaoRobot():
    def __init__(self, IP, Port):

        self.robotIP = IP # "10.152.246.114"
        self.PORT = Port  # 9559

        self.fractionMaxSpeed  = 0.1
        self.targetTime = None
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.getJointPosition, queue_size = 10)
        self.currentPose = JointState()
        self.response = MoveJointsResponse()

        self.max_values = np.array([320, 240])
        self.imageCenter = np.array([self.max_values[0]/2, self.max_values[1]/2])
        self.min_values = np.array([0, 0])

        self.motionProxy = ALProxy("ALMotion", self.robotIP, self.PORT)

        self.joints_positions = []
        self.target_name = None
        self.joints_names = []

        self.listener = tf.TransformListener()
        self.poses = []


    def setService(self, req):

        #print("getting req: ", req)
        
        pathList = []
        nameList = []

        if len(req.desired_pose_6D) == 3:
            if req.max_velocity_fraction != 0.0:
                raise("value error, pose expected for setPosition")
            
            elif req.execution_time != 0.0:
                # checked
                self.motionProxy.setStiffnesses(req.joint_name, 0.5)                
                currentPos = self.motionProxy.getPosition(req.joint_name, motion.FRAME_TORSO, True)
                target_pos = almath.Position6D(currentPos)
                target_pos.x = req.desired_pose_6D[0]
                target_pos.y = req.desired_pose_6D[1]
                target_pos.z = req.desired_pose_6D[2]
                pathList.append(list(target_pos.toVector()))
                target_pos = [target_pos.x, target_pos.y, target_pos.z]

                self.motionProxy.positionInterpolations(req.joint_name, motion.FRAME_TORSO, pathList, [7], [req.execution_time])
                # self.motionProxy.positionInterpolations([req.joint_name], motion.FRAME_TORSO, [list(targetPos.toVector())], [7], [req.execution_time])

        elif len(req.desired_pose_6D) == 6:

            if req.max_velocity_fraction != 0.0:

                curr_pos = self.motionProxy.getPosition(req.joint_name[0], motion.FRAME_TORSO, True)
                target_pos = req.desired_pose_6D # list(0.2 * (np.array(req.desired_pose_6D) - np.array(curr_pos)))  # 
                
                if len(req.joint_name) == 1:

                    if req.joint_name[0] == "RArm":
                        self.motionProxy.setStiffnesses(req.joint_name[0], 0.8)
                        self.motionProxy.setStiffnesses("LArm", 0.0)
                        self.motionProxy.setPosition(req.joint_name[0], 0, list(target_pos), req.max_velocity_fraction, 63)


                    elif req.joint_name[0] == "LArm":
                        self.motionProxy.setStiffnesses(req.joint_name[0], 0.8)
                        self.motionProxy.setStiffnesses("RArm", 0.0)
                        self.motionProxy.setPosition(req.joint_name[0], 0, list(target_pos), req.max_velocity_fraction, 63)
                    
                elif len(req.joint_name) > 1:
                    self.motionProxy.setStiffnesses(req.joint_name[0], 0.5)
                    self.motionProxy.setStiffnesses(req.joint_name[1], 0.5)
                    self.motionProxy.setPosition(req.joint_name[0], 0, list(target_pos), req.max_velocity_fraction, 63)
                    self.motionProxy.setPosition(req.joint_name[1], 0, list(target_pos), req.max_velocity_fraction, 63)

        
        else:
            raise("invalid number of arguments in desired pose 6D. should be either of length 3 or 6!")
        
        self.response.pose_6D = self.motionProxy.getPosition(req.joint_name[0], 0, True)    # self.motionProxy.getPosition(req.joint_name, 0, True)
        #print("Note: ", self.response.pose_6D)
        return self.response
    
    # def getPose(self):
        # idx_target_joint = self.joints_names.index(self.target_name)
        # return target_joint_position = self.joint_position[idx_target_joint]
        # {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
        
        
    def getJointPosition(self, msg):
        # self.joints_positions = msg.position
        self.joints_name = msg.name


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    rospy.init_node('move_joints_server')
    naoBot = NaoRobot(robotIP, PORT)
    server = rospy.Service('/move_joints_2', MoveJoints, naoBot.setService)
    #rospy.sleep(5)
    rospy.sleep(5)
    rospy.spin()
			
		
