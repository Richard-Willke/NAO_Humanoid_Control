#!/usr/bin/env python
import rospy
import time
import almath
import math
import sys
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from naoqi_bridge_msgs.msg import HeadTouch

motionProxy =0

ROBOT_IP = '10.152.246.114'
PORT = 9559
memory = None
myBroker = ALBroker("myBroker",
       "0.0.0.0",   # listen to anyone
       0,           # find a free port and use it
       ROBOT_IP,          # parent broker IP
       PORT)        # parent broker port


class NaoControl:

    def __init__(self, mode=1):
        self.head_front_touched = False
        self.head_middle_touched = False
        self.head_back_touched = False
        self.tactile_sub = rospy.Subscriber("/tactile_touch", HeadTouch, self.tactileCallback)
        self.mode = mode
        self.asr = ALProxy("ALSpeechRecognition", ROBOT_IP, PORT)
        self.atts = ALProxy("ALTextToSpeech", ROBOT_IP, PORT)
        self.am = ALProxy("ALMotion", ROBOT_IP, PORT)
        self.app = ALProxy("ALRobotPosture", ROBOT_IP, PORT)
        self.app.goToPosture("StandInit", 0.5)
        global memory
        memory = ALProxy("ALMemory")
        self.wtf = []
        self.num_steps = 6
        pass

    def update(self):
        if self.mode == 1:
            if self.head_front_touched:
                self.speechRecognition()
            elif self.head_middle_touched:
                self.readItOutLoud()
            elif self.head_back_touched:
                
                self.walk()
        else:
            print('waiting for your touch')
            time.sleep(1)
        pass

    def speechRecognition(self):
        # print("I'm touched, speak to me")
        self.asr.setLanguage("English")
        vocabulary = ["yes", "no", "please", "hello", "hi"]
        try:
            self.asr.setVocabulary(vocabulary, True)
        except:
            self.asr.unsubscribe("Test_ASR")
            self.asr.setVocabulary(vocabulary, True)
        self.asr.subscribe("Test_ASR")
        # print 'Speech recognition engine started'
        time.sleep(2)
        # print("Got you", memory.getData("WordRecognized"))
        heard = memory.getData("WordRecognized")[0][6: -5]
        print(type(heard), heard)
        self.asr.unsubscribe("Test_ASR")
        self.wtf.append(heard)
        print(self.wtf)

    def readItOutLoud(self):

        for i in range(len(self.wtf)):
            self.atts.say(self.wtf[i])
            time.sleep(1)
        self.wtf = []

    def walk(self):
        self.head_back_touched = False
        x  = 0.8
        y  = 0.5
        theta  = 0
        self.am.moveTo(x, y, theta)

    def tactileCallback(self, msg):
        if msg.state == 1 and msg.button == 1:
            self.head_front_touched = True
            self.head_middle_touched = False
            self.head_back_touched = False
        elif msg.state == 1 and msg.button == 2:
            self.head_back_touched = False
            self.head_front_touched = False
            self.head_middle_touched = True
        elif msg.state == 1 and msg.button == 3:
            self.head_back_touched = True
            self.head_front_touched = False
            self.head_middle_touched = False
        
if __name__ == '__main__':

    rospy.init_node('nao_3_py')
    nc = NaoControl()
    while not rospy.is_shutdown():
        try: 
            nc.update()
        except KeyboardInterrupt:
            nc.app.goToPosture("Crouch", 0.5)
            myBroker.shutdown()
        pass

    #rospy.spin()