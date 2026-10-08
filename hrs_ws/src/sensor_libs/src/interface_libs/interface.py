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
from config_wrap.config_wrapper import *

memory = None

class NaoInterface:
    def __init__(self, mode = 1):
        self.nao_params = NAO_Params()

        self.head_front_touched = False
        self.head_middle_touched = False
        self.head_back_touched = False

        self.tactile_sub = rospy.Subscriber("/tactile_touch", HeadTouch, self.tactileCallback) 
        
        self.mode = mode
        self.asr = ALProxy("ALSpeechRecognition", self.nao_params.robotIP, self.nao_params.PORT)
        self.atts = ALProxy("ALTextToSpeech", self.nao_params.robotIP, self.nao_params.PORT)
        self.am = ALProxy("ALMotion", self.nao_params.robotIP, self.nao_params.PORT)
        self.app = ALProxy("ALRobotPosture", self.nao_params.robotIP, self.nao_params.PORT)
        #self.app.goToPosture("StandInit", 0.5)
        
        global memory
        memory = ALProxy("ALMemory")
        self.wtf = []
        
        self.num_steps = 6
        self.theta = 0

        self.atts.say("Hello, I'm NAO. Ready to serve.")

        self.asr.setLanguage("English") 
        try:
            self.asr.setVocabulary(self.nao_params.vocabulary, True)
        except:
            self.asr.unsubscribe("Test_ASR")
            self.asr.setVocabulary(self.nao_params.vocabulary, True)
        self.asr.subscribe("Test_ASR")

        self.ready = False
        self.standingUp = False

    def update(self):
        if self.head_front_touched:
            self.speechRecognition()
        elif self.head_middle_touched:
            self.fallRecovery()

        if len(self.wtf) >= 1 and (self.wtf[0] == "hello " or self.wtf[0] == "hi "):
            self.executeCommands()
        elif len(self.wtf) >= 1 and not (self.wtf[0] == "hello " or self.wtf[0] == "hi "):
            self.wtf = []

    def fallRecovery(self):
        self.atts.say("Trying to stand up.")
        self.app.goToPosture("Sit", 0.7)
        self.app.goToPosture("Crouch", 0.7)
        self.app.goToPosture("StandInit", 0.7)
        self.head_middle_touched = False
        self.atts.say("Ready to walk again.")

    def speechRecognition(self):
        self.asr.subscribe("Test_ASR")
        time.sleep(5)
        heard = memory.getData("WordRecognized")[0][6: -5]
        self.asr.unsubscribe("Test_ASR")
        if heard != '':
            self.wtf.append(heard)
        
        print(self.wtf)

        

    def pickAction(self):
        print("AAA: ", self.wtf[-1])
        if self.wtf[-1] == "go ":
            self.ready = True

            self.head_front_touched = False
            self.head_middle_touched = False
            self.head_back_touched = False
            self.atts.say("Alright, ready to go.")
            print("Alright, ready to go.")

        else:
            self.atts.say("Sorry, I did not catch that command")

    def executeCommands(self):
        print("yes? how, can I help?")
        self.atts.say("yes? how, can I help?")
        time.sleep(1)
        self.wtf = []
        while len(self.wtf) < 1:
            self.speechRecognition()
            print("AAAAAAAAAAAAAAAAA:", len(self.wtf))
            try:
                if self.wtf[-1] == "abort ":
                    self.wtf.append(["abort "])
                    self.head_front_touched = False # TODO: maybe have an exit function
                    self.atts.say("Aborting command.")
                    break
            except: 
                pass

        self.pickAction()
        self.wtf = []

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

    def say(self, string):
        self.atts.say(string)

if __name__ == '__main__':

    rospy.init_node('nao_3_py')
    params = NAO_Params()
    broker = params.NaoBroker

    nc = NaoInterface()
    while not rospy.is_shutdown():
        try: 
            nc.update()
        except KeyboardInterrupt:
            nc.app.goToPosture("Crouch", 0.5)
            broker.shutdown()
        pass