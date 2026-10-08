import os
import numpy as np
import sys
import rospkg
from naoqi import ALBroker


""" Configurations for our project """

class NAO_Params:
    def __init__(self):
        ################################################################################
        # general
        ################################################################################

        self.pi = 3.14159

        ################################################################################
        # robot
        ################################################################################

        self.rospack = rospkg.RosPack()
        #self.nao_description = rospack.get_path('nao_description') #### change accordingly 
        #self.path = os.path.join(self.nao_description, "meshes/../..")
        self.robotIP = "10.152.246.156" # str(sys.argv[1])
        self.PORT = 9559 # int(sys.argv[2])

        self.joint_names = ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
                    'LHand', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch',
                    'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RShoulderPitch', 'RShoulderRoll',
                    'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand']


        ################################################################################
        # camera params
        ################################################################################

        self.color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
                        'white': [[180, 18, 255], [0, 0, 231]],
                        'red1': [[180, 255, 255], [159, 50, 70]],
                        'red2': [[9, 255, 255], [0, 50, 70]],
                        'green': [[89, 255, 255], [36, 50, 70]],
                        'blue': [[128, 255, 255], [90, 50, 70]],
                        'yellow': [[35, 255, 255], [25, 50, 70]],
                        'purple': [[158, 255, 255], [129, 50, 70]],
                        'orange': [[24, 255, 255], [10, 50, 70]],
                        'gray': [[180, 18, 230], [0, 0, 40]]}

        self.bottom_image_dim = np.zeros((320,240))
        self.top_image_dim = np.zeros((640,480))
        self.B = np.zeros(4)
        self.hist_value = None

        """ 
        NOTE from Yueyang: 
            - Is this being used for both camera? -- Affermitive 
            - Where does it come from?            -- Moodle
        NOTE from Yueyang:
            The data is for top camera at resolution 640x480,
            but both cameras are running on 320x240

        self.camera_matrix = np.array([551.543059, 0.0, 327.382898, 0.0, 553.736023, 225.02638, 0.0, 0.0, 1]).reshape(3, 3)
        self.dist_coeffs = np.array([-0.066494, 0.095481, -0.000279, 0.002292, 0.0]).reshape(1, 5)
        """

        self.camera_matrix = np.array([278.236008818534, 0.0, 156.194471689706, 0.0, 279.380102992049, 126.007123836447, 0.0, 0.0, 1]).reshape(3, 3)
        self.dist_coeffs = np.array([-0.0481869853715082,  0.0201858398559121, 0.0030362056699177, -0.00172241952442813, 0]).reshape(1, 5)

        self.lower_bound = 50 
        self.upper_bound = 255

        self.max_corners = 100 
        self.quality_level = 0.01 
        self.min_distance = 10

        self.valid_ids = [12, 16, 24] #TODO: adjust accordingly to the ones we will use in the final 
        self.goal_id = [2]

        ################################################################################
        # speech params
        ################################################################################

        self.vocabulary = ["hello", "hi", "yes", "no", "please", "abort", "go"] #["hello", "hi", "robot", "yes", "no", "please", "stand", "up", "turn", "left", "right", "abort"]

        ################################################################################
        # map model 
        ################################################################################

        self.map_size = [60, 60]
        self.map_grid_size = 0.05
        self.obstacle_dim = np.array([5, 5])
        self.obstacle_size = [0.1, 0.1, 0.1]
        self.obstacle_color = [1.0, 0.0, 0.0, 1.0]

        