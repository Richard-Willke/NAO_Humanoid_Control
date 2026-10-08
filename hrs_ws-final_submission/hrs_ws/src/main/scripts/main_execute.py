#!/usr/bin/env python2
import qi
import sys
import tf
import cv2 
import time
import rospy
import numpy as np
import localization.kalman_filter as kf
import planing.Astar as astar
from planing.Bezier import Curve

""" new camera library file """
import vision_libs.camera as cam
import interface_libs.interface as interface
import mapping.world_map_model as world

from utils.utils import quaternion_to_euler, to_rot, to_quat, rotation_matrix_to_euler, quaternion_rotation_matrix, rotation_matrix_to_xzy_quaternion
import config_wrap.config_wrapper as config_wrapper 
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
import argparse
from geometry_msgs.msg import PoseStamped, TransformStamped
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Float64MultiArray


""" Implementation of our project without any NAO voice commands """

class NaoRobot:

    def __init__(self, verbose = False):

        self.path_pub = rospy.Publisher("/nao_path_walker", Path, queue_size=10)
        self.broadcaster = tf.TransformBroadcaster() 
        self.tf_msg = TransformStamped()

        self.params = config_wrapper.NAO_Params()

        self.KF = kf.KalmanFilter()
        if verbose:
            self.interface_module.say("Location Tracking Ready")


        self.obstacle_subscriber = rospy.Subscriber("/nao_torso_obstacle", Float64MultiArray, self.obstacle_callback , queue_size=10)
        self.goal_subscriber = rospy.Subscriber("/nao_torso_goal", Float64MultiArray, self.goal_callback, queue_size=10)


        self.goal_publisher = rospy.Publisher("/goal_position", Marker, queue_size=10)
        self.obstacles_publisher = rospy.Publisher('/obstacles_positions', MarkerArray, queue_size=10)
        
        if verbose:
            self.interface_module.say("Camera Booted Up")
        
        self.map_grid_size = self.params.map_grid_size
        self.obstacle_dimensions = self.params.obstacle_dim
        self.map = world.MapInfo(width= self.params.map_size[0], height=self.params.map_size[1])


        if verbose:
            self.interface_module.say("Loaded Map")

        if verbose:
            self.interface_module.say("Ready")

        self.ls = tf.TransformListener()

        self.H_t_w = np.array([[1, 0, 0,  0],
                               [0, 1, 0,  0],
                               [0, 0, 1, 0.25],
                               [0, 0, 0,  0]])


        """ Variables Required for Rohan's Implementation """
        self.goal_detected = False
        self.new_obstacle_detected = False
        self.robot_pos = np.array([0.0, 0.0, 0.25])
        self.robot_rot = np.array([0, 0, 0, 1])
        self.robot_rot_matrix = np.eye(3)
        self.goal_pos = np.zeros(3)
        
        self.robot_discretised_pos = None
        self.discretised_goal = None
        self.NaoFootSteps = []
        self.NaoSide = []
        """ we save obstacles as [id, x,y,z, quatx, quaty , quatz, quatw] in marker_pos_world """
        self.marker_pos_world = {}
        self.goal_pos_world = {}
        self.time_for_one_step = 1.0



    def obstacle_callback(self, msg):
        
        """ A call back which listens the topic which publishes the position of detected Obstacles in
            torso frame of Nao and then converts them to world frame using a Tranfromation matrix made
            from NAO's position estimate using Kalman Filter """

        id = str(int(msg.data[0]))

        if id not in self.marker_pos_world.keys():

            transform_time = self.get_torso_to_odom_transform

            t_obj_torso = np.eye(4)
            t_obj_torso[:3,:3] = quaternion_rotation_matrix(msg.data[4:])
            t_obj_torso[:3, 3] = np.array(msg.data[1:4])

            """ we save as [id, x,y,z, quatx, quaty , quatz, quatw] """
            obj_pos_w = transform_time.dot(t_obj_torso)
            obj_quat_w = rotation_matrix_to_xzy_quaternion(obj_pos_w)
            

            self.marker_pos_world[id] = [int(id), obj_pos_w[0,3], obj_pos_w[1,3], obj_pos_w[2, 3], 
                                                obj_quat_w[0], obj_quat_w[1], obj_quat_w[2], obj_quat_w[3]]

            self.new_obstacle_detected = True


    def goal_callback(self, msg):


        """ A call back which listens the topic which publishes the position of detected Goal in
            torso frame of Nao and then converts them to world frame using a Tranfromation matrix made
            from NAO's position estimate using Kalman Filter """

        id = str(int(msg.data[0]))

        if id not in self.goal_pos_world.keys():

            transform_time = self.get_torso_to_odom_transform

            t_goal_torso = np.eye(4)
            t_goal_torso[:3,:3] = quaternion_rotation_matrix(msg.data[4:])
            t_goal_torso[:3, 3] = np.array(msg.data[1:4])

            """ we save as [id, x,y,z, quatx, quaty , quatz, quatw] """
            goal_pos_w = transform_time.dot(t_goal_torso)
            goal_quat_w = rotation_matrix_to_xzy_quaternion(goal_pos_w)
            
            self.goal_pos_world[id] = [int(id), goal_pos_w[0,3], goal_pos_w[1,3], goal_pos_w[2, 3], 
                                                goal_quat_w[0], goal_quat_w[1], goal_quat_w[2], goal_quat_w[3]]

            self.goal_detected = True
            self.discretised_goal = self.discretise_pos_to_map([goal_pos_w[0,3], goal_pos_w[1,3], goal_pos_w[2, 3]])

            

    def publish_path(self, princeple_steps):

        """ A function to publish computed path using Astar and Bezier curve """

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom" 

        for i, wp in enumerate(princeple_steps):

            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0 
            pose.pose.orientation.y = 0.0 
            pose.pose.orientation.z = 0.0 
            pose.pose.orientation.w = 1.0 
            path.poses.append(pose)

        self.path_pub.publish(path)
        print("Publishing Path message")



    def get_footsteps(self, time_for_one_step = 1.0):

        """ This function runs the Astar with obstacles in map,
            then creates a smooth path using output path of Astar using Bezier curve,
            then implemets FootStep planning,
            Finally returns the FootSteps required by Nao (each Footstep in frame of the other foot) """
        
        """ create discretised path using Astar """
        plan = astar.AStar(self.map.start, self.map.end, self.map)
        if plan.run(display = False): 
            self.map.path = plan.reconstruct_path()

        """ Convert discretised path to path in meters """
        pixel_path = self.map.path
        path_in_metres = np.zeros((len(pixel_path), 2))
        for i in range(len(pixel_path)):
            path_in_metres[i, :] = ( pixel_path[i][1] - pixel_path[0][1]) * self.params.map_grid_size ,  (pixel_path[0][0] - pixel_path[i][0]) * self.params.map_grid_size

        differences = np.diff(path_in_metres, axis=0)
        segment_lengths = np.linalg.norm(differences, axis=1)
        curve_length = np.sum(segment_lengths)
        time_for_one_footstep = self.time_for_one_step
        distance_one_footstep = self.params.map_grid_size
        total_number_of_footsteps = int(np.ceil(curve_length / distance_one_footstep))
        total_time = total_number_of_footsteps * time_for_one_footstep
        t_p = np.linspace(0, 1, total_number_of_footsteps)

        """ Bezier curve implementation is below """
        # The curve issue happens when there is no path ( as robot and objects are out of the map)
        princeple_steps = Curve(t_p, path_in_metres)
        self.publish_path(princeple_steps)

        t_p_unnormalized = t_p * total_number_of_footsteps
        delta_x = np.diff(princeple_steps[:, 0])
        delta_y = np.diff(princeple_steps[:, 1])
        yaw_radians = np.arctan2(delta_y, delta_x)
        Yaw_Path = np.r_[yaw_radians, yaw_radians[-1]]

        """ plan footsteps """
        NFS = astar.NaoFootStepPlanner()
        print('TotalNumber of FOOTSTEPS: ', total_number_of_footsteps)
        steps, side, steps_in_foot_frame, theta = NFS.planLineAlongPath(princeple_steps, total_number_of_footsteps)

        return  steps_in_foot_frame, side
    

    def discretise_pos_to_map(self, world_pos = None):

        """ This function converts real world positions to discretised map positions
            X world = Y map
            Y World = -X map """

        x_discretised = 30 - np.ceil(np.array([world_pos[1] /  self.params.map_grid_size])).item()
        y_discretised = np.ceil(np.array([world_pos[0] /  self.params.map_grid_size])).item()

        return [x_discretised, y_discretised]
    

    @property
    def get_torso_to_odom_transform(self):

        """ A fucntion to return current Transfrom from Nao torso/COM frame to World/odom frame """

        return self.H_t_w

    def step(self):

        """ This function updates the Nao's positon using Kalman Filter and
            also discretises it for the map """

        try:
            self.robot_pos, self.robot_rot = self.KF.update()
        except:
            pass

        self.H_t_w[:3,:3] = quaternion_rotation_matrix(self.robot_rot)
        self.H_t_w[:3, 3] = self.robot_pos

        """ Discretised robot position for replanning in map """
        self.robot_discretised_pos = self.discretise_pos_to_map(self.robot_pos)
        
        self.publish_all_markers()

    def create_marker_obstacle(self, publisher = None):

        """ A function to publish obstacles as markers """

        if len(self.marker_pos_world) != 0:

            marker_array = MarkerArray()
            time = rospy.Time.now()

            for key, val in self.marker_pos_world.items():
            
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = time
                marker.ns = "cube"
                marker.id = val[0]
                marker.type = Marker.CUBE 
                marker.action = Marker.ADD
                marker.pose.position.x = val[1]
                marker.pose.position.y = val[2]
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = val[4]
                marker.pose.orientation.y = val[5]
                marker.pose.orientation.z = val[6]
                marker.pose.orientation.w = val[7]
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0  
                marker.lifetime = rospy.Duration(2)
                marker_array.markers.append(marker)

            publisher.publish(marker_array)


    def create_marker_goal(self, publisher = None):

        """ A function to publish goal as markers """

        if len(self.goal_pos_world) != 0:

            marker = Marker()
            time = rospy.Time.now()
            marker.header.frame_id = "odom"
            marker.header.stamp = time
            marker.ns = "cube"
            marker.id = self.goal_pos_world['0'][0]
            marker.type = Marker.CUBE 
            marker.action = Marker.ADD
            marker.pose.position.x = self.goal_pos_world['0'][1]
            marker.pose.position.y = self.goal_pos_world['0'][2]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = self.goal_pos_world['0'][4]
            marker.pose.orientation.y = self.goal_pos_world['0'][5]
            marker.pose.orientation.z = self.goal_pos_world['0'][6]
            marker.pose.orientation.w = self.goal_pos_world['0'][7]
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  
            marker.lifetime = rospy.Duration(0)
            publisher.publish(marker)


    def publish_all_markers(self):

        """ Publish all markers """

        self.create_marker_goal(publisher =self.goal_publisher)
  
        self.create_marker_obstacle(publisher =self.obstacles_publisher)

        
    def update(self):
    
        """ This function computes the Foot step plan when goal is detected or obstacle is detected,
            that is this function does the replanning when obstacle/goal is detected"""

        self.map.start = (self.robot_discretised_pos[0], self.robot_discretised_pos[1])
        self.map.end = (self.discretised_goal[0], self.discretised_goal[1])
        obstacle_list = []

        if self.new_obstacle_detected == True:

            str_id = list(self.marker_pos_world.keys())[-1]

            desired_val = self.marker_pos_world[str_id]

            discretised_obstacle = self.discretise_pos_to_map([desired_val[1], desired_val[2], desired_val[3]])
            rpy = quaternion_to_euler(desired_val[4:])
    
            obstacle_list = self.map.discretize(np.array(discretised_obstacle), rpy[2])

            
        self.map.obstacle += obstacle_list
        
        self.NaoFootSteps, self.NaoSide = self.get_footsteps()

        self.new_obstacle_detected = False

        return self.NaoFootSteps, self.NaoSide 


""" Main function """

def main(session):

    nao = NaoRobot()
    params = config_wrapper.NAO_Params()

    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")

    # Wake up robot
    motion_service.wakeUp()

    # Send robot to Pose Init
    posture_service.goToPosture("StandInit", 0.5)

    Done = False
    i = 0
    time_for_one_step = 1.0
    previous_step_info = None

    time.sleep(5)

    """ Main loop """

    while not rospy.is_shutdown():
        
        nao.step()

        """ if goal not detected do not do anything """

        if nao.goal_detected:

            """ recompute the Path to goal """
            
            nao.step()
            nao.NaoFootSteps, nao.NaoSide = nao.update()
            print(nao.robot_discretised_pos, nao.discretised_goal)
            print('Number of Steps to take are:',len(nao.NaoFootSteps))

            while (not nao.new_obstacle_detected) and (not Done):
                
                legName = [nao.NaoSide[i]]
                X = nao.NaoFootSteps[i][0].item()
                Y = nao.NaoFootSteps[i][1].item()
                Theta = nao.NaoFootSteps[i][2].item() 
                footSteps = [[X, Y, Theta]]
                timeList = [time_for_one_step]
                clearExisting = False

                previous_step_info = [nao.NaoFootSteps[i][0], nao.NaoFootSteps[i][1], nao.NaoFootSteps[i][2], nao.NaoSide[i]]

                motion_service.setFootSteps(legName, footSteps, timeList, clearExisting)
                motion_service.waitUntilMoveIsFinished()
                

                nao.step() 

                print("Taking Step Number", i, Done, nao.new_obstacle_detected)

                i+=1
                if i == (len(nao.NaoFootSteps)):
                    Done = True
                    i = 0
            
            """ End the main function if all footsteps to goal are taken (goal reached) """

            if Done == True:
                break
                    
            print("New obstacle detected: ",nao.new_obstacle_detected)
            print("Completed footsteps plan: ", Done)
            i = 0
            Done = False
            nao.NaoFootSteps = []       
            nao.NaoSide = []

            """ Logic to take one extra FootStep, when stopped in between (obstacle detected) """
            if (Done == False) and (nao.new_obstacle_detected):

                if previous_step_info[-1] == "RLeg":

                    step = [0.0, -0.12, 0.0]
                    motion_service.setFootSteps(["LLeg"], [step], [1.0], False)
                    
                    motion_service.waitUntilMoveIsFinished()
                    motion_service.killMove()
                    
                    
                elif previous_step_info[-1] == "LLeg":
                    
                    step = [0.0, 0.12, 0.0]
                    motion_service.setFootSteps(["RLeg"], [step],  [1.0], False)

                    motion_service.waitUntilMoveIsFinished()
                    motion_service.killMove()
                    


                

if __name__ == "__main__":

    rospy.init_node("Let_Walk")
    rate = rospy.Rate(20)
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="10.152.246.156",
                        help="Robot IP address. On robot or Local Naoqi: use '10.152.246.156'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    main(session)