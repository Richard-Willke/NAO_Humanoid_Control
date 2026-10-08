#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use setFootSteps Method"""

import qi
import argparse
import sys
import time
import rospy
import numpy as np
sys.path.append('/workspaces/hrs/test/path_planning')
sys.path.append('/workspaces/hrs/test/kalman_filter/kalman_filter')
from Astar import *
from kalman_filter import KalmanFilter
from world_map_model import *

def get_footsteps(time=0.6):
    # GET MAP
    m = MapInfo()
    m.show()

    # SET START AND END POINT
    # Test 1
    start =  (30, 0)
    end = (30, 10)

    # Test 2
    # start =  (30, 0)
    # end = (45, 35)

    # Test 3 
    #start =  (30, 0)
    #end = (20, 30)

    # SET OBSTACLES AS EMPTY LIST
    m.start = start
    m.end = end
    m.obstacle =  []

    # CALCULATE PATH
    plan = AStar(m.start, m.end, m)
    if plan.run(display = True): 
        m.path = plan.reconstruct_path()
    m.wait_close()

    # FIT CURVE
    pixel_path = m.path

    path_in_metres = np.zeros((len(pixel_path), 2))
    for i in range(len(pixel_path)):
        path_in_metres[i, :] = ( pixel_path[i][1] - pixel_path[0][1]) * 0.05,  (pixel_path[0][0] - pixel_path[i][0]) * 0.05

    differences = np.diff(path_in_metres, axis=0)
    segment_lengths = np.linalg.norm(differences, axis=1)
    curve_length = np.sum(segment_lengths)

    time_for_one_footstep = time
    distance_one_footstep = 0.05
    total_number_of_footsteps = int(np.ceil(curve_length / distance_one_footstep))

    total_time = total_number_of_footsteps * time_for_one_footstep

    t_p = np.linspace(0, 1, total_number_of_footsteps)
    princeple_steps = Curve(t_p, path_in_metres)
    
    # PLAN FOOTSTEPS
    t_p_unnormalized = t_p * total_number_of_footsteps

    delta_x = np.diff(princeple_steps[:, 0])
    delta_y = np.diff(princeple_steps[:, 1])

    yaw_radians = np.arctan2(delta_y, delta_x)
    Yaw_Path = np.r_[yaw_radians, yaw_radians[-1]]
    
    NFS = NaoFootStepPlanner()

    steps, side, steps_in_foot_frame, theta = np.array(NFS.planLineAlongPath(princeple_steps,total_number_of_footsteps))

    return  steps_in_foot_frame, side

def main(session):
    """
    This example uses the setFootSteps method.
    """
    # Get the services ALMotion & ALRobotPosture.
    KF = KalmanFilter()
    steps_in_foot_frame, side = get_footsteps()
    Done = False
    i = 0

    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")

    # Wake up robot
    motion_service.wakeUp()

    # Send robot to Pose Init
    posture_service.goToPosture("StandInit", 0.5)

    while not Done:

        try:

            robot_pos, robot_rot = KF.update()

        except:

            # why not just use pass, so we have previous robot position saved    
            robot_pos, robot_rot = (np.zeros(3), np.array([0,0,0]))

        # A small step forwards and anti-clockwise with the left foot
        legName = [side[i]]
        if i % 2 == 0:
            legName = ["RLeg"]
        else:
           legName = ["LLeg"]

        X = steps_in_foot_frame[i][0].item()#np.clip(steps_in_foot_frame[i][0].item(), 0.0, 0.06).item()
        Y = steps_in_foot_frame[i][1].item()#np.clip(steps_in_foot_frame[i][1].item(), -0.16, 0.16).item()
        Theta = steps_in_foot_frame[i][2].item() #np.clip(steps_in_foot_frame[i][2].item(), -0.21, 0.21).item()
        footSteps = [[X, Y, Theta]]
        timeList = [0.6]
        clearExisting = False
        motion_service.setFootSteps(legName, footSteps, timeList, clearExisting)
        time.sleep(1.0)


        i +=1

        if i == (len(steps_in_foot_frame)-1):
            Done == True

    motion_service.waitUntilMoveIsFinished()



    #A small step forwards and anti-clockwise with the left foot
    # print(footstep)
    # X = 0.8#np.round(footstep[i][0],2).item()
    # Y = -np.round(footstep[i][1],2).item()
    # Theta = 0.0#np.round(footstep[i][2], 2).item()
    # legName = ["RLeg"]
    # footSteps = [[X, Y, Theta]]
    # timeList = [0.6]
    # clearExisting = False
    # print(footSteps)


    # legName = ["LLeg", "RLeg"]
    # X       = 0.04ss
    # Y       = 0.1
    # Theta   = -0.3
    # footSteps = [[X, Y, Theta], [X, -Y, Theta]]
    # timeList = [0.6, 1.2]
    # clearExisting = False
    #motion_service.setFootSteps(legName, footSteps, timeList, clearExisting)



    #A small step forwards and anti-clockwise with the left foot
    # legName  = ["RLeg"]
    # X        = 0.06
    # Y        = -0.12
    # Theta    = 0.0 #footstep[i][2]
    # footSteps = [[X, Y, Theta]]
    # timeList = [0.6]
    # clearExisting = False
    # motion_service.setFootSteps(legName, footSteps, timeList, clearExisting)

    time.sleep(1.0)

    motion_service.waitUntilMoveIsFinished()

    # Go to rest position
    motion_service.rest()


if __name__ == "__main__":
    rospy.init_node("nao_test")
    rate = rospy.Rate(5)
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