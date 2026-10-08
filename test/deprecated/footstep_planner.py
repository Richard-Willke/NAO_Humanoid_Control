# NOTE: USAGE DEPRECATED!!!!!!!!
import numpy as np
from enum import Enum
import tutorial_8.talos_conf as config
#import talos_conf as config

import pybullet as pb
from simulator.pybullet_wrapper import PybulletWrapper
#from pybullet_wrapper import PybulletWrapper

import scipy.interpolate as interp
import scipy.integrate as integrate

#from mapinfo import MapInfo
from tutorial_8.mapinfo import MapInfo
from copy import deepcopy
import math
from tutorial_8.Astar import AStar
from tutorial_8.ThetaStar import TStar
from scipy.optimize import curve_fit

class Side(Enum):
    """Side
    Describes which foot to use
    """
    LEFT=0
    RIGHT=1

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return self.value * other
        return NotImplemented

    def __rmul__(self, other):
        return self.__mul__(other)

def other_foot_id(id):
    if id == Side.LEFT:
        return Side.RIGHT
    else:
        return Side.LEFT

        
class FootStep:
    """FootStep
    Holds all information describing a single footstep
    """
    def __init__(self, pose, footprint, side=Side.LEFT):
        """inti FootStep

        Args:
            pose (pin.SE3): the pose of the footstep
            footprint (np.array): 3 by n matrix of foot vertices
            side (_type_, optional): Foot identifier. Defaults to Side.LEFT.
        """
        self.pose = pose
        self.footprint = footprint
        self.side = side
        
    def poseInWorld(self):
        return self.pose
        
    def plot(self, simulation):
    
        simulation.addGlobalDebugRectancle(self.pose.translation,Q=pin.Quaternion(self.pose.rotation).coeffs(), length=0.2, width=0.1, line_ids=None, color=[0, 0, 0], lineWidth=1, lifeTime=0)
        
        text = 'LEFT' if self.side == Side.LEFT else 'RIGHT'
        simulation.addUserDebugText(parent_id=-1, link_id=-1, text=text, x=self.pose.translation)
    
        simulation.addSphereMarker(position=self.pose.translation)
        
        return None

class FootStepPlanner:
    """FootStepPlanner
    Creates footstep plans (list of right and left steps)
    """
    
    def __init__(self, conf):
        self.conf = conf
        self.steps = []
        
    def planLine(self,steps, T_0_w, side, no_steps, theta):
        """plan a sequence of steps in a straight line

        Args:
            T_0_w (pin.SE3): The inital starting position of the plan
            side (Side): The intial foot for starting the plan
            no_steps (_type_): The number of steps to take

        Returns:
            list: sequence of steps
        """
        
        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x
        dy = 2*self.conf.step_size_y
        
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn

        footprint = np.array([[lfxp, lfxn, lfxn, lfxp],
                            [lfyp, lfyp, lfyn, lfyn],
                            [0, 0, 0, 0]])
        
        # Plan a sequence of steps with T_0_w being the first step pose.
        x0, y0 = T_0_w.pose.translation[0:2] 

        for i in range(no_steps):
            if i == 0:
                step = (x0, y0 - (side==Side.RIGHT)*dy )
            elif i == 1:
                step = (x0, y0- (side==Side.RIGHT)*dy )
            elif i == no_steps-1:
                step = (steps[-1].pose.translation[0], y0-(side==Side.RIGHT)*dy ) # same x as last step but diff y
            else:
                step = (x0 + (i-1)*dx, y0-(side==Side.RIGHT)*dy )

            # add z coordinate
            step = np.r_[step, 0.0]

            # create the footstep object
            step = FootStep(pin.SE3(np.eye(3), step), footprint, side)
            steps.append(step)

            side = other_foot_id(side)
        
        self.steps = steps
        return steps
    
    def planLineThetaStar(self,side, T_support):
        """plan a sequence of steps along path from theta star pathfinding algorithm

        Args:
            side (Side): The intial foot for starting the plan
            T_support (pin.SE3.pose): The inital support foot pose of the plan

        Returns:
            list: sequence of steps
        """

        ####### map for theta star #########
        m = MapInfo(100, 40)
        #m.show()
        
        obstacle = [(55,15+i) for i in range (6)] + [(55-i, 20) for i in range (6)] + [(50, 20-i) for i in range (6)] + [(55-i,15) for i in range (6)] + [(40,23+i) for i in range (6)] + [(40-i, 28) for i in range (6)] + [(35, 28-i) for i in range (6)] + [(40-i, 23) for i in range (6)]
        start =  (50, 5)
        end = (35, 35)
        
        m.start = start
        m.end = end
        m.obstacle = obstacle
        Tstar = TStar(m)

        m.path = Tstar.a_star_planning(m,display=False)
        #m.wait_close()
        
        pixel_path = m.path
        pixel_path.pop(2) # delete the unnecessary corner point
        path_in_metres = np.zeros((len(pixel_path), 2))

        for i in range(len(pixel_path)):
            # i unit in y direction is 0.2 m and i unit in x direction is 0.1 m
            # the xy and for the plot here are pybullet are not same 
            path_in_metres[i, :] = ( pixel_path[i][1] - pixel_path[0][1]) * 0.2,  (pixel_path[0][0] - pixel_path[i][0]) * 0.2 
        
        # linear distance between first two points = 4.6m, so like 22-24 steps and 0.8 second per step, so like 18 seconds for trajectroy
        Trajectory1_coeffs_x = Tstar.Polynomial_coeffs(path_in_metres[0,0], path_in_metres[1,0], 0, 18)
        Trajectory1_coeffs_y = Tstar.Polynomial_coeffs(path_in_metres[0,1], path_in_metres[1,1], 0, 18)
        
        # linear distance between second and third points = 2.3m, so like 12 steps and 0.8 second per step, so like 10 seconds roughhly for trajectroy
        Trajectory2_coeffs_x = Tstar.Polynomial_coeffs(path_in_metres[1,0], path_in_metres[2,0], 0, 10)
        Trajectory2_coeffs_y = Tstar.Polynomial_coeffs(path_in_metres[1,1], path_in_metres[2,1], 0, 10)
        
        X_1 = []
        Y_1 = []
        
        X_2 = []
        Y_2 = []
        
     ############# for the first trajectory (from initial point to corner point)  ###########
        for t in np.arange(0, 18, 0.5):
            T_xy_coefficients = np.array([[1,  t,  t**2,   t**3,    t**4,    t**5],
                                        [0,  1,  2*t,  3*t**2,  4*t**3,  5*t**4],
                                        [0,  0,  2,  6*t,  12*t**2,  20*t**3]])
        
            X_pos = T_xy_coefficients[0,:].dot(Trajectory1_coeffs_x)
            X_1.append(X_pos)
            Y_pos = T_xy_coefficients[0,:].dot(Trajectory1_coeffs_y)
            Y_1.append(Y_pos)

        pos_1 = np.zeros((len(X_1), 2))
        pos_1[:len(X_1), 0] = X_1[:]
        pos_1[:len(Y_1), 1] = Y_1[:]

        delta_x_1 = np.diff(pos_1[:, 0])
        delta_y_1 = np.diff(pos_1[:, 1])

        yaw_radians = np.arctan2(delta_x_1, delta_y_1) # Note make sure yaw is calculated correctly

        delta_X_1 = np.diff(path_in_metres[:2,0]) 
        delta_Y_1 = np.diff(path_in_metres[:2,1])

        Yaw_Rad_1 = np.arctan2(delta_Y_1, delta_X_1)
        no_steps_1 = 34 # estimated number of steps for first trajectory

        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x
        dy = 2*self.conf.step_size_y
        
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn

        footprint = np.array([[lfxp, lfxn, lfxn, lfxp],
                            [lfyp, lfyp, lfyn, lfyn],
                            [0, 0, 0, 0]])
        
        # Plan a sequence of steps with the initial support foot pose being the first step pose.
        x0, y0 = T_support.translation[0:2] 

        steps = []
        for i in range(no_steps_1 - 1):
            if i == 0:
                step = (x0, y0-(side==Side.RIGHT)*dy )
            elif i == 1:
                step = (x0, y0-(side==Side.RIGHT)*dy )
            elif i == 2:
                step = (x0, y0-(side==Side.RIGHT)*dy )
            elif i == 3:
                step = (x0, y0-(side==Side.RIGHT)*dy - dy/2)
            elif i == 4:
                step = (x0, y0-(side==Side.RIGHT)*dy - 0.5*dy )
            elif i == 5:
                step = (x0, y0-(side==Side.RIGHT)*dy - dy/2)
            elif i == 6:
                step = (x0, y0-(side==Side.RIGHT)*dy - 0.5*dy)   
            elif i == no_steps_1-1:
                step = (steps[-1].pose.translation[0], steps[-1].pose.translation[1] ) # same x as last step but diff y
            else:
                step = (pos_1[i,0], pos_1[i,1]-(side==Side.RIGHT)*dy)

            step = np.r_[step, 0.0] # add z coordinate

            # create the footstep object
            if i <=4: # first few steps for stabilizing at the same position without changing orientation
                step = FootStep(pin.SE3(np.eye(3), step), footprint, side)
            elif i>1 and i <= no_steps_1 -1: # after 2 steps change orientation to yaw of first trajectory
                step = FootStep(pin.SE3(self.get_Rz(Yaw_Rad_1[0]), step), footprint, side)
            steps.append(step)
            side = other_foot_id(side)

        #######################################################################################
        ############# for the second trajectory (from corner point to final point)  ###########
        
        for t in np.arange(0, 10, 0.5):
            T_xy_coefficients = np.array([[1,  t,  t**2,   t**3,    t**4,    t**5],
                                        [0,  1,  2*t,  3*t**2,  4*t**3,  5*t**4],
                                        [0,  0,  2,  6*t,  12*t**2,  20*t**3]])

            X_pos_2 = T_xy_coefficients[0,:].dot(Trajectory2_coeffs_x)
            X_2.append(X_pos_2)
            Y_pos_2 = T_xy_coefficients[0,:].dot(Trajectory2_coeffs_y)
            Y_2.append(Y_pos_2)

        pos_2 = np.zeros((len(X_2), 2))
        pos_2[:len(X_2), 0] = X_2[:]
        pos_2[:len(Y_2), 1] = Y_2[:]
        
        # differences in x and y
        delta_x_2 = np.diff(pos_2[:, 0])
        delta_y_2 = np.diff(pos_2[:, 1])
        yaw_radians = np.arctan2(delta_x_2, delta_y_2) # Note make sure yaw is calculated correctly
        
        delta_X_2 = np.diff(path_in_metres[1:,0]) 
        delta_Y_2 = np.diff(path_in_metres[1:,1])
        
        Yaw_Rad_2 = np.arctan2(delta_Y_2, delta_X_2)
        last_support = steps[-1]
        no_steps_2 = 20 # estimated number of steps for second trajectory

        steps = self.planLine(steps, last_support, side, no_steps_2, Yaw_Rad_2)

        # x0, y0 = last_support.pose.translation[0:2]  # LEFT

        # for i in range(no_steps_2-1):
        #     if i == 0:
        #         step = (x0, y0-(side==Side.RIGHT)*dy )
        #     elif i == 1:
        #         step = (x0, y0-(side==Side.RIGHT)*dy )
        #     elif i == 2:
        #         step = (x0, y0-(side==Side.RIGHT)*dy )
            

        #     elif i == no_steps_2-1:
        #         step = (steps[-1].pose.translation[0], steps[-1].pose.translation[1] ) # same x as last step but diff y
        #     else:
        #         step = (pos_2[i,0], pos_2[i,1]-(side==Side.RIGHT)*dy)

        #     #add z coordinate
        #     print(i ,step)
        #     step = np.r_[step, 0.0]

        #     if i <=2:
        #         step = FootStep(pin.SE3(last_support.pose.rotation, step), footprint, side)
        #     elif i>2 and i <= no_steps_2 -1:
        #         step = FootStep(pin.SE3(self.get_Rz(Yaw_Rad_2[0]), step), footprint, side)

        #     steps.append(step)
        #     side = other_foot_id(side)

        self.steps = steps
        return steps

    def planLineAlongPath(self, path, side, no_steps):
        """plan a sequence of steps in a strait line

        Args:
            path : The planned path to plan footstep
            side (Side): The intial foot for starting the plan
            no_steps (_type_): The number of steps to take

        Returns:
            list: sequence of steps
        """
        
        m = MapInfo(100, 60)
        #m.show()
        
        obstacle = [(55,15+i) for i in range (6)] + [(55-i, 20) for i in range (6)] + [(50, 20-i) for i in range (6)] + [(55-i,15) for i in range (6)] + [(40,23+i) for i in range (6)] + [(40-i, 28) for i in range (6)] + [(35, 28-i) for i in range (6)] + [(40-i, 23) for i in range (6)]
        start =  (50, 5)
        end = (35, 35)
        
        m.start = start
        m.end = end
        m.obstacle = obstacle
        
        plan = AStar(m.start, m.end, m)
        
        # Note display should always be False, otherwise we wont see the path, the code neeeds to be fixed
        if plan.run(display = False): 
            m.path = plan.reconstruct_path()

        #m.wait_close()
        pixel_path = m.path

        path_in_metres = np.zeros((len(pixel_path), 2))
        Px = np.zeros(len(pixel_path))
        Py = np.zeros(len(pixel_path))
    
        for i in range(len(pixel_path)):

            path_in_metres[i, :] = ( pixel_path[i][1] - pixel_path[0][1]) * 0.2,  (pixel_path[0][0] - pixel_path[i][0]) * 0.2
            Px[i] = path_in_metres[i, 0]
            Py[i] = path_in_metres[i, 1]
        
        
        T_0_w = pin.SE3(np.eye(3), np.zeros(3))
        
        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x
        dy = 2*self.conf.step_size_y
        
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn

        footprint = np.array([[lfxp, lfxn, lfxn, lfxp],
                            [lfyp, lfyp, lfyn, lfyn],
                            [0, 0, 0, 0]])

        def estimate_path_deriv(path, t, h=0.01):
            # attention! h actually is in time, normally it shoudl be in x thats why we cant derive thought t 
            x1, y1 = path(t)
            x2, y2 = path(t+h)
            deriv = (y2-y1)/(x2-x1)
            return deriv 

        def get_two_consec_points(path, t, h=0.2):
            x1, y1 = path(Px, Py, t)
            x2, y2 = path(Px, Py, t+h)
            delta_x = x2 - x1
            delta_y = y2 - y1
            return delta_x, delta_y 

        theta = 0.0

        steps = []
        for i in range(no_steps):
            if i == 0:
                t = 0.0
                x0, y0 = 0, 0
                delta_x, delta_y = get_two_consec_points(path, t)
                theta = np.arctan2(delta_y, delta_x)
                v_ortho = np.array([delta_y, -delta_x])
                v_ortho = v_ortho/np.linalg.norm(v_ortho)
                v_ortho = -v_ortho
                step = np.array([x0,y0]) + ((side==Side.LEFT)*dy -dy/2) * v_ortho
                step = (step[0], step[1])

            elif i == 1:
                t = 0.0
                x0, y0 = 0, 0 # still path of 0.0 because we want to place the second footstep right next to the first.
                delta_x, delta_y = get_two_consec_points(path, t)
                theta = np.arctan2(delta_y, delta_x)
                v_ortho = np.array([delta_y, -delta_x])
                v_ortho = v_ortho/np.linalg.norm(v_ortho)
                v_ortho = -v_ortho
                step = np.array([x0,y0]) + ((side==Side.LEFT)*dy -dy/2) * v_ortho
                step = (step[0], step[1])
                #print(f" newstep: {step}")
            elif i == no_steps-1:
                t = 1.0
                x0, y0 = path(Px, Py, t)
                delta_x, delta_y = get_two_consec_points(path, t)
                theta = np.arctan2(delta_y, delta_x)
                v_ortho = np.array([delta_y, -delta_x])
                v_ortho = v_ortho/np.linalg.norm(v_ortho)
                v_ortho = -v_ortho

                step = np.array([steps[-1].pose.translation[0],y0]) + ((side==Side.LEFT)*dy -dy/2) * v_ortho
                step = (step[0], step[1])

            else:
                t = (i-1)/(no_steps-3)
                x0, y0 = path(Px, Py, t)
                delta_x, delta_y = get_two_consec_points(path, t)
                theta = np.arctan2(delta_y, delta_x)
                v_ortho = np.array([delta_y, -delta_x])
                v_ortho = v_ortho/np.linalg.norm(v_ortho)
                v_ortho = -v_ortho

                step = np.array([x0,y0]) + ((side==Side.LEFT)*dy -dy/2) * v_ortho
                step = (step[0], step[1])

            # add z coordinate
            step = np.r_[step, 0.0]
            rot_z = pin.AngleAxis(theta, np.r_[0.0, 0.0, 1.0]).toRotationMatrix()

            # create the footstep object
            step = FootStep(pin.SE3(rot_z.dot(T_0_w.rotation), step), footprint, side)
            steps.append(step)

            side = other_foot_id(side)
        
        self.steps = steps
        return steps
    
    def get_Rz(self, theta):
        
        R_z = np.array([[np.cos(theta), -np.sin(theta), 0], 
                        [np.sin(theta), np.cos(theta),   0],
                        [0,                    0,       1]])
        return R_z
    
    
    def planStaticRotation(self, T_support_w, T_swing_w, side, circular_rotation_steps):
        # footstep planning with static rotation
        T_0_w = pin.SE3(np.eye(3), np.zeros(3))
        
        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x
        dy = 2*self.conf.step_size_y
        
        
        # Rotation of 30 degrees
        Rot = np.array([[np.sqrt(3) / 2,   -1/2,        0],
                         [1/2,           np.sqrt(3)/2,   0],
                         [0,                  0,         1]])
        
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn

        footprint = np.array([[lfxp, lfxn, lfxn, lfxp],
                            [lfyp, lfyp, lfyn, lfyn],
                            [0, 0, 0, 0]])
        theta = 0.0
        steps = []
    
        x0, y0 = T_support_w.translation[0:2]
        current_x, current_y = x0, y0 - 2*dy   
        theta = np.pi/12
        rot_value = np.pi/12
    
        for i in range(circular_rotation_steps):
            if i <= 1: 
                if i == 0:
                    step = (x0, y0+(side==Side.RIGHT)*dy )  
                    theta = 0.0     
                elif i == 1:
                    step = (x0, y0-(side==Side.RIGHT)*dy )  
                    theta = 0.0
            else:
                if i % 2 == 0:  
                    step = (x0 , y0)
                    theta = (i-1)*rot_value
                elif i % 2 == 1 : 
                    theta = theta + rot_value
                    current_x = current_x + dy * np.cos(theta)
                    current_y = current_y + dy * np.sin(theta)
                    step = (current_x, current_y) # same x as last step but diff y
            step = np.r_[step, 0.0]
            
            step = FootStep(pin.SE3(self.get_Rz(theta), step), footprint, side)
            steps.append(step)

            side = other_foot_id(side)
        self.steps = steps
        return steps
    
    def planpathAstar(self, path_in_metres, yaw_radians, side, noSteps):
        # footstep planning with path from A star algorithm
        T_0_w = pin.SE3(np.eye(3), np.zeros(3))
        
        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x
        dy = 2*self.conf.step_size_y
           
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn

        footprint = np.array([[lfxp, lfxn, lfxn, lfxp],
                            [lfyp, lfyp, lfyn, lfyn],
                            [0, 0, 0, 0]])

        steps = []
        x0, y0 = T_0_w.translation[0:2] 

        for i in range(noSteps):
            if i <= 1:
                if i == 0:
                    step = (x0, y0+(side==Side.RIGHT)*dy )
                    theta = 0.0
                    
                    step = np.r_[step, 0.0]
            
                    step = FootStep(pin.SE3(self.get_Rz(theta), step), footprint, side)
                    steps.append(step)

                    side = other_foot_id(side)
 
                elif i == 1:
                    step = (x0, y0+(side==Side.RIGHT)*dy)
                    theta = 0.0
                    step = np.r_[step, 0.0]

                    step = FootStep(pin.SE3(self.get_Rz(theta), step), footprint, side)
                    steps.append(step)

                    side = other_foot_id(side)

            elif i > 1 and i <= 3:
                deltax = np.sin(yaw_radians[i - 2]) * dy   
                stepr = ( path_in_metres[i - 1, 0] + deltax , path_in_metres[i - 1, 1]-(side==Side.RIGHT)*dy)
                theta = yaw_radians[i - 2]
                
                stepr = np.r_[stepr, 0.0]
        
                stepR = FootStep(pin.SE3(self.get_Rz(theta), stepr), footprint, side)
                steps.append(stepR)

                side = other_foot_id(side)
                
                deltaxn = np.sin(yaw_radians[i - 2]) * dy
                stepl = ( path_in_metres[i - 1, 0] , path_in_metres[i - 1, 1]+(side==Side.RIGHT)*dy) 
                theta = yaw_radians[i - 3]
                stepl = np.r_[stepl, 0.0]

                stepL = FootStep(pin.SE3(self.get_Rz(theta), stepl), footprint, side)
                steps.append(stepL)

                side = other_foot_id(side)

        self.steps = steps
        return steps

    def plot(self, simulation):
        for step in self.steps:
            step.plot(simulation)
      
if __name__=='__main__':
    """ Test footstep planner
    """

    planner = FootStepPlanner(config)
    side = Side.LEFT
    footsteps = planner.planLineThetaStar(side)

    simulator = PybulletWrapper(sim_rate=1000)
    planner.plot(simulator)
    while 1:
        simulator.step()
    
   
