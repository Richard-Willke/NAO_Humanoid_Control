#!/usr/bin/env python2

from mapping.world_map_model import MapInfo
from copy import deepcopy
import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, writers
import scipy.interpolate as interp
import scipy.integrate as integrate
from Bezier import Curve
from enum import Enum

class AStar(object):

    """ A class to implement Astar algorithm """

    def __init__(self, start, end, map_info):
        self._s = start
        self._e = end
        self._map_info = map_info
        self._openset = dict()
        self._closeset = dict()

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) 

    def neighbor_nodes(self, x):
        plist = [(x[0] - 1, x[1] - 1), (x[0] - 1, x[1]), (x[0] - 1, x[1] + 1), (x[0], x[1] + 1), (x[0] + 1, x[1] + 1), (x[0] + 1, x[1]), (x[0] + 1, x[1] - 1), (x[0], x[1] - 1)]

        for p in plist:
            if not self._map_info.is_collision(point=p):
                yield p

    def reconstruct_path(self):
        pt = self._e
        path = []

        while pt:
            path.append(pt)
            pt = self._closeset[pt]['camefrom']
        return path[::-1]

    def run(self, display=False):
        h = self.distance(self._s, self._e)
        self._openset[self._s] = {'g': 0, 'h': h, 'f': h, 'camefrom': None}

        while self._openset:
            x = min(self._openset, key=lambda key: self._openset[key]['f'])
            self._closeset[x] = deepcopy(self._openset[x])
            del self._openset[x]

            if self.distance(x, self._e) < 1.0:
                if x != self._e:
                    self._closeset[self._e] = {'camefrom': x}

                return True

            if display:
                self._map_info.close = x

            for y in self.neighbor_nodes(x):
                if y in self._closeset:
                    continue

                tentative_g_score = self._closeset[x]['g'] + self.distance(x, y)
                if y not in self._openset:
                    tentative_is_better = True
                elif tentative_g_score < self._openset[y]['g']:
                    tentative_is_better = True
                else:
                    tentative_is_better = False
                if tentative_is_better:
                    h = self.distance(y, self._e)
                    self._openset[y] = {'g': tentative_g_score, 'h': h, 'f': tentative_g_score + h, 'camefrom': x}

                    if display:
                        self._map_info.open = y
        return False



def Polynomial_coeffs(ipos, fpos, ti, tf):
    ipos = ipos
    fpos = fpos
    
    iv= 0
    fv = 0 
    
    ia = 0 
    fa =0

    x = [ipos, fpos, iv, fv, ia, fa]
    a = np.zeros((6,1))

    T = np.array([[1,  ti,  ti**2,  ti**3, ti**4,    ti**5],
                  [1, tf,  tf**2,  tf**3,  tf**4,    tf**5],
                  [0,  1,  2*ti,  3*ti**2, 4*ti**3,  5*ti**4],
                  [0,  1,  2*tf,  3*tf**2, 4*tf**3,  5*tf**4],
                  [0,  0,   2,     6*ti,  12*ti**2, 20*ti**3],
                  [0,  0,   2,     6*tf,  12*tf**2, 20*tf**3]])

    if np.linalg.det(T) !=0:
        a = np.linalg.inv(T).dot(x)

    else:    
        a = np.linalg.pinv(T).dot(x) 
    
    return a

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

class FootStep:
    """FootStep
    Holds information describing the position single footstep
    """
    def __init__(self, pos = None, side = Side.RIGHT):
        """inti FootStep

        Args:
            pos (np.array of size 3 (x, y and yaw wrt other foot)): the pos of the footstep
            side (_type_, optional): Foot identifier. Defaults to Side.LEFT.
        """
        self.pos = pos
        self.side = side
        
    def posInWorld(self):
        return self.pos

def other_foot_id(id):
    if id == Side.LEFT:
        return Side.RIGHT
    else:
        return Side.LEFT
    


class NaoFootStepPlanner:

    """FootStepPlanner: Creates footstep plans (list of right and left steps) """
    
    def __init__(self):
        self.steps = []

    def compute_relative_transform(self, prev, curr, side):

        if side == "RLeg":

            y_rel = np.clip(np.array([curr[1] - prev[1] + 0.4]), -0.12, 0.01).item()
            x_rel = np.clip(np.array([curr[0] - prev[0]]), 0.0, 0.05).item()
            yaw_rel = prev[2] - curr[2]

            return [x_rel, y_rel, yaw_rel]

        else:
            y_rel = np.clip(np.array([curr[1] - prev[1]]), -0.9, 0.1).item()
            x_rel = np.clip(np.array([curr[0] - prev[0]]), 0.0, 0.06).item()
            yaw_rel = prev[2] - curr[2]

            return [x_rel, y_rel, yaw_rel]


    def planLineAlongPath(self, princeple_steps, num_steps):
        """
        This function returns steps: Foot steps in world frame,
                              side: Which Foot is used RLeg or LLeg, 
                              steps_in_foot_frame: The real steps used by NAO, i.e each foot step position with respect to other foot, 
                              theta: The real theta used by NAO, i.e each foot step yaw orientation with respect to other foot . """

        delta = np.diff(princeple_steps, axis=0)
        deviation = 0.05
        
        theta = []
        for i in range(num_steps-1):
            np.float32(theta.append(np.arctan2(delta[i, 0], delta[i, 1])))
        theta.append(theta[-1])

        side = []
        steps = [] 
        for i in range(1, num_steps):
            x_new = np.float32(princeple_steps[i, 0] + deviation * (-1)**(i + 1) * np.cos(theta[i]))
            y_new = np.float32(princeple_steps[i, 1] + deviation * (-1)**(i) * np.sin(theta[i]))
            step = [x_new, y_new, np.float32(theta[i])]

            if i % 2 == 0:
                side.append("LLeg")
            else:
                side.append("RLeg")

            steps.append(step)


        initial_left_foot = [0.0, 0.06, 1.57]  

        initial_right_foot = [0.0, -0.06, 1.57]

        # Compute all relative steps
        steps_in_foot_frame = []

        prev_foot = initial_left_foot  # First footstep is  right foot


        for i, step in enumerate(steps):

            relative_step = self.compute_relative_transform(prev_foot, step, side[i])

            steps_in_foot_frame.append(relative_step)

            prev_foot = step  # Update the previous foot


        steps.append([0.0, 0.0, 0.0])   # Add a placeholder for the last footstep

        """ Note by Rohan: This below method should also be implemented when a robot is stopped in between a path as a new Obstacle is detected
            and we break outside of the while loop. Just after the while loop save this below method to align both foot. """

        if side[-1] == "LLeg":

            side.append("RLeg")
            steps_in_foot_frame.append(np.array([0.0, -0.12, 0.0])) 
            
        else:

            side.append("LLeg")
            steps_in_foot_frame.append(np.array([0.0, 0.12, 0.0])) 

        theta.append(np.array([0.0]))   # Add placeholder for last footstep

        steps_in_foot_frame = np.array(steps_in_foot_frame)

        return steps, side, steps_in_foot_frame, theta


def plot_foot_steps(foot_steps, princeple_steps, ax):

    """
    Args:
        foot_steps (_type_): the foot step plan
        XY_foot_print (_type_): the dimensions of the foot (x,y)
        ax (_type_): the axis to plot on
    """

    feet_legth, feet_width = 0.05, 0.02
        
    ax.plot([princeple_steps[i,0] for i in range(len(foot_steps))], [princeple_steps[i,1] for i in range(len(foot_steps))], 'x', color = 'purple', label = 'Foot steps')
    
            
    for i in range(len(foot_steps)):
        
        x_pos = foot_steps[i][0]
        y_pos = foot_steps[i][1]
        
        if i % 2 == 0:
            
            shade = 'green'
            
        elif i % 2 == 1:
            
            shade = 'red'
            
        ax.fill([x_pos - feet_legth / 2, x_pos + feet_legth / 2, x_pos + feet_legth / 2, x_pos - feet_legth / 2], 
                [y_pos - feet_width/2, y_pos - feet_width/2, y_pos + feet_width/2, y_pos + feet_width/2], color=shade)

    plt.show()


if __name__ == "__main__":
    m = MapInfo(width=60, height=60)
    m.show()

    start =  (30, 3)
    end = (30, 7)
    m.start = start
    m.end = end
    plan = AStar(m.start, m.end, m)
    if plan.run(display = True): 
        m.path = plan.reconstruct_path()
    m.wait_close()

    #BEZIER CURVE FITTING
    pixel_path = m.path

    path_in_metres = np.zeros((len(pixel_path), 2))
    for i in range(len(pixel_path)):
        path_in_metres[i, :] = ( pixel_path[i][1] - pixel_path[0][1]) * 0.05,  (pixel_path[0][0] - pixel_path[i][0]) * 0.05

    differences = np.diff(path_in_metres, axis=0)
    segment_lengths = np.linalg.norm(differences, axis=1)
    curve_length = np.sum(segment_lengths)

    time_for_one_footstep = 0.6
    distance_one_footstep = 0.06
    total_number_of_footsteps = int(np.ceil(curve_length / distance_one_footstep))

    total_time = total_number_of_footsteps * time_for_one_footstep

    t_p = np.linspace(0, 1, total_number_of_footsteps)
    princeple_steps = Curve(t_p, path_in_metres)
    t_p_unnormalized = t_p * total_number_of_footsteps

    delta_x = np.diff(princeple_steps[:, 0])
    delta_y = np.diff(princeple_steps[:, 1])

    yaw_radians = np.arctan2(delta_y, delta_x)
    Yaw_Path = np.r_[yaw_radians, yaw_radians[-1]]

    NFS = NaoFootStepPlanner()
    steps, side, steps_in_foot_frame, theta = np.array(NFS.planLineAlongPath(princeple_steps, total_number_of_footsteps))
   

    st = np.zeros((len(steps), 2))

    for i in range(len(steps)):
        st[i, :] = steps[i][:2]

    plt.figure()
    plt.plot(
    	princeple_steps[:, 0],   # x-coordinates.
    	princeple_steps[:, 1],    # y-coordinates.
        'x'
    )
    plt.plot(
    	princeple_steps[:, 0],   # x-coordinates.
    	princeple_steps[:, 1]    # y-coordinates.
    )
    plt.plot(
    	path_in_metres[:, 0],  # x-coordinates.
    	path_in_metres[:, 1],  # y-coordinates.
    	'ro:'           # Styling (red, circles, dotted).
    )
    plt.figure()
    plt.plot(
    	st[:, 0],   # x-coordinates.
    	st[:, 1],    # y-coordinates.
        'x'
    )
    plt.grid()
    plt.show()

    fig1, ax1 = plt.subplots(1,1)
    plot_foot_steps(steps, princeple_steps, ax1)
    plt.show()