#!/usr/bin/env python
import matplotlib.pyplot as plt
import math
import copy
import time
from scipy.spatial import cKDTree
import numpy as np

from matplotlib.animation import FuncAnimation, writers

import config_wrap.config_wrapper as config



""" A class for a discretised world model """

class MapInfo(object):

    def __init__(self, width=150, height=150):

        self.width = width
        self.height = height
        self._okdtree = None

        self._start = (-1, -1)
        self._end = (-1, -1)
        self._obstacle = []
        self._open = (-1, -1)
        self._close = (-1, -1)
        self._path = []
        self._roadmap = dict()
        self._update_i = 0
        plt.figure()

        self.nao_params = config.NAO_Params()

    @property
    def start(self):
        return self._start

    @start.setter
    def start(self, s):
        self._start = s
        self.draw_point(self._start, 'o', color='green')

    def draw_point(self, p, shape, color):
        plt.plot(p[0], p[1], shape, color=color)
        if len(p) == 3:
            arrow_length = self.width / 20.0
            plt.arrow(p[0], p[1], arrow_length * math.cos(p[2]), arrow_length * math.sin(p[2]), head_width=arrow_length/5)

    @property
    def end(self):
        return self._end

    @end.setter
    def end(self, e):
        self._end = e
        self.draw_point(self._end, 'o', color='red')

    @property
    def obstacle(self):
        return self._obstacle

    @obstacle.setter
    def obstacle(self, o):
        """ Updated By Rohan """
        self._obstacle = copy.deepcopy(o)
        
        if not o: 
            self._okdtree = cKDTree(tuple(np.array([[0, 0]])))
        else:
            self._okdtree = cKDTree(o)

        for i in range(len(self._obstacle)):
            plt.plot(self._obstacle[i][0], self._obstacle[i][1], 's', color='black')


    def is_collision(self, **kwargs):
        if 'path' in kwargs:
            px, py = kwargs['path']
            for p in zip(px, py):
                d, _ = self._okdtree.query(p)
                if d <= 1.0 or p[0] < 0 or p[0] > self.width or p[1] < 0 or p[1] > self.height:
                    return True
            return False
        if 'car_outline' in kwargs:
            px, py = kwargs['car_outline']
            for p in zip(px, py):
                d, _ = self._okdtree.query(p)
                if d <= 1.0 or p[0] < 0 or p[0] > self.width or p[1] < 0 or p[1] > self.height:
                    return True
            return False
        if 'point' in kwargs:
            p = kwargs['point']
            d, _ = self._okdtree.query(p)
            if d <= 1.0 or p[0] < 0 or p[0] > self.width or p[1] < 0 or p[1] > self.height:
                return True
            return False

    @property
    def roadmap(self):
        return self._roadmap

    @roadmap.setter
    def roadmap(self, o):
        self._roadmap = copy.deepcopy(o)
        t = zip(*self.roadmap.keys())
        plt.plot(t[0], t[1], '.', color='blue')
        self.update()
        for k, v in self.roadmap.items():
            for p in v:
                plt.plot([k[0], p[0]], [k[1], p[1]], color='lightblue')
        plt.plot(t[0], t[1], '.', color='blue')
        self.update()

    def set_rand(self, r):
        self.draw_point(r, 'd', color='blue')
        self.update()

    def set_rrt(self, rrt):
        plt.clf()
        plt.plot(self._border_x, self._border_y, 'black')
        plt.plot(self.start[0], self.start[1], 'o', color='green')
        plt.plot(self.end[0], self.end[1], 'o', color='red')
        t = zip(*self.obstacle)
        plt.plot(t[0], t[1], 's', color='black')
        for r in rrt.items():
            t = zip(*r)
            plt.plot(t[0], t[1], color='lightblue')
        if self._update_i % 20 == 1:
            self.update()

    def set_rrt_dubins(self, rrt):
        plt.clf()
        plt.plot(self._border_x, self._border_y, 'black')
        self.draw_point(self._start, 'o', color='green')
        self.draw_point(self._end, 'o', color='red')
        t = zip(*self.obstacle)
        plt.plot(t[0], t[1], 's', color='black')
        for r in rrt.items():
            if r[1][2]:
                x = r[1][2][0]
                y = r[1][2][1]
                plt.plot(x, y, color='lightblue')
        if self._update_i % 20 == 1:
            self.update()

    def set_rrt_connect(self, rrta, rrtb):
        plt.clf()
        plt.plot(self._border_x, self._border_y, 'black')
        plt.plot(self.start[0], self.start[1], 'o', color='green')
        plt.plot(self.end[0], self.end[1], 'o', color='red')
        t = zip(*self.obstacle)
        plt.plot(t[0], t[1], 's', color='black')
        for r in rrta.items():
            t = zip(*r)
            plt.plot(t[0], t[1], color='lightblue')
        for r in rrtb.items():
            t = zip(*r)
            plt.plot(t[0], t[1], color='lightblue')
        if self._update_i % 20 == 1:
            self.update()

    @property
    def open(self):
        return self._open

    @open.setter
    def open(self, o):
        self._open = o
        plt.plot(self.open[0], self.open[1], 'x', color='lightblue')
        self._update_i += 1
        if self._update_i % 10 == 1:
            self.update()

    @property
    def close(self):
        return self._close

    @close.setter
    def close(self, o):
        self._close = o
        plt.plot(self.close[0], self.close[1], 'x', color='blue')
        self._update_i += 1
        if self._update_i % 10 == 1:
            self.update()

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, o):
        self._path = copy.deepcopy(o)
        x, y = zip(*self._path)
        plt.plot(x, y, color = 'green')

        
    def show(self):
        plt.axis('equal')
        plt.xlim((0, self.width))
        plt.ylim((0, self.height))

    def update(self):
        plt.pause(0.001)
    
    def wait_close(self):
        plt.show()


        
    def discretize(self, pos, rot):
        if rot < 0:
            rot += 2*np.pi

        rot = np.mod(rot,np.pi/2)

        # pos = middle object position
        # rot = angle around z axis in world frame 
        R = np.array([[np.cos(rot), -np.sin(rot)],[np.sin(rot), np.cos(rot)]])

        obstacle_dimension = np.array([self.nao_params.obstacle_dim[0], self.nao_params.obstacle_dim[1]])#self.nao_params.obstacle_dim # np.array(self.params.obstacle_dim )#np.array([7, 7]) 

        corner1 = -np.array(obstacle_dimension/2, dtype = float)
        corner2 = np.array(obstacle_dimension/2, dtype = float)
        corner3 = np.array([- (obstacle_dimension[0]/2), (obstacle_dimension[1]/2)], dtype = float)
        corner4 = np.array([+ (obstacle_dimension[0]/2), - (obstacle_dimension[1]/2)], dtype = float)

        if np.mod(rot, np.pi/2) != 0.0 or rot != 0.0:
            corner1 = tuple(pos + np.array((R.dot(corner1.reshape(-1,1))).reshape(-1), dtype = float))
            corner2 = tuple(pos + np.array((R.dot(corner2.reshape(-1,1))).reshape(-1), dtype = float))
            corner3 = tuple(pos + np.array((R.dot(corner3.reshape(-1,1))).reshape(-1), dtype = float))
            corner4 = tuple(pos + np.array((R.dot(corner4.reshape(-1,1))).reshape(-1), dtype = float))

        corners = [corner1, corner2, corner3, corner4]

        coord_up, coord_down, coord_left, coord_right = (None, None, None, None)
        # get upmost corner and leftmost corner 
        if np.mod(rot, np.pi/2) != 0  or rot != 0.0:
            coord_up    = np.array([(np.array(corners).reshape(4,2))[np.argmax((np.array(corners).reshape(4,2))[:,1]),0], (np.array(corners).reshape(4,2))[np.argmax((np.array(corners).reshape(4,2))[:,1]),1]], dtype = float)
            coord_left  = np.array([(np.array(corners).reshape(4,2))[np.argmin((np.array(corners).reshape(4,2))[:,0]),0], (np.array(corners).reshape(4,2))[np.argmin((np.array(corners).reshape(4,2))[:,0]),1]], dtype = float)
        else:
            coord_up = corner2
            coord_left = corner3

        # get bottom most corner and rightmost corner 
        if np.mod(rot, np.pi/2) != 0  or rot != 0.0:
            coord_down  = np.array([(np.array(corners).reshape(4,2))[np.argmin((np.array(corners).reshape(4,2))[:,1]),0], (np.array(corners).reshape(4,2))[np.argmin((np.array(corners).reshape(4,2))[:,1]),1]], dtype = float)
            coord_right = np.array([(np.array(corners).reshape(4,2))[np.argmax((np.array(corners).reshape(4,2))[:,0]),0], (np.array(corners).reshape(4,2))[np.argmax((np.array(corners).reshape(4,2))[:,0]),1]], dtype = float)
        else:
            coord_down = corner1
            coord_right = corner4

        delta, delta2, delta3, delta4 = (0,0,0,0)

        delta = np.array(coord_up - coord_left, dtype = float)
        a = (R.dot(np.array(obstacle_dimension/2, dtype = float).reshape(-1,1))).reshape(-1)
        x = np.array(np.linspace(pos[0] - delta[0] + a[0], pos[0] + a[0], 10), dtype = float)#int)
        y = np.array(np.linspace(pos[1] - delta[1] + a[1], pos[1] + a[1], 10), dtype = float)#int)

        delta2 = np.array(coord_right - coord_up, dtype = float)
        x2 = np.array(np.linspace(pos[0] + a[0], pos[0] + delta2[0] + a[0], 10), dtype = float)#int)
        y2 = np.array(np.linspace(pos[1] + a[1], pos[1] + delta2[1] + a[1], 10), dtype = float)#int)

        delta3 = np.array(coord_down - coord_left, dtype = float)
        x3 = np.array(np.linspace(pos[0] - delta3[0] - a[0], pos[0] - a[0], 10), dtype = float)#int)
        y3 = np.array(np.linspace(pos[1] - delta3[1] - a[1], pos[1] - a[1], 10), dtype = float)#int)

        delta4 = np.array(coord_right - coord_down, dtype = float)
        x4 = np.array(np.linspace(pos[0] - a[0], pos[0] + delta4[0] - a[0], 10), dtype = float)#int)
        y4 = np.array(np.linspace(pos[1] - a[1], pos[1] + delta4[1] - a[1], 10), dtype = float)#int)

        edge_coord1 = (np.vstack((x,y))).T
        edge_coord2 = (np.vstack((x2,y2))).T
        edge_coord3 = (np.vstack((x3,y3))).T
        edge_coord4 = (np.vstack((x4,y4))).T

        edge = []

        for i in range((edge_coord1.shape)[0]):
            edge.append(tuple(edge_coord1[i]))
        
        for i in range((edge_coord2.shape)[0]):
            edge.append(tuple(edge_coord2[i]))

        for i in range((edge_coord3.shape)[0]):
            edge.append(tuple(edge_coord3[i]))

        for i in range((edge_coord4.shape)[0]):
            edge.append(tuple(edge_coord4[i]))

        corner1 = - np.array(obstacle_dimension/2, dtype = float)
        corner2 = np.array(obstacle_dimension/2, dtype = float)
        corner3 = np.array([- (obstacle_dimension[0]/2), (obstacle_dimension[1]/2)], dtype = float)
        corner4 = np.array([+ (obstacle_dimension[0]/2), - (obstacle_dimension[1]/2)], dtype = float)

        corner1 = tuple(pos + np.array((R.dot(corner1.reshape(-1,1))).reshape(-1), dtype = float))#int))
        corner2 = tuple(pos + np.array((R.dot(corner2.reshape(-1,1))).reshape(-1), dtype = float))#int))
        corner3 = tuple(pos + np.array((R.dot(corner3.reshape(-1,1))).reshape(-1), dtype = float))#int))
        corner4 = tuple(pos + np.array((R.dot(corner4.reshape(-1,1))).reshape(-1), dtype = float))#int))

        corners = [corner1, corner2, corner3, corner4]

        obstacle = corners + edge + [tuple(pos)]

        return obstacle       


if __name__ == "__main__":

    m = MapInfo()
    m.show()
    
    pos = [np.array([20, 20]), np.array([40, 40]), np.array([40, 20]), np.array([10, 40])]
    rot = [np.pi/2, 0.2, 0.5, np.pi/4]
    obstacles = [] 
    for i in range(len(pos)):
        obstacles += m.discretize(pos[i], rot[i])
    start =  (30, 0)
    end = (20, 45)
    
    m.start = start
    m.end = end
    m.obstacle = obstacles
    
    
    m.wait_close()