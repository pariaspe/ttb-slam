import time
from ttb_slam.turtlebot_control import MyTurtlebot
import random
import tf
import math
from nav_msgs.msg import OccupancyGrid

class Explorer:
    def __init__(self):
        self.dir =np.array([0,89,179,269], dtype=int)
        self.wall = [False,False,False,False]

    def wall_searcher(self, ranges, resol):
        
        if ranges[self.dir[0]] != float('inf'):
            self.wall[0] = True
        else:
            self.wall[0] = False

        if ranges[self.dir[1]] != float('inf'):
            self.wall[1] = True
        else:
            self.wall[1] = False

        if ranges[self.dir[2]] != float('inf'):
            self.wall[2] = True
        else:
            self.wall[2] = False

        if ranges[self.dir[3]] != float('inf'):
            self.wall[3] = True
        else:
            self.wall[3] = False

    def center_robot(self, ranges, max_dist):
        
        #When we have wall at both sizes
        if self.wall[1] and self.wall[3]:
            if ranges

    def 