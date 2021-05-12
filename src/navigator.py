import time
from ttb_slam.turtlebot_control import MyTurtlebot
import random
import tf
import math
from nav_msgs.msg import OccupancyGrid

class Navigator:

    def wall_searcher(self, ranges, resol):
        
        if ranges[0] != float('inf'):
            front_wall = True
        else:
            front_wall = False

        if ranges[89] != float('inf'):
            left_wall = True
        else:
            left_wall = False

        if ranges[179] != float('inf'):
            back_wall = True
        else:
            back_wall = False

        if ranges[269] != float('inf'):
            right_wall = True
        else:
            right_wall = False
        

        for i in (0,int(360/resol)):
            if ranges[i*resol]

    
    def center_robot(self, ranges, max_dist):


    def 