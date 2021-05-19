import rospy
from ttb_slam.turtlebot_control import MyTurtlebot
from bump_go import bump_go
from bug_nav import bug_nav
from map import MyMap
from nav_msgs.srv import GetMap

import numpy as np


class Explorer:
    RATE = 0.02

    def __init__(self):
        self.turtle = MyTurtlebot()

        rospy.wait_for_service("/my_map/get")
        self.map_client = rospy.ServiceProxy("/my_map/get", GetMap)

    def do_bump_go(self, timeout=None, autostop=False):
        if autostop:
            bump_go(self.turtle, timeout, self.map_connectivity)
        else:
            bump_go(self.turtle, timeout)
        self.turtle.stop()

    def do_bug_nav(self, timeout=None, autostop=False):
        if autostop:
            bug_nav(self.turtle, timeout, self.map_connectivity)
        else:
            bug_nav(self.turtle, timeout)
        self.turtle.stop()

    # this function checks connectivity in free space,
    # if there is unexplored space next to it, map is not finished
    # due to bad precision, some free spaces are behind walls
    # possible solution would be counting number of adjacencies and if value
    # is under some threshold, we suppose the map is finished
    def map_connectivity(self):
        resp = self.map_client()
        map_gen = MyMap()
        grid = map_gen.from_msg(resp.map)

        finished = True
        adjacency_count = 0
        threshold = 20
        len_x, len_y = np.shape(grid)
        list_of_zeros = np.transpose(np.where(grid == 0))
        print('list of possible points has dimensions: ', np.shape(list_of_zeros))
        for a in list_of_zeros:
            # list of zeros is a tuple
            # print('adjacency count is: ',adjacency_count)
            x, y = a
            # print('checking position ',x,y)
            # check adyacent positions
            if x > 0:
                if grid[x - 1, y] == -1:
                    adjacency_count += 1
                    # if adyacency_count >= threshold:
                    #     finished = False
                    #     return finished
            if x < len_x - 1:
                if grid[x + 1, y] == -1:
                    adjacency_count += 1
                    # if adyacency_count >= threshold:
                    #     finished = False
                    #     return finished
            if y > 0:
                if grid[x, y - 1] == -1:
                    adjacency_count += 1
                    # if adyacency_count >= threshold:
                    #     finished = False
                    #     return finished
            if y < len_y - 1:
                if grid[x, y + 1] == -1:
                    adjacency_count += 1
                    # if adyacency_count >= threshold:
                    #     finished = False
                    #     return finished
        print('the total number of adjacent points to unexplored ones is ', adjacency_count)
        if adjacency_count > threshold:
            finished = False
            print('map is incomplete')
        return finished

    def follow_path(self, path):
        for pose in path.plan.poses:
            print("Going to", pose.pose.position.x, pose.pose.position.y)
            self.turtle.set_pos(pose.pose.position.x, pose.pose.position.y)
        self.turtle.stop()
    
    def send_pos(self):
        pose = self.turtle.get_estimated_pose()
        return pose.position.x, pose.position.y
