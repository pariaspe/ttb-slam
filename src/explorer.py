#!/usr/bin/env python
import rospy
import actionlib
from ttb_control.turtlebot_control import MyTurtlebot
from bump_go import bump_go
from bug_nav import bug_nav
from map import MyMap
from nav_msgs.srv import GetMap
from ttb_slam.msg import ExploreAction, ExploreResult

from scan_to_grid import Laser2Grid

import numpy as np


status_enum = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING",
               "RECALLED", "LOST"]


class Explorer:
    RATE = 0.02

    def __init__(self):
        rospy.init_node("Explorer")

        self.turtle = MyTurtlebot(headless=True)
        self.do_autostop = True

        # Map generator
        self.map_generator = Laser2Grid(headless=True)

        rospy.wait_for_service("/my_map/get")
        self.map_client = rospy.ServiceProxy("/my_map/get", GetMap)
        self.server = actionlib.SimpleActionServer("/explorer", ExploreAction, self.do_explore, False)
        self.server.start()

    def do_explore(self, goal):
        print(goal)
        if goal.goal.id == "1":
            self.do_bug_nav(autostop=self.do_autostop)
        elif goal.goal.id == "2":
            self.do_bump_go(autostop=self.do_autostop)
        else:
            result = ExploreResult()
            result.result.goal_id.stamp = goal.goal.stamp
            result.result.goal_id.id = goal.goal.id
            result.result.status = status_enum.index("REJECTED")
            result.result.text = "Unknown exploration strategy"
            print(result)
            self.server.set_succeeded(result)

        result = ExploreResult()
        result.result.goal_id = goal.goal
        result.result.status = status_enum.index("SUCCEEDED")
        result.result.text = "Exploration completed"
        self.server.set_succeeded(result)

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
        threshold = 1
        len_x, len_y = np.shape(grid)
        list_of_zeros = np.transpose(np.where(grid == 0))
        print('list of possible points has dimensions: ', np.shape(list_of_zeros))
        for a in list_of_zeros:
            x, y = a
            if x > 0:
                if grid[x - 1, y] == -1:
                    adjacency_count += 1

            if x < len_x - 1:
                if grid[x + 1, y] == -1:
                    adjacency_count += 1

            if y > 0:
                if grid[x, y - 1] == -1:
                    adjacency_count += 1

            if y < len_y - 1:
                if grid[x, y + 1] == -1:
                    adjacency_count += 1

            if adjacency_count > threshold:
                finished = False
                print('map is incomplete')
                return finished
        if finished:
            print('the total number of adjacent points to unexplored ones is ', adjacency_count)
            
        return finished


if __name__ == "__main__":
    explorer = Explorer()
    rospy.spin()
