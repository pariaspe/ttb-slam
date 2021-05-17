#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

from greedy_navigator import best_first_search
from map import MyMap
from map import generate_voronoi
import map

class Planner:
    def __init__(self):
        rospy.init_node("Planner")
        rospy.loginfo("Node initialized")

        rospy.wait_for_service("my_map/binary/get")
        self.map_client = rospy.ServiceProxy("my_map/binary/get", GetMap)

        self.path_srv = rospy.Service("planner/path/get", GetPlan, self.get_path)

    def get_path(self, req):
        start = req.start
        end = req.goal
        tolerance = req.tolerance

        resp = self.map_client()
        map_ = MyMap()
        map_.from_msg(resp.map)
        cv2.imshow("Map BW", MyMap.to_img(map_.grid))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        voronoi_graph = map_.run()
        free_points = [voronoi_graph == 0]  # Points in white in the voronoi
        min_start = 10
        min_end = 10
        # Get the closest point in the graph for start and end
        for point in free_points:
            tmp_start = (abs(point[0] - start.pose.position.x) + abs(point[1] - start.pose.position.y))
            tmp_end = (abs(point[0] - end.pose.position.x) + abs(point[1] - end.pose.position.y))
            if tmp_start < min_start:
                min_start = tmp_start
            if tmp_end < min_end:
                min_end = tmp_end
        path_list = best_first_search(voronoi_graph,
                                      (min_start[0], min_start[1]),
                                      (min_end[0], min_end[1]), 0)
        path_list.append([end.pose.position.x, end.pose.position.y])
        path_list.insert(0, [start.pose.position.x, start.pose.position.y])
        path = Path()
        for p in path_list:
            point = PoseStamped()
            point.pose.position.x = p[0]
            point.pose.position.y = p[1]
            path.poses.append(point)
        return path


if __name__ == "__main__":
    planner = Planner()
    rospy.spin()
