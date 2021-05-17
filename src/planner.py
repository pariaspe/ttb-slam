#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2

from greedy_navigator import best_first_search
from map import MyMap
from map import generate_voronoi


class Planner:
    def __init__(self):
        rospy.init_node("Planner")
        rospy.loginfo("Node initialized")

        rospy.wait_for_service("my_map/get")
        self.map_client = rospy.ServiceProxy("my_map/get", GetMap)

        self.path_srv = rospy.Service("planner/path/get", GetPlan, self.get_path)

    def get_path(self, req, new=True):
        start = req.start
        end = req.goal
        tolerance = req.tolerance

        # New map exploration
        if new:
            resp = self.map_client()
            map_ = MyMap()
            map_.from_msg(resp.map)

            my_down = MyMap(grid=map_.downscale(map_.grid, 20), resolution=1 / 20)
            explored_map = my_down.binary
            cv2.imshow("Map BW", map_.to_img(True))
        # Load explored map
        else:
            try:
                explored_map = np.genfromtxt("Generated/explored_map.csv", delimiter=',')
            except ValueError:
                print("There is no map to load")

        voronoi_graph = generate_voronoi(explored_map)
        cv2.imshow("Voronoi", voronoi_graph)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        free_points = np.transpose(np.where(voronoi_graph == 0))  # Points in white in the voronoi
        min_start = 100
        min_end = 100
        # Get the closest point in the graph for start and end
        for point in free_points:
            tmp_start = (abs(point[0] - start.pose.position.x) + abs(point[1] - start.pose.position.y))
            tmp_end = (abs(point[0] - end.pose.position.x) + abs(point[1] - end.pose.position.y))
            if tmp_start < min_start:
                min_start = tmp_start
                start_point = point
            if tmp_end < min_end:
                min_end = tmp_end
                end_point = point
        path_list = best_first_search(voronoi_graph,
                                      (start_point[0], start_point[1]),
                                      (end_point[0], end_point[1]), 0)
        path_list.append([end.pose.position.x, end.pose.position.y])
        path_list.insert(0, [start.pose.position.x, start.pose.position.y])
        print("resuelto")
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
