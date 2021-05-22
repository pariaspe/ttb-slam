#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2

from greedy_navigator import best_first_search
from map import MyMap, generate_voronoi


class Planner:
    def __init__(self):
        rospy.init_node("Planner")
        rospy.loginfo("Node initialized")

        rospy.wait_for_service("my_map/get")
        self.map_client = rospy.ServiceProxy("my_map/get", GetMap)

        self.path_srv = rospy.Service("planner/path/get", GetPlan, self.get_path)

        self.path = Path()
        self.path_pb = rospy.Publisher("my_path", Path, queue_size=1)
        self.is_path = False
        self.path_timer = rospy.Timer(rospy.Duration(secs=1), self.publish_path)

    def get_path(self, req):
        start = req.start
        end = req.goal
        tolerance = req.tolerance

        resp = self.map_client()
        map_ = MyMap()
        map_.from_msg(resp.map)

        my_down = MyMap(grid=map_.downscale(map_.grid, 20), resolution=1 / 20)
        explored_map = my_down.binary
        cv2.imshow("Map BW", map_.to_img(True))

        voronoi_graph = generate_voronoi(explored_map)
        cv2.imshow("Voronoi", voronoi_graph)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        free_points = np.transpose(np.where(voronoi_graph == 0))  # Points in white in the voronoi
        min_start = 100
        min_end = 100
        
        # closest_start = min(free_points, key=lambda p: abs(p[0] - start.pose.position.x) + abs(p[1] - start.pose.position.y))
        # closest_end = min(free_points, key=lambda p: abs(p[0] - end.pose.position.x) + abs(p[1] - end.pose.position.y))
        # Get the closest point in the graph for start and end
        for point in free_points:
            tmp_start = (abs(point[0] - start.pose.position.x*10) + abs(point[1] - start.pose.position.y*10))
            tmp_end = (abs(point[0] - end.pose.position.x*10) + abs(point[1] - end.pose.position.y*10))
            if tmp_start < min_start:
                min_start = tmp_start
                start_point = point
            if tmp_end < min_end:
                min_end = tmp_end
                end_point = point
        path_list = best_first_search(voronoi_graph,
                                      (start_point[0], start_point[1]),
                                      (end_point[0], end_point[1]), 0)
        path_list = list(map(lambda i: (float(i[0])/10, float(i[1])/10), path_list))
        path_list.append((end.pose.position.x, end.pose.position.y))
        path_list.insert(0, (start.pose.position.x, start.pose.position.y))
        
        self.is_path = True
        self.path = Path()
        self.path.header.frame_id = "map"
        filtered_path = []
        filtered_path = np.copy(path_list[::2])
        if path_list[-1][0] != filtered_path[-1][0] or path_list[-1][1] != filtered_path[-1][1]: 
            filtered_path = np.vstack([filtered_path, path_list[-1]])
        
        filtered_path = np.reshape(filtered_path, (-1,2))
        no_obstacles = True
        n = 1
        while no_obstacles:
            try_path = self.interpolate_path(filtered_path,0.2*n)
            n += 1
            no_obstacles = self.check_obstacles(try_path)
            if no_obstacles: 
                smooth_path = np.copy(try_path)
                print('smoothened trajectory is valid')
        
        for p in smooth_path:
            point = PoseStamped()
            point.pose.position.x = p[0]
            point.pose.position.y = p[1]
            self.path.poses.append(point)
        return self.path

    def interpolate_path(self, og_path, resolution):
        duplicates = []
        filtered_path = np.copy(og_path)
        for i in range(1,len(filtered_path)):
            if np.allclose(filtered_path[i],filtered_path[i-1],rtol=0,atol=resolution):
                duplicates.append(i)
        if duplicates:
            filtered_path = np.delete(filtered_path, duplicates, axis=0)

        return filtered_path
            
    def check_obstacles(self, path):
        rospy.wait_for_service("my_map/get")
        map_client = rospy.ServiceProxy("my_map/get", GetMap)
        grid = map_client()
        rospy.wait_for_service("/my_map/binary/get")
        binarymap = rospy.ServiceProxy("/my_map/binary/get", GetMap)
        resp = binarymap()
        map = MyMap()
        map.from_msg(grid.map)
        binary = MyMap.from_msg(map, resp.map)
        for i in range(1,len(path)):
            dist = abs(path[i-1][0]-path[i][0] + path[i-1][1]-path[i][1])
            if dist > 0.2:
                elements = int(round((dist/0.2), 0))
            else:
                elements = 1
            tray_points = np.linspace(path[i-1],path[i],num=elements,dtype=int)
            if binary[tray_points] == 1:
                valid = False
                print('trajectory is going through obstacles, incorrect')
                break
        return valid
            


    def publish_path(self, event):
        if self.is_path:
            self.path_pb.publish(self.path)


if __name__ == "__main__":
    planner = Planner()
    rospy.spin()
