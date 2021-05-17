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
# def generate_voronoi(original_img):
#     """
#     Loads a binary map and returns the voronoi representation
#     :param original_img:
#     :return:
#     """

#     # Load map as an image
#     ret, original_img = cv2.threshold(original_img, 0, 1, cv2.THRESH_BINARY_INV)

#     # Resize the image to improve voronoi precision
#     mult = 10
#     dim = (original_img.shape[1] * mult, original_img.shape[0] * mult)
#     original_img = cv2.resize(original_img, dim, interpolation = cv2.INTER_AREA)

#     img = original_img.copy()

#     size = np.size(img)
#     skel = np.zeros(img.shape, img.dtype)

#     # element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))  # Element for morph transformations
#     # img = cv2.erode(img, element)

#     element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))  # Element for morph transformations
#     done = False

#     # Skelitization
#     while not done:
#         eroded = cv2.erode(img, element)
#         temp = cv2.dilate(eroded, element)
#         temp = cv2.subtract(img, temp)
#         skel = cv2.bitwise_or(skel, temp)
#         img = eroded.copy()

#         # Stop when the image is fully eroded
#         zeros = size - cv2.countNonZero(img)
#         if zeros == size:
#             done = True

#     # Image showing

#     # cv2.imshow("skel",skel)
#     # cv2.imshow("image", original_img)
#     # cv2.waitKey(0)
#     # cv2.destroyAllWindows()

#     # Free spaces = 0
#     ret, final_img = cv2.threshold(skel, 0, 1, cv2.THRESH_BINARY_INV)
#     plt.imshow(final_img, cmap='Greys', interpolation='nearest')
#     plt.savefig('Generated/voronoi.png')
#     return final_img


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
        #cv2.waitKey(0)

        voronoi_graph = map_.run()
        print(voronoi_graph)
        path_list = best_first_search(voronoi_graph,
                                      (start.pose.position.x, start.pose.position.y),
                                      (end.pose.position.x, end.pose.position.y), 0)
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
