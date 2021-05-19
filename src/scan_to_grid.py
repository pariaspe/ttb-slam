#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import SetMap

from math import degrees
import numpy as np
from map import MyMap

from utils import quat_to_euler, polar_to_geom


# Bresenham Algorithm allows aliasing with integers only, useful to find cells in a line
def BresenhamAlgorithm(start, end):
    x1, y1 = start
    x2, y2 = end
    correction = int(1)
    # to eliminate the point actually detected
    if x1 < x2:
        x2 -= correction
    else:
        x2 += correction
    if y1 < y2:
        y2 -= correction
    else:
        y2 += correction

    dx = x2 - x1
    dy = y2 - y1

    steep = abs(dy) - abs(dx)
    if steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    swapped = x1 > x2
    if swapped:
        # x2 += 2
        # y2 += 2
        x1, x2 = x2, x1
        y1, y2 = y2, y1

    # quiza se pueda hacer abs desde el principio
    dx = x2 - x1
    dy = y2 - y1

    error = int(dx / 2)
    ystep = 1 if y1 < y2 else -1

    y = y1
    points = []
    # for x in range(x1, x2 + 1):
    for x in range(x1, x2):
        coord = (y, x) if steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    if swapped:
        points.reverse

    return points


def get_middle_points(start, end, num=70):
    return list(set(zip(np.linspace(start[0], end[0], num).astype(int),
                        np.linspace(start[1], end[1], num).astype(int))))


class Laser2Grid:
    RESOLUTION = 0.05
    SET_MAP_RATE = 100000000  # 0.1 sec

    def __init__(self, headless=False):
        if not headless:
            rospy.init_node("Laser2Grid")
            rospy.loginfo("Node initialized")

        self.global_x = 0
        self.global_y = 0
        self.global_yaw = 0
        self.global_ang_z = 0
        # if dimensions are not squared, it doesn't work properly
        self.width = int(25 / self.RESOLUTION)
        self.height = int(25 / self.RESOLUTION)

        self.grid_map = MyMap(grid=np.ones((self.width, self.height), dtype=np.int) * -1, resolution=self.RESOLUTION)

        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        self.laserSub = rospy.Subscriber("/odom", Odometry, self.position_cb)

        # Waiting for Map Manager
        rospy.wait_for_service("/my_map/set")
        self.set_map_client = rospy.ServiceProxy("/my_map/set", SetMap)
        self.set_map_timer = rospy.Timer(rospy.Duration(nsecs=self.SET_MAP_RATE), self.set_map)

    def position_2_grid(self, x, y):
        """Translates real pose to grid position"""
        return [int(round(x / self.RESOLUTION, 0) - 1),
                int(round(y / self.RESOLUTION, 0) - 1)]

    def laser_cb(self, data):
        """Fills occupancy grid with the detected laser info"""
        if abs(self.global_ang_z) != 0.0:
            return

        robot_pos = self.position_2_grid(self.global_x, self.global_y)

        points = []
        for i, rng in enumerate(data.ranges):
            is_obs = False if rng == float('inf') else True
            rng = data.range_max if not is_obs else rng
            end_pos = list(map(lambda x, y: int(x + y), robot_pos,
                               polar_to_geom(i + degrees(self.global_yaw),
                                             rng / self.RESOLUTION)))  # translation 2 robot_pos
            points += get_middle_points(robot_pos, end_pos)

            if is_obs:
                self.grid_map.mark_as_occupied(end_pos[0], end_pos[1])

        for p in set(points):
            self.grid_map.mark_as_free(p[0], p[1])
        
    def position_cb(self, data):
        """Updates robot status"""
        self.global_x = data.pose.pose.position.x
        self.global_y = data.pose.pose.position.y
        _, _, self.global_yaw = quat_to_euler(data.pose.pose.orientation)
        self.global_ang_z = round(data.twist.twist.angular.z, 2)

    def set_map(self, event):
        """Send map to map manager via setter"""
        self.set_map_client(self.grid_map.to_msg(), PoseWithCovarianceStamped())


if __name__ == '__main__':
    l2pc = Laser2Grid()
    rospy.spin()
