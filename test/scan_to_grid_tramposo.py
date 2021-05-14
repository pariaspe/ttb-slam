#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose

from math import sin, cos, radians, pi, degrees
import tf
import numpy as np
import time
from binary_map import MyBinaryMap
import matplotlib.pyplot as plt

def quat_to_euler(orientation):
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(quat)  # roll, pitch, yaw


def polar_to_geom(ang, dist):
    """
    Converts polar coord to geometric coord. It supposes origin as (0, 0)
    :param ang: angle (degrees)
    :param dist: length
    :return: [x, y]
    """
    return [cos(radians(ang)) * dist, sin(radians(ang)) * dist]


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


class MapGrid:
    t0 = time.time()

    def __init__(self, resolution, width, height):
        self.frame = "map"
        self.resolution = resolution
        #for binary resolutions < 0.5 ttb must stop before generating map or range error will distort map
        self.binary_resol = 1
        self.width = width
        self.height = height
        self.plotgrid = True
        self.grid = np.ones((self.width, self.height), dtype=np.int) * -1
        # self.binary = np.zeros((40,40), dtype=np.int)

    def mark_as_free(self, x, y):
        # correction for wall error
        x = x+1
        y = y+1
        if self.grid[x, y] == -1:  # unknown --> visited
            self.grid[x, y] = 20
        elif 50 <= self.grid[x, y] < 100:  # not 100% sure --> subtract 30
            self.grid[x, y] -= 20
        elif 10 <= self.grid[x, y] < 50:
            self.grid[x, y] -= 10
        elif self.grid[x, y] < 10:
            self.grid[x, y] = 0

    def mark_as_occupied(self, x, y):
        # correction for wall error
        x = x+1
        y = y+1
        # mark occupied cell
        if 50 < self.grid[x, y] <= 90:
            self.grid[x, y] += 10
        elif 50 >= self.grid[x, y]:
            self.grid[x, y] += 20
        elif self.grid[x, y] > 90:
            self.grid[x, y] = 100

        # TEMPORAL #
        t1 = time.time()
        total = int(t1 - self.t0)
        if total % 50 == 0: self.plotgrid = True
        if total > 20 and self.plotgrid:
            self.binary_map = MyBinaryMap(self.grid, self.binary_resol/self.resolution).run()
            self.plotgrid = False
            np.savetxt("Generated/occupancy_grid_test.csv", self.grid, delimiter=",")
      
    def _mark_as_probable_obs(self, x, y):
        if self.grid[int(x), int(y)] < 1:
            self.grid[int(x), int(y)] = 50
        elif self.grid[int(x), int(y)] < 90:
            self.grid[int(x), int(y)] += 20

    def to_msg(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.info.map_load_time.secs = round(time.time())
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        # Set the origin of the occupancy grid to fit the map
        grid_orientation = tf.transformations.quaternion_from_euler(pi, 0, pi / 2)
        msg.info.origin = Pose()
        msg.info.origin.orientation.x = grid_orientation[0]
        msg.info.origin.orientation.y = grid_orientation[1]
        msg.info.origin.orientation.z = grid_orientation[2]
        msg.info.origin.orientation.w = grid_orientation[3]
        msg.data = self.grid.ravel().tolist()

        return msg


class Laser2Grid:
    RESOLUTION = 0.05
    GRID_RATE = 100000000  # 0.1 secs

    def __init__(self, headless=False):
        if not headless:
            rospy.init_node("Laser2Grid")
            rospy.loginfo("Node initialized")

        self.global_x = 0
        self.global_y = 0
        self.global_yaw = 0
        self.global_ang_z = 0
        #if dimensions are not squared, it doesn't work properly
        self.width = int(25 / self.RESOLUTION)
        self.height = int(25 / self.RESOLUTION)

        self.grid_map = MapGrid(self.RESOLUTION, self.width, self.height)
        
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        self.laserSub = rospy.Subscriber("/odom", Odometry, self.position_cb)

        self.occup_grid_pub = rospy.Publisher("/my_map", OccupancyGrid, queue_size=1)
        self.grid_timer = rospy.Timer(rospy.Duration(nsecs=self.GRID_RATE), self.send_occupancy_grid)

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
            if not is_obs:
                rng = data.range_max
            end_pos = map(lambda x, y: int(x + y), robot_pos,
                          polar_to_geom(i + degrees(self.global_yaw),
                                        rng / self.RESOLUTION))  # translation 2 robot_pos
            points += BresenhamAlgorithm(robot_pos, end_pos)

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

    def send_occupancy_grid(self, event):
        """Publish OccupancyGrid map"""
        self.occup_grid_pub.publish(self.grid_map.to_msg())


if __name__ == '__main__':
    l2pc = Laser2Grid()
    rospy.spin()
