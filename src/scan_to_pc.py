import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from laser_geometry import LaserProjection
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from math import sin, cos, radians, pi, degrees
import tf
import numpy as np
import time


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
    return [cos(radians(ang))*dist, sin(radians(ang))*dist]


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

    error = int(dx/2)
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


class Laser2PC:

    def __init__(self):
        self.global_x = 0
        self.global_y = 0
        self.global_yaw = 0
        self.global_ang_z = 0
        self.RESOLUTION = 0.05
        self.width = int(20/self.RESOLUTION)
        self.height = int(20/self.RESOLUTION)
        self.robot_in_grid = [0, 0]

        self.full_scan = []
        self.full_scan.append((0, 0, 0, 0, 0))
        self.scanned_map = np.zeros((self.width, self.height))  # initialize map to zero. Each cell represent a RESOLUTION squared area
        self.occupancy_grid = np.ones((self.width, self.height), dtype=np.int)*-1

        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.laserSub = rospy.Subscriber("/odom", Odometry, self.positionCallback)

        self.occup_grid_pub = rospy.Publisher("/my_map", OccupancyGrid, queue_size=1)

    def position_2_grid(self, x, y):
        """Translates real pose to grid position"""
        return [int(round(x/self.RESOLUTION, 0) - 1),
                int(round(y/self.RESOLUTION, 0) - 1)]

    def free_2_grid(self, laser_ranges, max_rng):
        robot_pos = self.position_2_grid(self.global_x, self.global_y)

        points = []
        for i, rng in enumerate(laser_ranges):
            if rng == float('inf'):
                rng = max_rng
            end_pos = map(lambda x, y: int(x+y), robot_pos,
                          polar_to_geom(i + degrees(self.global_yaw), rng/self.RESOLUTION))  # translation to robot_pos
            points += BresenhamAlgorithm(robot_pos, end_pos)

        for p in set(points):
            if self.occupancy_grid[p[0], p[1]] == -1:  # unknown --> visited
                self.occupancy_grid[p[0], p[1]] = 0
            elif 50 <= self.occupancy_grid[p[0], p[1]] < 90:  # not 100% sure --> substract 50
                self.occupancy_grid[p[0], p[1]] -= 30
            if 10 <= self.occupancy_grid[p[0], p[1]] < 50:  # unknown --> visited
                self.occupancy_grid[p[0], p[1]] -= 10

    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data)   # Check transformLaserScanToPointCloud()

        point_generator = pc2.read_points(cloud_out)
        
        if abs(self.global_ang_z) == 0.0:
            self.free_2_grid(data.ranges, data.range_max)

        for point in point_generator:
            rep_point = False

            # Assign a global point in the map to the relative point scanned
            global_point = (self.global_x + point[0]*cos(self.global_yaw) - point[1]*sin(self.global_yaw),
                            self.global_y + point[0]*sin(self.global_yaw) + point[1]*cos(self.global_yaw),
                            0, 0, 0)

            # Check if the point is repeated (into a resolution threshold)
            for scanned_point in self.full_scan:
                dist_x = global_point[0] - scanned_point[0]
                dist_y = global_point[1] - scanned_point[1]
                if abs(dist_x) < self.RESOLUTION and abs(dist_y) < self.RESOLUTION:
                    rep_point = True

            if not rep_point and abs(self.global_ang_z) == 0.0:  # avoid mapping while turning to avoid rotational error
                self.full_scan.append(global_point)
                position = self.position_2_grid(global_point[0], global_point[1])

                self.scanned_map[position[0]][position[1]] = 1  # mark occupied cell
                self.occupancy_grid[position[0]][position[1]] = 100  # mark occupied cell
                # MARK NEIGHBOURS WITH 50% PROB // add 20% probability if not 100% probability
                if self.occupancy_grid[int(position[0]+1), int(position[1])] < int(1):
                    self.occupancy_grid[int(position[0]+1), int(position[1])] = int(50)
                elif self.occupancy_grid[int(position[0]+1), int(position[1])] < int(90):
                    self.occupancy_grid[int(position[0]+1), int(position[1])] += int(20)

                if self.occupancy_grid[int(position[0]), int(position[1]+1)] < int(1):
                    self.occupancy_grid[int(position[0]), int(position[1]+1)] = int(50)
                elif self.occupancy_grid[int(position[0]), int(position[1]+1)] < int(90):
                    self.occupancy_grid[int(position[0]), int(position[1]+1)] += int(20)

                if self.occupancy_grid[int(position[0]-1), int(position[1])] < int(1):
                    self.occupancy_grid[int(position[0]-1), int(position[1])] = int(50)
                elif self.occupancy_grid[int(position[0]-1), int(position[1])] < int(90):
                    self.occupancy_grid[int(position[0]-1), int(position[1])] += int(20)

                if self.occupancy_grid[int(position[0]), int(position[1]-1)] < int(1):
                    self.occupancy_grid[int(position[0]), int(position[1]-1)] = int(50)
                elif self.occupancy_grid[int(position[0]), int(position[1]-1)] < int(90):
                    self.occupancy_grid[int(position[0]), int(position[1]-1)] += int(20)

        # Create the point cloud from the list
        global_cloud_out = pc2.create_cloud(cloud_out.header, cloud_out.fields, self.full_scan)
        global_cloud_out.header.frame_id = "map"  # sets the reference of the point cloud to the world

        self.pcPub.publish(global_cloud_out)    # publish the pc

    def positionCallback(self, data):
        self.global_x = data.pose.pose.position.x
        self.global_y = data.pose.pose.position.y
        _, _, self.global_yaw = quat_to_euler(data.pose.pose.orientation)
        self.global_ang_z = round(data.twist.twist.angular.z, 2)

    def send_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.map_load_time.secs = round(time.time())
        msg.info.resolution = self.RESOLUTION
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin = Pose()

        # Set the origin of the occupancy grid to fit the map
        grid_orientation = tf.transformations.quaternion_from_euler(pi, 0, pi/2)

        msg.info.origin.orientation.x = grid_orientation[0]
        msg.info.origin.orientation.y = grid_orientation[1]
        msg.info.origin.orientation.z = grid_orientation[2]
        msg.info.origin.orientation.w = grid_orientation[3]

        msg.data = self.occupancy_grid.ravel().tolist()
        self.occup_grid_pub.publish(msg)

    def print_scanned_map(self):
        for row in self.scanned_map:
            print("".join(map(lambda x: " " if not x else "O", row)))


if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    rospy.loginfo("Node initialized")
    l2pc = Laser2PC()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # os.system('clear')
        # l2pc.print_scanned_map()
        l2pc.send_occupancy_grid()
        rate.sleep()
