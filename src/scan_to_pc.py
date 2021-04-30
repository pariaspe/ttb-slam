import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from gazebo_msgs.msg import ModelStates
from laser_geometry import LaserProjection
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from math import sin, cos, radians, ceil
import tf
import numpy as np
import time
import os


def quat_to_euler(orientation):
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(quat)  # roll, pitch, yaw

class Laser2PC():
    RESOLUTION = 0.2

    def __init__(self):
        self.global_x = 0
        self.global_y = 0
        self.global_yaw = 0

        self.full_scan = []
        self.full_scan.append((0,0,0,0,0))
        self.scanned_map = np.zeros((100, 100)) #initialize map to zero, then we fill it. Each cell represent a RESOLUTION squared area
        self.occupancy_grid = np.ones((100, 100), dtype=np.int)*-1

        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.laserSub = rospy.Subscriber("gazebo/model_states", ModelStates, self.positionCallback)  # TODO: cambiar a /Odom

        self.occup_grid_pub = rospy.Publisher("/my_map", OccupancyGrid, queue_size=1)

    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data)   # Check transformLaserScanToPointCloud()

        point_generator = pc2.read_points(cloud_out)
        #point_list = pc2.read_points_list(cloud_out)

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
            if not rep_point:
                self.full_scan.append(global_point)
                position_x = int(round(global_point[0]/self.RESOLUTION, 0) - 1) #position in map equals to rounded distance divided by RESOLUTION - 1
                position_y = int(round(global_point[1]/self.RESOLUTION, 0) - 1)
                self.scanned_map[position_x][position_y] = 1 #mark occupied cell
                self.occupancy_grid[position_x][position_y] = 100 #mark occupied cell


        # Create the point cloud from the list
        global_cloud_out = pc2.create_cloud(cloud_out.header, cloud_out.fields, self.full_scan)
        global_cloud_out.header.frame_id = "map"  # sets the reference of the point cloud to the world


        self.pcPub.publish(global_cloud_out)    # publish the pc


    def positionCallback(self, data):
        #print(type(data.pose[0].position.x))
        robot_idx = data.name.index('turtlebot3_waffle_pi')
        self.global_x = data.pose[robot_idx].position.x
        self.global_y = data.pose[robot_idx].position.y
        _, _, self.global_yaw = quat_to_euler(data.pose[robot_idx].orientation)
        #print(self.global_yaw)

    def resizeMap(self, scan_data):
        #find max and min value in x and y // Seguramente se pueda hacer todo esto en un par de lineas
        X_max = numpy.amax(scan_data[0])
        X_min = numpy.amin(scan_data[0])
        Y_max = numpy.amax(scan_data[1])
        Y_min = numpy.amin(scan_data[1])
        X_length = math.ceil((X_max - X_min)/self.RESOLUTION)
        Y_length = math.ceil((Y_max - Y_min)/self.RESOLUTION)
        if  X_length - 1 > len(self.scanned_map[0]) or Y_length - 1 > len(self.scanned_map[1]): #if map size is not big enough append rows until it fits
            newrows = X_length - 1 - len(self.scanned_map[0])
            newcols = Y_length - 1 - len(self.scanned_map[1])
            if newrows > 0: #now we need to know at what side we stack the new rows
                self.scanned_map = np.stack((self.scanned_map,np.zeros((newrows,len(self.scanned_map[1])))), axis=0) #to stack on the other side, change stack positions
                #need to find a way to stack in the correct side
            if newcols > 0:
                self.scanned_map = np.stack((self.scanned_map,np.zeros((len(self.scanned_map[0]),newcols))), axis=-1)

    def send_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "world"
        msg.info.map_load_time.secs = round(time.time())
        msg.info.resolution = self.RESOLUTION
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin = Pose()
        msg.data = self.occupancy_grid.ravel().tolist()
        self.occup_grid_pub.publish(msg)


    def print_scanned_map(self):
        for row in self.scanned_map:
            print("".join(map(lambda x: " " if not x else "O",row)))



if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    rospy.loginfo("Node initialized")
    l2pc = Laser2PC()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # os.system('clear')
        # l2pc.print_scanned_map()
        l2pc.send_occupancy_grid()
        rate.sleep()
