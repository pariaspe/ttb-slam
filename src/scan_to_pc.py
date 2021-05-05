import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
#from gazebo_msgs.msg import ModelStates
from laser_geometry import LaserProjection
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

#from ttb_slam.turtlebot_control import MyTurtlebot

from math import sin, cos, tan, radians, ceil, pi
import tf
import numpy as np
import time
import os


def quat_to_euler(orientation):
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(quat)  # roll, pitch, yaw

class Laser2PC():

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
        self.full_scan.append((0,0,0,0,0))
        self.scanned_map = np.zeros((self.width, self.height)) #initialize map to zero, then we fill it. Each cell represent a RESOLUTION squared area
        self.occupancy_grid = np.ones((self.width, self.height), dtype=np.int)*-1
        
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.laserSub = rospy.Subscriber("/odom", Odometry, self.positionCallback)

        self.occup_grid_pub = rospy.Publisher("/my_map", OccupancyGrid, queue_size=1)

    # THIS FUNCTION ADAPTS REAL DISTANCES TO ITS CORRESPONDENT GRID POSITION
    def position_2_grid(self, X, Y): 
        position = []
        #position[0] = int(round(X/self.RESOLUTION, 0) - 1)
        #position[1] = int(round(Y/self.RESOLUTION, 0) - 1)
        position = [int(round(X/self.RESOLUTION, 0) - 1), int(round(Y/self.RESOLUTION, 0) - 1)]
        return(position)

    def free_to_map(self):
        self.robot_in_grid = self.position_2_grid(self.global_x, self.global_y)
        for i in range(self.robot_in_grid[0] - 1, self.robot_in_grid[0] + 3):
            for j in range(self.robot_in_grid[1] - 1, self.robot_in_grid[1] + 3):
                self.occupancy_grid[i, j] = 0

    def free_2_grid(self, robot_position, detection_list):
        #line equation from range and angle
        points_listed = np.zeros((360, 10, 2))
        for i in (0,359):
            points_listed = self.BresenhamAlgorithm(robot_position, detection_list[i])
            #No se si se puede comprobar de golpe para todos los valores, sino utilizar bucle for:
            # if self.occupancy_grid[points_listed[0],points_listed[1]] == -1: #if unknown, visited
            #     self.occupancy_grid[points_listed[0],points_listed[1]] = 0
            # elif self.occupancy_grid[points_listed[0],points_listed[1]] < 90: #if not 100% sure, subtract 50
            #     self.occupancy_grid[points_listed[0],points_listed[1]] -= 50
            
            for j in len(points_listed):
                if self.occupancy_grid[points_listed[j, 0],points_listed[j, 1]] == -1: #if unknown, visited
                    self.occupancy_grid[points_listed[j, 0],points_listed[j, 1]] = 0
                elif self.occupancy_grid[points_listed[j, 0],points_listed[j, 1]] < 90: #if not 100% sure, subtract 50
                    self.occupancy_grid[points_listed[j, 0],points_listed[j, 1]] -= 50

        

    #Bresenham Algorithm allows alliasing with integers only, useful to find cells in a line    
    def BresenhamAlgorithm(self, start, end):
        x1, y1 = start
        x2, y2 = end

        dx = x2 - x1
        dy = y2 - y1

        steep = abs(dy) - abs(dx)
        if steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        
        swapped = x1 > x2
        if swapped:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        #quizá se pueda hacer abs desde el principio
        dx = x2 - x1
        dy = y2 - y1

        error = int(dx/2)
        ystep = 1 if y1 < y2 else -1

        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        
        if swapped:
            points.reverse
        
        return(points)



    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data)   # Check transformLaserScanToPointCloud()

        point_generator = pc2.read_points(cloud_out)
        #point_list = pc2.read_points_list(cloud_out)
        
        #self.robot_in_grid = self.position_2_grid(self.global_x, self.global_y)
        #self.occupancy_grid[self.robot_in_grid[0], self.robot_in_grid[1]] = 1
        self.free_to_map()

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

            if not rep_point and abs(self.global_ang_z) == 0.0: #avoid mapping while turning to avoid additional rotational error
                self.full_scan.append(global_point)
                #position_x = int(round(global_point[0]/self.RESOLUTION, 0) - 1) #position in map equals to rounded distance divided by RESOLUTION - 1
                #position_y = int(round(global_point[1]/self.RESOLUTION, 0) - 1)
                position = self.position_2_grid(global_point[0], global_point[1])
                                                
                self.scanned_map[position[0]][position[1]] = 1 #mark occupied cell
                self.occupancy_grid[position[0]][position[1]] = 100 #mark occupied cell
                #MARK NEIGHBOURS WITH 50% PROB // add 20% probability if not 100% probability
                if  self.occupancy_grid[int(position[0]+1), int(position[1])]   < int(1):
                    self.occupancy_grid[int(position[0]+1), int(position[1])]   = int(50)
                elif self.occupancy_grid[int(position[0]+1), int(position[1])]   < int(90):
                    self.occupancy_grid[int(position[0]+1), int(position[1])]   += int(20)
                
                if  self.occupancy_grid[int(position[0]), int(position[1]+1)] < int(1):
                    self.occupancy_grid[int(position[0]), int(position[1]+1)] = int(50)
                elif self.occupancy_grid[int(position[0]), int(position[1]+1)]   < int(90):
                    self.occupancy_grid[int(position[0]), int(position[1]+1)]   += int(20)
                
                if  self.occupancy_grid[int(position[0]-1), int(position[1])]   < int(1):
                    self.occupancy_grid[int(position[0]-1), int(position[1])]   = int(50)
                elif self.occupancy_grid[int(position[0]-1), int(position[1])]   < int(90):
                    self.occupancy_grid[int(position[0]-1), int(position[1])]   += int(20)
                
                if  self.occupancy_grid[int(position[0]), int(position[1]-1)] < int(1):
                    self.occupancy_grid[int(position[0]), int(position[1]-1)] = int(50)
                elif self.occupancy_grid[int(position[0]), int(position[1]-1)]   < int(90):
                    self.occupancy_grid[int(position[0]), int(position[1]-1)]   += int(20)
                


        # Create the point cloud from the list
        global_cloud_out = pc2.create_cloud(cloud_out.header, cloud_out.fields, self.full_scan)
        global_cloud_out.header.frame_id = "map"  # sets the reference of the point cloud to the world


        self.pcPub.publish(global_cloud_out)    # publish the pc

    
    def positionCallback(self, data):
        self.global_x = data.pose.pose.position.x
        self.global_y = data.pose.pose.position.y
        _, _, self.global_yaw = quat_to_euler(data.pose.pose.orientation)
        self.global_ang_z = round(data.twist.twist.angular.z,2)
       

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
            print("".join(map(lambda x: " " if not x else "O",row)))



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
