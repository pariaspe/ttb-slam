import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from gazebo_msgs.msg import ModelStates
from laser_geometry import LaserProjection

from math import sin, cos, radians, ceil
import tf
import numpy
#import __future__
#from __future__ import print_function

resolution = 0.2
global_x = 0
global_y = 0
global_yaw = 0
full_scan = []
full_scan.append((0,0,0,0,0))
count = 0
scanned_map = np.zeros((100, 100)) #initialize map to zero, then we fill it. Each cell represent a resolution squared area

def quat_to_euler(orientation):
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(quat)  # roll, pitch, yaw

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.laserSub = rospy.Subscriber("gazebo/model_states", ModelStates, self.positionCallback)
    
    def laserCallback(self, data):

        global full_scan
        global resolution
        global count
        global scanned_map

        cloud_out = self.laserProj.projectLaser(data)   # Check transformLaserScanToPointCloud()

        point_generator = pc2.read_points(cloud_out)
        #point_list = pc2.read_points_list(cloud_out)
    
        for point in point_generator:
            rep_point = False
            angle = point[-1]
            global_point = (global_x + point[0]*cos(global_yaw) - point[1]*sin(global_yaw),
                            global_y + point[0]*sin(global_yaw) + point[1]*cos(global_yaw),
                            0, 0, 0)
            if angle == 400: #esto??
                #print(global_x)
                #print(sin(global_yaw))
                print('x:{point[0]}')
                #print(global_y)
                #print(cos(global_yaw))
                print('y:{point[1]}')
            count += 1
            #print(count)
            for scanned_point in full_scan:
                dist_x = global_point[0] - scanned_point[0]
                dist_y = global_point[1] - scanned_point[1]
                if abs(dist_x) < resolution and abs(dist_y) < resolution:
                    rep_point = True
            if not rep_point:
                full_scan.append(global_point)
                position_x = round(global_point[0]/resolution, 0) - 1 #position in map equals to rounded distance divided by resolution - 1
                position_y = round(global_point[1]/resolution, 0) - 1
                scanned_map[position_x][position_y] = 1 #mark occupied cell


                
        global_cloud_out = pc2.create_cloud(cloud_out.header, cloud_out.fields, full_scan)

        if count % 1000 == 0: #cada cierto numero de iteraciones llamamos a la funcion de crear mapa
            print(list(full_scan)) #primera columna parecen las x, segunda las y. Procesar dicha informacion

        self.pcPub.publish(global_cloud_out)
        

    def positionCallback(self, data):    
        global global_x
        global global_y
        global global_yaw
        #print(type(data.pose[0].position.x))
        robot_idx = data.name.index('turtlebot3_waffle_pi')
        global_x = data.pose[robot_idx].position.x
        global_y = data.pose[robot_idx].position.y
        _, _, global_yaw = quat_to_euler(data.pose[robot_idx].orientation)
        #print(global_yaw)

    def resizeMap(self, scan_data):
        global resolution
        global scanned_map
        #find max and min value in x and y // Seguramente se pueda hacer todo esto en un par de lineas 
        X_max = numpy.amax(scan_data[0])
        X_min = numpy.amin(scan_data[0])
        Y_max = numpy.amax(scan_data[1])
        Y_min = numpy.amin(scan_data[1])
        X_length = math.ceil((X_max - X_min)/resolution) 
        Y_length = math.ceil((Y_max - Y_min)/resolution)
        if  X_length - 1 > len(scanned_map[0]) or Y_length - 1 > len(scanned_map[1]): #if map size is not big enough append rows until it fits
            newrows = X_length - 1 - len(scanned_map[0])
            newcols = Y_length - 1 - len(scanned_map[1])
            if newrows > 0: #now we need to know at what side we stack the new rows
                scanned_map = np.stack((scanned_map,np.zeros((newrows,len(scanned_map[1])))), axis=0) #to stack on the other side, change stack positions
                #need to find a way to stack in the correct side
            if newcols > 0:
                scanned_map = np.stack((scanned_map,np.zeros((len(scanned_map[0]),newcols))), axis=-1)





if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    rospy.loginfo("Node initialized")
    l2pc = Laser2PC()
    rospy.spin()