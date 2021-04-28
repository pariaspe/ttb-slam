import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from gazebo_msgs.msg import ModelStates
from laser_geometry import LaserProjection

from math import sin, cos, radians, ceil
import tf

#import __future__
#from __future__ import print_function

resolution = 0.2
global_x = 0
global_y = 0
global_yaw = 0
full_scan = []
full_scan.append((0,0,0,0,0))
count = 0

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

        cloud_out = self.laserProj.projectLaser(data)   # Check transformLaserScanToPointCloud()

        point_generator = pc2.read_points(cloud_out)
        
    
        for point in point_generator:
            rep_point = False
            angle = point[-1]
            global_point = (global_x + point[0]*cos(global_yaw) - point[1]*sin(global_yaw),
                            global_y + point[0]*sin(global_yaw) + point[1]*cos(global_yaw),
                            0, 0, 0)
            if angle == 400:
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
                
        global_cloud_out = pc2.create_cloud(cloud_out.header, cloud_out.fields, full_scan)

        if count % 1000 == 0: #cada cierto número de iteraciones llamamos a la función de crear mapa
            print(list(full_scan)) #primera columna parecen las x, segunda las y. Procesar dicha información

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

    def pointCloudToMap(self, scan_data):
        #find max and min value in x and y // Seguramente se pueda hacer todo esto en un par de lineas 
        X_max = numpy.amax(scan_data[1])
        X_min = numpy.amin(scan_data[1])
        Y_max = numpy.amax(scan_data[2])
        Y_min = numpy.amin(scan_data[2])
        X_length = math.ceil((X_max - X_min)/resolution) #quizá sea mejor utilizar math.ceil para redondear hacia arriba en vez de round
        Y_length = math.ceil((Y_max - Y_min)/resolution)
        map = np.zeros((X_length, Y_length)) #initialize map to zero, then we fill it
        #todavía no se me ha ocurrido como comprobar los puntos para rellenar la matriz, hay que implementarlo a continuación



if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    rospy.loginfo("Node initialized")
    l2pc = Laser2PC()
    rospy.spin()