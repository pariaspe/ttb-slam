import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from gazebo_msgs.msg import ModelStates
from laser_geometry import LaserProjection

from math import sin, cos, radians
import tf

resolution = 0.1
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
                print(f'x:{point[0]}')
                #print(global_y)
                #print(cos(global_yaw))
                print(f'y:{point[1]}')
            count += 1
            for scanned_point in full_scan:
                dist_x = global_point[0] - scanned_point[0]
                dist_y = global_point[1] - scanned_point[1]
                if abs(dist_x) < resolution and abs(dist_y) < resolution:
                    rep_point = True
            if not rep_point:
                full_scan.append(global_point)
                
        global_cloud_out = pc2.create_cloud(cloud_out.header, cloud_out.fields, full_scan)
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




if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    rospy.loginfo("Node initialized")
    l2pc = Laser2PC()
    rospy.spin()