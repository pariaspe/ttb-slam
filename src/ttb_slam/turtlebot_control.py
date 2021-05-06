#!/usr/bin/env python

import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class MyTurtlebot:
    def __init__(self):
        rospy.init_node('turtlebot_node')
        rospy.loginfo("Turtlebot init.")

        self.__is_running = True
        self.__ranges = [float('inf')] * 360
        self.__estimated_pose = Pose()

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        rospy.on_shutdown(self.__shutdown)

    def is_running(self):
        return self.__is_running

    def scan_cb(self, msg):
        self.__ranges = msg.ranges

    def odom_cb(self, msg):
        self.__estimated_pose = msg.pose.pose

    def get_estimated_pose(self):
        return self.__estimated_pose

    def get_frontal_dist(self):
        dist = self.__ranges[-22:] + self.__ranges[:23]  # from -22.5 to 22.5
        return dist
    
    def get_all_dist(self):
        dists = self.__ranges
        return dists

    def set_vel(self, vx=0, vy=0, vz=0, ax=0, ay=0, az=0):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = vy
        vel_msg.linear.z = vz
        vel_msg.angular.x = ax
        vel_msg.angular.y = ay
        vel_msg.angular.z = az

        self.vel_pub.publish(vel_msg)

    def get_sensor_dist_resol(self, angle_resol):
        default_resol = 10
        angle_resol = int(angle_resol) #to avoid introduced floats
        if 360 % angle_resol != 0 or angle_resol<1:
            print('Invalid resolution, setting value to default --> ',default_resol)
            angle_resol = default_resol
        
        vect_size = int(round(360/angle_resol, 0))
        sensor_resol = np.empty(vect_size, dtype=object)

        for i in range(1, vect_size):
            if self.__ranges[(i-1)*angle_resol] != np.inf : #filtering out of range values
                sensor_resol[i] = self.__ranges[(i-1)*angle_resol]
        
        return sensor_resol

    def stop(self):
        self.set_vel()

    def __shutdown(self):
        self.__is_running = False
        self.stop()  # stopping the robot before leaving
        rospy.loginfo("Bye!")


if __name__ == "__main__":
    turtle = MyTurtlebot()
    rospy.spin()
