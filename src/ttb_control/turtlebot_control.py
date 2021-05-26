#!/usr/bin/env python

import rospy
import numpy as np 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from PID import PID
from math import sqrt, pow, atan2

from utils import quat_to_euler


class MyTurtlebot:
    def __init__(self, headless=False):
        if not headless:
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

    def get_yaw(self):
        pose = self.get_estimated_pose()
        _, _, yaw = quat_to_euler(pose.orientation)
        return yaw

    def get_frontal_dist(self):
        dist = self.__ranges[-22:] + self.__ranges[:23]  # from -22.5 to 22.5
        return dist
    
    def get_all_dist(self):
        dists = self.__ranges
        return dists

    def set_vel(self, vx=0, az=0):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = az

        self.vel_pub.publish(vel_msg)

    def set_pos(self, x, y, tolerance=0.01):
        ang_pid = PID(P=2, I=0, D=0.75)
        linear_pid = PID(P=1, I=0, D=0.3)

        pose = self.get_estimated_pose()
        theta = atan2(y - pose.position.y, x - pose.position.x)
        ang_pid.setPoint(theta)
        ang_pid.update(self.get_yaw())
        while abs(ang_pid.getError()) > tolerance:
            az = ang_pid.update(self.get_yaw())
            self.set_vel(az=az)
        self.stop()

        dist = sqrt(pow(x, 2) + pow(y, 2))
        linear_pid.setPoint(dist)
        linear_pid.update(sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2)))
        while abs(linear_pid.getError()) > tolerance:
            pose = self.get_estimated_pose()

            ang_pid.setPoint(atan2(y - pose.position.y, x - pose.position.x))
            az = ang_pid.update(self.get_yaw())

            dist = sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2))
            vx = abs(linear_pid.update(dist))

            az = -0.75 if az < -0.75 else az  # min
            az = 0.75 if az > 0.75 else az  # max

            vx = 0.075 if vx < 0.075 else vx  # min
            vx = 0.5 if vx > 0.2 else vx  # max, std vel 0.5
            self.set_vel(vx=vx, az=az)
        self.stop()

    def get_sensor_dist_resol(self, angle_resol):
        default_resol = 10
        angle_resol = int(angle_resol)  # to avoid introduced floats
        if 360 % angle_resol != 0 or angle_resol<1:
            print('Invalid resolution, setting value to default --> ', default_resol)
            angle_resol = default_resol
        
        vect_size = int(round(360/angle_resol, 0))
        sensor_resol = np.empty(vect_size, dtype=object)

        for i in range(1, vect_size):
            if self.__ranges[(i-1)*angle_resol] != np.inf:  # filtering out of range values
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
