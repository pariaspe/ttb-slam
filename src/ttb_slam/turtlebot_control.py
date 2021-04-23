#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class MyTurtlebot:
    def __init__(self):
        rospy.init_node('turtlebot_node')
        rospy.loginfo("Turtlebot init.")

        self.__is_running = True
        self.__ranges = [float('inf')] * 360

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        rospy.on_shutdown(self.__shutdown)

    def is_running(self):
        return self.__is_running

    def scan_cb(self, msg):
        self.__ranges = msg.ranges

    def get_frontal_dist(self):
        dist = self.__ranges[-22:] + self.__ranges[:23]  # from -22.5 to 22.5
        return dist

    def set_vel(self, vx=0, vy=0, vz=0, ax=0, ay=0, az=0):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = vy
        vel_msg.linear.z = vz
        vel_msg.angular.x = ax
        vel_msg.angular.y = ay
        vel_msg.angular.z = az

        self.vel_pub.publish(vel_msg)

    def stop(self):
        self.set_vel()

    def __shutdown(self):
        self.__is_running = False
        self.stop()  # stopping the robot before leaving
        rospy.loginfo("Bye!")


if __name__ == "__main__":
    turtle = MyTurtlebot()
    rospy.spin()
