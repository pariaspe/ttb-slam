import time
from ttb_slam import MyTurtlebot
import random
import tf
from math import sin, cos
from nav_msgs.msg import OccupancyGrid

class MyNavigator:
    def __init__(self):
        rospy.init_node('navigator_node')
        rospy.loginfo("Navigator init.")

        self.map_sub = rospy.Subscriber('/topic_name', OccupancyGrid, self.Navigator)

    def Navigator(self):
        