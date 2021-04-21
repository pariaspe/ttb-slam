#/usr/bin/env python

import rospy
#import turtlebot3_msgs
#from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
    print('in callback preprint')
    print('number of range points:', len(msg.ranges))
    print('in callback')

rospy.init_node('scan_sub_node')
sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
print('subscriber declared')
rospy.spin

