from __future__ import print_function
import rospy
import numpy as np
#import turtlebot3_msgs
#from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
    print('number of range points:', len(msg.ranges))
    print('distances:\n 0 degrees:%.2f'% msg.ranges[0], 'm \n 90 degrees: %.2f'% msg.ranges[89], 'm\n 180 degrees: %.2f'% msg.ranges[179], 'm\n 270 degrees: %.2f'% msg.ranges[269],'m')
    rospy.sleep(2)

#filtra los valores de inf quedando None en sus posiciones
def mapper(ranger):
    instant_map = [None] * len(ranger.ranges)
    for i in range(len(ranger.ranges)):
        if ranger.ranges[i] != np.inf:
            instant_map[i] = ranger.ranges[i]
    rospy.sleep(5)
    print(list(instant_map))

rospy.init_node('scan_sub_node')

sub = rospy.Subscriber('/scan', LaserScan, callback)

mapping = rospy.Subscriber('/scan', LaserScan, mapper) 

rospy.spin()

