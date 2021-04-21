from __future__ import print_function
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
message = rospy.wait_for_message("/scan", LaserScan)

print('range at 0 degrees:', message.ranges[0])
print('range at 90 degrees:', message.ranges[89])
print('range at 180 degrees:', message.ranges[179])
print('range at 270 degrees:', message.ranges[269])

rospy.spin

