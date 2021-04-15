#import time
#from turtlebot_control import MyTurtlebot
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('rotate_node')

    # Create a publisher which can "talk" to Turtlesim and tell it to move
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(1)     
    # Create a Twist message and add linear x and angular z values
move_cmd = Twist()
move_cmd.linear.x = 1.0
move_cmd.angular.z = 1.0

reset_vel = Twist()
reset_vel.linear.x = 0
reset_vel.linear.y = 0
reset_vel.linear.z = 0
reset_vel.angular.x = 0
reset_vel.angular.x = 0
reset_vel.angular.x = 0

    # Save current time and set publish rate at 10 Hz
#now = rospy.Time.now()
#rate = rospy.Rate(10)

    # For the next 6 seconds publish cmd_vel move commands to Turtlesim
while not rospy.is_shutdown():
    print("Choose 1 to move the robot")
    opt = input()
    if opt == 1:
        print("The robot will move now")
        pub.publish(move_cmd)
    else:
        print("The robot will stop now")
        pub.publish(reset_vel)
    rate.sleep()
    
#while rospy.Time.now() < now + rospy.Duration.from_sec(6):
    #pub.publish(move_cmd)
    #rate.sleep()
