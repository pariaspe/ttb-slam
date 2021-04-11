import rospy
from geometry_msgs.msg import Twist


class MyTurtlebot:
    def __init__(self):
        rospy.init_node('turtlebot_node')
        rospy.loginfo("Turtlebot init.")

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.on_shutdown(self.shutdown)

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

    def shutdown(self):
        rospy.loginfo("Bye!")


if __name__ == "__main__":
    turtle = MyTurtlebot()
    rospy.spin()
