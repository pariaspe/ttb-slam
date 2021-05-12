import rospy
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


RATE = 0.5


def pub_my_path(pub):
    pose = PoseStamped()
    pose.pose.position.x = 2.5
    pose.pose.position.y = 2.5

    pose2 = PoseStamped()
    pose2.pose.position.x = 2.5
    pose2.pose.position.y = 5

    pose3 = PoseStamped()
    pose3.pose.position.x = 7
    pose3.pose.position.y = 5

    msg = Path()
    msg.header.frame_id = "map"
    msg.poses = [pose, pose2, pose3]
    pub.publish(msg)


def main():
    print("Test Started")
    rospy.init_node("path_node")

    pub = rospy.Publisher("my_path", Path, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_my_path(pub)
        rate.sleep()

    print("Test Finished")


if __name__ == "__main__":
    main()
