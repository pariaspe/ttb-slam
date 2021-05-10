import rospy
import time
from ttb_slam import MyTurtlebot
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import cv2 as cv


RATE = 0.02
bridge = CvBridge()

def cam_cb(msg):
    global bridge
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv.imshow("Turtlebot", img)
    cv.waitKey(3)


def main():
    print("Test Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    rospy.Subscriber('/camera/rgb/image_raw', Image, cam_cb)

    while turtle.is_running():
        turtle.set_vel(az=0.3)

        time.sleep(RATE)

    cv.destroyAllWindows()
    print("Test Finished")

if __name__ == "__main__":
    main()
