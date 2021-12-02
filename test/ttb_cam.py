import rospy
import time
from ttb_control.turtlebot_control import MyTurtlebot
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import image_geometry
import cv2 as cv
from cv2 import aruco
import message_filters
from sensor_msgs import point_cloud2 as pc2

RATE = 0.02
bridge = CvBridge()

cam_info_sub = None
cam_model = image_geometry.PinholeCameraModel()
pc = PointCloud2()

mark_pose = (0, 0, 0)


def cam_info_cb(msg):
    global cam_model
    cam_model.fromCameraInfo(msg)
    cam_info_sub.unregister()


def pc_cb(msg):
    global pc
    pc = msg


def callback(rgb, depth):
    """
    https://answers.ros.org/question/367829/converting-image-pixel-coordinates-2dx-y-to-3d-world-coordinatequaternion/
    https://answers.ros.org/question/372663/convert-pixel-coordinates-uv-to-pointcloud2-x-y-z-python/
    """
    global bridge
    rgb_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
    depth_img = bridge.imgmsg_to_cv2(depth, depth.encoding)
    height, width = rgb_img.shape[:2]

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(rgb_img, aruco_dict, parameters=parameters)

    if ids is not None:
        corner = corners[0][0]
        centroid = (corner[:, 0].mean(), corner[:, 1].mean())
        rgb_resized = cv.circle(rgb_img, centroid, radius=3, color=(0, 255, 255), thickness=-1)

        depth = get_depth(int(centroid[0]), int(centroid[1]))
        vector = cam_model.projectPixelTo3dRay(centroid)

        norm_vector = [el / vector[2] for el in vector]
        poses = [el * depth[0] for el in norm_vector]
        global mark_pose
        mark_pose = poses
        # print(poses)
    else:
        mark_pose = None
    frame_markers = aruco.drawDetectedMarkers(rgb_img.copy(), corners, ids)
    frame_markers_depth = aruco.drawDetectedMarkers(depth_img.copy(), corners, ids)

    rgb_resized = cv.resize(frame_markers, (width/4, height/4), interpolation=cv.INTER_AREA)
    depth_resized = cv.resize(frame_markers_depth, (width/4, height/4), interpolation=cv.INTER_AREA)

    cv.imshow("Depth", depth_resized)
    cv.imshow("RGB", rgb_resized)
    cv.waitKey(3)


def get_depth(x, y):
    global pc
    gen = pc2.read_points(pc, field_names='z', skip_nans=False, uvs=[(x, y)])
    return next(gen)


def main():
    print("Test Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    global cam_info_sub
    cam_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, cam_info_cb)
    pc_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, pc_cb)

    time.sleep(0.5)
    rgb_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)

    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(callback)

    while turtle.is_running():
        turtle.set_vel(az=0.3)

        pose = turtle.get_estimated_pose()
        turtle_pose = (pose.position.x, pose.position.y, pose.position.z)
        # if mark_pose is not None:
        #     # print(-mark_pose[2] + turtle_pose[0], mark_pose[0] + turtle_pose[1], mark_pose[2] + turtle_pose[0])
        #     print(mark_pose)

        time.sleep(RATE)

    cv.destroyAllWindows()
    print("Test Finished")


if __name__ == "__main__":
    main()
