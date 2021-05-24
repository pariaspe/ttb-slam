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
import numpy as np


RATE = 0.02
bridge = CvBridge()

cam_info_sub = None
cam_model = image_geometry.PinholeCameraModel()
pc = PointCloud2()

#proj_matrix = []
#dist_coeff = []

mark_pose = (0, 0, 0)


def cam_info_cb(msg):
    global cam_model
    cam_model.fromCameraInfo(msg)
    cam_info_sub.unregister()

    #global proj_matrix 
    #global dist_coeff
    #proj_matrix = np.reshape(msg.K, (3,3))
    #dist_coeff = msg.D

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

    rgb_resized = cv.resize(rgb_img, (rgb.width, rgb.height), interpolation=cv.INTER_AREA)
    depth_resized = cv.resize(depth_img, (depth.width, depth.height), interpolation=cv.INTER_AREA)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    gray = cv.cvtColor(rgb_resized, cv.COLOR_BGR2GRAY)  # Change grayscale
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        corner = corners[0][0]
        centroid = (int(corner[:, 0].mean()), int(corner[:, 1].mean()))
        rgb_resized = cv.circle(rgb_resized, centroid, radius=3, color=(0, 255, 255), thickness=-1)

        depth = get_depth(int(centroid[0]), int(centroid[1]))
        vector = cam_model.projectPixelTo3dRay(centroid)

        
        #norm_vector = [el / vector[2] for el in vector]
        #poses = [el * depth[0] for el in norm_vector]
        global mark_pose
        #mark_pose = poses
        # print(poses)
        proj_matrix = np.reshape(cam_model.K, (3,3))
        dist_coeff = np.reshape(cam_model.D, 5)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.5, proj_matrix, dist_coeff)
        (rvecs- tvecs).any() # Get rid of numpy value array error
        rotation_matrix, _ = cv.Rodrigues(rvecs)    # Change from Rodrigues angles to rotation matrix
        tvecs = tvecs[0]    # There is just 1 marker, change with multiples
        trans_matrix = np.insert(np.insert(rotation_matrix, 3, tvecs, axis=1), 3, [0, 0, 0, 1], axis=0) # Obtain the transformation matrix

        pose_markers = rgb_resized.copy()
        #for rvec, tvec in rvecs, tvecs:
        aruco.drawAxis(pose_markers, proj_matrix, dist_coeff, rvecs, tvecs, 0.05)
        cv.imshow("markerPose", pose_markers)
    else:
        mark_pose = None
    frame_markers = aruco.drawDetectedMarkers(rgb_resized.copy(), corners, ids)
    frame_markers_depth = aruco.drawDetectedMarkers(depth_resized.copy(), corners, ids)
    
    

    
    cv.imshow("Depth", frame_markers)
    cv.imshow("Second", frame_markers_depth)
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
        turtle.set_vel(az=0.2)

        pose = turtle.get_estimated_pose()
        turtle_pose = (pose.position.x, pose.position.y, pose.position.z)
        if mark_pose is not None:
            # print(-mark_pose[2] + turtle_pose[0], mark_pose[0] + turtle_pose[1], mark_pose[2] + turtle_pose[0])
            #print(mark_pose)
            pass

        time.sleep(RATE)

    cv.destroyAllWindows()
    print("Test Finished")


if __name__ == "__main__":
    main()
