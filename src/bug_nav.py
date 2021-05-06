import time
from ttb_slam.turtlebot_control import MyTurtlebot
import random
import tf
import math
from nav_msgs.msg import OccupancyGrid

RATE = 0.02

def quat_to_euler(orientation):
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(quat)  # roll, pitch, yaw

def calc_rel_pose(pose, dist):
    _, _, yaw = quat_to_euler(pose.orientation)
    return pose.position.x + cos(yaw)*dist, pose.position.y + sin(yaw)*dist

def detect_wall(ranges):
    # Obtain the index of the closest wall
    min_dist = min(ranges)
    idx_min = ranges.index(min_dist)
    return idx_min

def bug_initialization(turtle):
    resolution = 1  # Resolution of the advancing angle
    angle = 90      # Angle in which it is wanted to advance
    wall_idx = 0
    while wall_idx not in range(angle-res, angle + res + 1):
        full_distances = turtle.get_all_dist()
        wall_idx = detect_wall(full_distances)

        if full_distances[wall_idx] == float('inf'):    # If the bot does not detect any wall, advance
            turtle.set_vel(vx=0.3)
            break
        
        turtle.set_vel(az=0.2)  # Rotate until parallel to the wall
        time.sleep(0.05)
        print(wall_idx)
    turtle.stop()



def main():
    print("Bug Navigation Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    
    bug_initialized = False
    while turtle.is_running():
        pose = turtle.get_estimated_pose()
        full_distances = turtle.get_all_dist()      

        # Conditions for a good initialization
        if not bug_initialized:
            bug_initialization(turtle)
            bug_initialized = True
        
        turtle.set_vel(vx=0.3)
        wall_dist = full_distances[90]


    print("Test Finished")

if __name__ == "__main__":
    main()
