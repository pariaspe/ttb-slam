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
    res = 2  # Resolution of the advancing angle
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
        #print(wall_idx)
    turtle.stop()



def main():
    print("Bug Navigation Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    initial_position = turtle.get_estimated_pose().position

    wall_angle = 10
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

        if full_distances[0] < 0.8: # wall in front
            turtle.stop()   # Stop turtle
            wall_side = math.sqrt(full_distances[-wall_angle]**2 + full_distances[0]**2 - 2*full_distances[-wall_angle]*full_distances[0]*math.cos(math.radians(wall_angle)))
            angle = round((360 - math.degrees(math.asin((full_distances[-wall_angle]*math.sin(math.radians(wall_angle)))/wall_side)*2))/2)

            #print(f'angle: {angle}, distance x+10: {full_distances[-wall_angle]}, sin 10: {math.sin(math.radians(wall_angle))}, wall side: {wall_side}')

            initial_orientation = pose.orientation.z
            turtle_orientation = initial_orientation
            while math.degrees(abs(turtle_orientation - initial_orientation)) < angle:
                turtle.set_vel(az=-0.3)
                time.sleep(0.05)
                turtle.stop()
                turtle_orientation = turtle.get_estimated_pose().orientation.z
                print(f'{turtle_orientation}, {math.degrees(initial_orientation)})')
        
        if False: # gap on side
            pass
        
        # Check whether the ttb arrived to the initial position
        current_position = turtle.get_estimated_pose().position
        if current_position.x - initial_position.x < error and current_position.y - initial_position.y < error:
            turtle.stop()
            break


    print("Exploration Finished")

if __name__ == "__main__":
    main()
