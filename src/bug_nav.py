import time
from ttb_slam.turtlebot_control import MyTurtlebot
import random
import tf
import math
from nav_msgs.msg import OccupancyGrid
import numpy as np

RATE = 0.02

def quat_to_euler(orientation):
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(quat)  # roll, pitch, yaw


def detect_wall(ranges):
    # Obtain the index of the closest wall
    min_dist = min(ranges)
    idx_min = ranges.index(min_dist)
    return idx_min

def remap_angle(angle):
    # Remaps the angle from [180, -180] to [0, 360]
    if angle <= 0:
        return -angle
    elif angle > 0:
        return 360 - angle

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
    
    print('Wall detected at',format(wall_idx))   #ROSINFO
    turtle.stop()

def calculate_wall_angle(full_distances, wall_angle):
    # Calculate the angle to the wall
    wall_side = math.sqrt(full_distances[wall_angle-15]**2 + full_distances[wall_angle]**2 - 2*full_distances[wall_angle-15]*full_distances[wall_angle]*math.cos(math.radians(15)))
    angle = round((360 - math.degrees(math.asin((full_distances[wall_angle-15]*math.sin(math.radians(15)))/wall_side))*2)/2)
    
    return angle

def rotate_against_wall(turtle, angle, direction):
    turtle.stop()
    #error = 4       # Little correction
    angle = angle #- error
    print('Rotating', angle, 'degrees')
    pose = turtle.get_estimated_pose()

    initial_orientation = math.degrees(quat_to_euler(pose.orientation)[2])  # Save initial orientation
    initial_orientation = remap_angle(initial_orientation)
    turtle_orientation = initial_orientation

    while abs(turtle_orientation - initial_orientation) < angle:  # Rotate until reached the angle
        turtle.set_vel(az=0.3 * direction)
        time.sleep(0.05)
        turtle_orientation = math.degrees(quat_to_euler(turtle.get_estimated_pose().orientation)[2])
        turtle_orientation = remap_angle(turtle_orientation)

        if initial_orientation + angle > 360:   # Check if the turn will overlap
            if turtle_orientation < 100:
                turtle_orientation += 360
        elif initial_orientation - angle < 0:
            if turtle_orientation > 280:
                turtle_orientation -= 360
#necesitamos pasar el grid a la funcion        
def map_connectivity(grid):
    finished = True
    len_x, len_y = np.shape(grid)
    list_of_zeros = np.transpose(np.where(grid == 0))
    for a in list_of_zeros:
        #list of zeros is a tuple
        x = a[0]
        y = a[1]
        #check adyacent positions
        if x > 0:
            if grid[x-1, y] == -1:
                finished = False
                print('map is not complete')
                return finished
        if x < len_x - 1:
            if grid[x+1, y] == -1:
                finished = False
                print('map is not complete')
                return finished
        if y > 0:
            if grid[x, y-1] == -1:
                finished = False
                print('map is not complete')
                return finished
        if y < len_y - 1:
            if grid[x, y+1] == -1:
                finished = False
                print('map is not complete')
                return finished
        return finished


def main():
    print("Bug Navigation Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    initial_position = turtle.get_estimated_pose().position

    print('Initial position is: \n',initial_position)

    timer = 0
    position_error = 1
    wall_dist = 1
    bug_initialized = False
    while turtle.is_running():
        pose = turtle.get_estimated_pose()
        full_distances = turtle.get_all_dist()      

        # Conditions for a good initialization
        if not bug_initialized:
            bug_initialization(turtle)
            bug_initialized = True
        
        turtle.set_vel(vx=0.3)
        time.sleep(0.05)
        

        # If the robot is not fully parallel to the wall
        if full_distances[85] - full_distances[95] > 0.02:
            print("Adjusting to the left...")
            turtle.set_vel(az=0.2, vx=0.3)
            time.sleep(0.2)
            
        elif full_distances[95] - full_distances[85] > 0.02:
            print("Adjusting to the right...")
            turtle.set_vel(az=-0.2, vx=0.3)
            time.sleep(0.2)

        if full_distances[0] < wall_dist: # If there is wall in front
            print('Detected wall in front')
            turtle.stop()   # Stop turtle
            angle = calculate_wall_angle(full_distances, 0)
            rotate_against_wall(turtle, angle, -1)      
        
        if full_distances[75] - full_distances[90] > 1: # gap on left side
            print("Detected a gap on the side")         

            turtle.set_vel(vx=0.3)
            time.sleep(3)
            turtle.stop()
            rotate_against_wall(turtle, 90, 1)      # assumed it is a 90 degree turn
            turtle.set_vel(vx=0.3)      # Advance to see where the next wall is
            time.sleep(3)
            
        
        if timer > 60:     # Little delay to give time to move from original position
            # Check if map is completed, if not, add 60s exploration
            # Check whether the ttb arrived to the initial position
            map_finished = map_connectivity()
            if map_finished:
                return map_finished
            else:
                timer = 0
            current_position = turtle.get_estimated_pose().position
            if abs(current_position.y - initial_position.y) < position_error and abs(current_position.x - initial_position.x) < position_error:
                turtle.stop()
                break
            timer = 100
        timer +=1


    print("Exploration Finished")
    print('Final position is: \n',current_position)
    # return map_finished
if __name__ == "__main__":
    main()
