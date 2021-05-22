import time
from map import MyMap
from ttb_control.turtlebot_control import MyTurtlebot
import math
from nav_msgs.srv import GetMap
import numpy as np
import rospy

from utils import quat_to_euler

RATE = 0.02


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


def calculate_wall_angle(full_distances, wall_angle):
    # Calculate the angle to the wall
    wall_side = math.sqrt(full_distances[wall_angle-15]**2 + full_distances[wall_angle]**2 - 2*full_distances[wall_angle-15]*full_distances[wall_angle]*math.cos(math.radians(15)))
    angle = round((360 - math.degrees(math.asin((full_distances[wall_angle-15]*math.sin(math.radians(15)))/wall_side))*2)/2)
    
    return angle


def rotate_against_wall(turtle, angle, direction):
    turtle.stop()
    # error = 4       # Little correction
    angle = angle  # - error
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


# this function checks connectivity in free space, 
# if there is unexplored space next to it, map is not finished
# due to bad precision, some free spaces are behind walls
# possible solution would be counting number of adyacencies and if value
# is under some threshold, we suppose the map is finished        
def map_connectivity():
    rospy.wait_for_service("/my_map/get")
    map_client = rospy.ServiceProxy("/my_map/get", GetMap)
    try:
        resp = map_client()
        print('service get_map successfully called')
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

    # print('shape is', np.shape(resp))
    map_gen = MyMap()
    # print('print map_ ',map_)
    grid = map_gen.from_msg(resp.map)
    finished = True
    adyacency_count = 0
    threshold = 20
    len_x, len_y = np.shape(grid)
    list_of_zeros = np.transpose(np.where(grid == 0))
    print('list of possible points has dimensions: ', np.shape(list_of_zeros))
    for a in list_of_zeros:
        # list of zeros is a tuple
        # print('adjacency count is: ',adjacency_count)
        x = a[0]
        y = a[1]
        # print('checking position ',x,y)
        # check adyacent positions
        if x > 0:
            if grid[x-1, y] == -1:
                adyacency_count += 1
                # if adyacency_count >= threshold:
                #     finished = False
                #     return finished
        if x < len_x - 1:
            if grid[x+1, y] == -1:
                adyacency_count += 1
                # if adyacency_count >= threshold:
                #     finished = False
                #     return finished
        if y > 0:
            if grid[x, y-1] == -1:
                adyacency_count += 1
                # if adyacency_count >= threshold:
                #     finished = False
                #     return finished
        if y < len_y - 1:
            if grid[x, y+1] == -1:
                adyacency_count += 1
                # if adyacency_count >= threshold:
                #     finished = False
                #     return finished
    print('the total number of adjacent points to unexplored ones is ', adyacency_count)
    if adyacency_count > threshold:
        finished = False
        print('map is incomplete')
    return finished


def bug_nav(turtle, timeout=None, check_connectivity=None):
    wall_dist = 1
    map_finished = False
    check_timeout = 10
    timer = time.time()

    start_time = time.time()
    while turtle.is_running() and not map_finished:
        full_distances = turtle.get_all_dist()

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

        if full_distances[0] < wall_dist:  # If there is wall in front
            print('Detected wall in front')
            turtle.stop()  # Stop turtle
            angle = calculate_wall_angle(full_distances, 0)
            rotate_against_wall(turtle, angle, -1)

        if full_distances[75] - full_distances[90] > 1:  # gap on left side
            print("Detected a gap on the side")

            turtle.set_vel(vx=0.3)
            time.sleep(3)
            turtle.stop()
            rotate_against_wall(turtle, 90, 1)  # assumed it is a 90 degree turn
            turtle.set_vel(vx=0.3)  # Advance to see where the next wall is
            time.sleep(3)

        if check_connectivity is not None and time.time() - timer > check_timeout:
            # Check if map is completed, if not, add 60s exploration
            # Check whether the ttb arrived to the initial position
            if check_connectivity():
                print('map is COMPLETE')
                break
            else:
                print('map is incomplete, keep exploring')
            timer = time.time()

        if timeout is not None and time.time() - start_time > timeout:
            print("Timeout")
            break


def main():
    print("Bug Navigation Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    initial_position = turtle.get_estimated_pose().position

    print('Initial position is: \n', initial_position)

    bug_nav(turtle, timeout=None, check_connectivity=map_connectivity)
    turtle.stop()


if __name__ == "__main__":
    main()
