import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import SetMap, GetPlan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped

import numpy as np

from map import MyMap
from explorer import Explorer
import bug_nav

MAP = OccupancyGrid()
goal_point = PointStamped()
goal_point.point.x = 0


def my_map(data):
    global MAP
    MAP = data

def getPoint(data):
    global goal_point
    goal_point = data


def main():
    print("Brain started")

    # Waiting for Map Manager
    rospy.wait_for_service("/my_map/set")
    set_map_client = rospy.ServiceProxy("/my_map/set", SetMap)

    # Waiting for Planner
    rospy.wait_for_service("/planner/path/get")
    get_path = rospy.ServiceProxy("/planner/path/get", GetPlan)

    sub = rospy.Subscriber("/my_map", OccupancyGrid, my_map)
    sub_point = rospy.Subscriber("/clicked_point", PointStamped, getPoint)

    print("Do you want to explore a new map)[Y/n]")
    anw = input()
    if anw == 'n':
        newMap = False
    else:
        newMap = True

    if newMap == "Y":
        # bump and go navigation
        explorer = Explorer()
        explorer.do_bump_go(timeout=20)

        # bug navigation with connectivity detection
        map_finished = False
        timeout_counter = 0
        while not map_finished and timeout_counter < 4:
            map_finished = bug_nav.main()
            timeout_counter += 1

        if map_finished: print('map has been completed')
        else: print('map is not totally complete')

        print('exploration finished')
        resp = set_map_client(MAP, PoseWithCovarianceStamped())
        print(resp)
    else:
        explorer = Explorer()

        # Load explored map
        try:
            explored_map = np.genfromtxt("Generated/explored_map.csv", delimiter=',')
            print(explored_map)
        except ValueError:
            print("There is no map to load")

        map_ = MyMap(MyMap.upscale(explored_map, 16), resolution=1)
        resp = set_map_client(map_.to_msg(), PoseWithCovarianceStamped())
        print(resp)

    start = PoseStamped()
    start.pose.position.x = 2.5
    start.pose.position.y = 2.5
    goal = PoseStamped()
    while 1:
        goal.pose.position.x = goal_point.point.x
        goal.pose.position.y = goal_point.point.y
        if goal_point.point.x != 0:
            break
    path = get_path(start, goal, 0.001)

    explorer.follow_path(path)

    print("Brain ended")


if __name__ == "__main__":
    main()
