import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import LoadMap, GetPlan
from geometry_msgs.msg import PoseStamped, PointStamped

import tf
import sys

from explorer import Explorer
import bug_nav
from ttb_slam.turtlebot_control import MyTurtlebot
import time

goal_point = PointStamped()
goal_point.point.x = 0
noInfo = True


def getPoint(data):
    global goal_point
    uncentered_point = PointStamped()
    
    listener = tf.TransformListener()
    listener.waitForTransform("/world", "/map", rospy.Time(0), rospy.Duration(4.0))
    uncentered_point.header.frame_id = "world"
    uncentered_point.header.stamp = rospy.Time(0)
    uncentered_point.point.x = data.point.x
    uncentered_point.point.y = data.point.y
    uncentered_point.point.z = data.point.z
    goal_point = listener.transformPoint("map", uncentered_point)

    global noInfo
    noInfo = False


def main():
    print("Brain started")

    # Waiting for Map Manager
    rospy.wait_for_service("/my_map/load")
    load_map_client = rospy.ServiceProxy("/my_map/load", LoadMap)

    # Waiting for Planner
    rospy.wait_for_service("/planner/path/get")
    get_path = rospy.ServiceProxy("/planner/path/get", GetPlan)

    sub_point = rospy.Subscriber("/clicked_point", PointStamped, getPoint)

    print("Do you want to explore a new map? [Y/n]")
    if sys.version_info > (3, 0):
        anw = input()
    else:
        anw = raw_input()

    newMap = False if anw == 'n' else True
    if newMap:
        # bump and go navigation
        explorer = Explorer()
        #explorer.do_bump_go(timeout=20)

        # bug navigation with connectivity detection
        map_finished = False
        while not map_finished:
            map_finished = bug_nav.main()

        if map_finished: print('map has been completed')
        else: print('map is not totally complete')

        print('exploration finished')
    else:
        explorer = Explorer()

        resp = load_map_client("Generated/explored_map.csv")
        print(resp.result == 0)

    start = PoseStamped()
    start.pose.position.x, start.pose.position.y = explorer.send_pos()
    
    goal = PoseStamped()
    print("Your inital point is: ", (start.pose.position.x, start.pose.position.y))
    print("Select your goal in the map")
    
    # Hacer un while en el que se puedan elegir mas puntos
    # Anhadir un timeout
    global noInfo
    while noInfo:
        pass
    goal.pose.position.x = goal_point.point.x
    goal.pose.position.y = goal_point.point.y

    print("Selected goal Position is: ", (goal_point.point.x, goal_point.point.y))
    path = get_path(start, goal, 0.001)

    explorer.follow_path(path)

    print("Brain ended")


if __name__ == "__main__":
    main()
