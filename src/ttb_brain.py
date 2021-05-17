import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import SetMap, GetPlan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from explorer import Explorer
import bug_nav

MAP = OccupancyGrid()


def my_map(data):
    global MAP
    MAP = data


def main():
    print("Brain started")

    print("Do you want to explore a new map)[Y/n]")
    newMap = input()

    if newMap:
        # Waiting for Map Manager
        rospy.wait_for_service("/my_map/set")
        set_map_client = rospy.ServiceProxy("/my_map/set", SetMap)

        # Waiting for Planner
        rospy.wait_for_service("/planner/path/get")
        get_path = rospy.ServiceProxy("/planner/path/get", GetPlan)

        sub = rospy.Subscriber("/my_map", OccupancyGrid, my_map)

        #bump and go navigation
        explorer = Explorer()
        explorer.do_bump_go(timeout=20)

        #bug navigation with connectivity detection
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

    start = PoseStamped()
    start.pose.position.x = 2.5
    start.pose.position.y = 2.5
    goal = PoseStamped()
    goal.pose.position.x = 2.5
    goal.pose.position.y = 5.5
    path = get_path(start, goal, 0.001) # Añadir el true para new en planner.py

    explorer.follow_path(path)

    print("Brain ended")



if __name__ == "__main__":
    main()
