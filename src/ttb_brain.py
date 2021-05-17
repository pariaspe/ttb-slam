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

    # Waiting for Map Manager
    rospy.wait_for_service("/my_map/set")
    set_map_client = rospy.ServiceProxy("/my_map/set", SetMap)

    # Waiting for Planner
    rospy.wait_for_service("/planner/path/get")
    get_path = rospy.ServiceProxy("/planner/path/get", GetPlan)

    sub = rospy.Subscriber("/my_map", OccupancyGrid, my_map)

    explorer = Explorer()
    explorer.do_bump_go(timeout=20)
    #bug_nav.main()
    print('exploration finished')
    resp = set_map_client(MAP, PoseWithCovarianceStamped())
    print(resp)

    start = PoseStamped()
    start.pose.position.x = 2.5
    start.pose.position.y = 2.5
    goal = PoseStamped()
    goal.pose.position.x = 2.5
    goal.pose.position.y = 5.5
    path = get_path(start, goal, 0.001)

    explorer.follow_path(path)

    print("Brain ended")
    rospy.spin()


if __name__ == "__main__":
    main()
