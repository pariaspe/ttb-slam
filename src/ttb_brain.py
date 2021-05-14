import rospy
from bug_nav import main
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import SetMap, GetPlan

MAP = OccupancyGrid()

def my_map(data):
    global MAP
    MAP = data

rospy.loginfo("Brain started")

# Waiting for Map Manager
rospy.wait_for_service("my_map/set")
set_map_client = rospy.ServiceProxy("my_map/set", SetMap)

# Waiting for Planner
rospy.wait_for_service("planner/path/get")
get_path = rospy.ServiceProxy("planner/path/get", GetPlan)

sub = rospy.Subscriber("/my_map", OccupancyGrid, my_map)

main()

set_map_client(MAP)

plan = GetPlan()
plan.start.pose.position.x = 2.5
plan.start.pose.position.y = 2.5
plan.goal.pose.position.x = 7.5
plan.goal.pose.position.y = 2.5
path = get_path(plan)

# print(path)

rospy.loginfo("Brain ended")
