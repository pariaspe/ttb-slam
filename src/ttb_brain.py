import rospy
from nav_msgs.srv import LoadMap, GetPlan
from geometry_msgs.msg import PoseStamped, PointStamped
import actionlib
from actionlib_msgs.msg import GoalID
from ttb_slam.msg import ExploreAction, ExploreGoal

import tf
import sys
import time

goal_point = None


def ask_user(question, options):
    while True:
        print(question + " [{}]".format("/".join(options)))
        if sys.version_info > (3, 0):
            anw = input()
        else:
            anw = raw_input()

        if anw in options:
            break
        else:
            print("Invalid answer..\n")
    return anw


def getPoint(data):
    uncentered_point = PointStamped()
    
    listener = tf.TransformListener()
    listener.waitForTransform("/world", "/map", rospy.Time(0), rospy.Duration(4.0))
    uncentered_point.header.frame_id = "world"
    uncentered_point.header.stamp = rospy.Time(0)
    uncentered_point.point.x = data.point.x
    uncentered_point.point.y = data.point.y
    uncentered_point.point.z = data.point.z
    global goal_point
    goal_point = listener.transformPoint("map", uncentered_point)


def main():
    rospy.init_node("brain_ttb")
    rospy.loginfo("Brain started")

    # Waiting for Map Manager
    rospy.wait_for_service("/my_map/load")
    load_map_client = rospy.ServiceProxy("/my_map/load", LoadMap)

    # Waiting for Planner
    rospy.wait_for_service("/planner/path/get")
    get_path = rospy.ServiceProxy("/planner/path/get", GetPlan)

    sub_point = rospy.Subscriber("/clicked_point", PointStamped, getPoint)

    explorer_client = actionlib.SimpleActionClient("/explorer", ExploreAction)
    explorer_client.wait_for_server()

    newMap = bool(ask_user("> Do you want to explore a new map?", ["Y", "n"]) == "Y")
    if newMap:
        strategy = ask_user("> Choose exploration strategy (1: Bug, 2: BumpGo):", ["1", "2"])

        rospy.loginfo("Starting exploration.")
        goal = ExploreGoal(goal=GoalID(id=strategy))
        explorer_client.send_goal(goal)
        explorer_client.wait_for_result()
        result = explorer_client.get_result()

        if result.result.status == 3:
            rospy.loginfo('Exploration finished successfully.')
        else:
            rospy.loginfo("Exploration failed.")
    else:
        resp = load_map_client("Generated/explored_map.csv")
        print(resp.result == 0)

    time.sleep(1)  # wait a second to avoid 0,0 pose
    start = PoseStamped()
    # start.pose.position.x, start.pose.position.y = explorer.send_pos()
    rospy.loginfo("Initial Point: [{}, {}]".format(start.pose.position.x, start.pose.position.y))

    print("> Select Goal Point in rviz map:")
    while goal_point is None:
        time.sleep(0.1)

        # TODO Hacer un while en el que se puedan elegir mas puntos
        # TODO Anhadir un timeout

    goal = PoseStamped()
    goal.pose.position.x = goal_point.point.x
    goal.pose.position.y = goal_point.point.y

    rospy.loginfo("Goal Point: [{}, {}]".format(goal_point.point.x, goal_point.point.y))
    path = get_path(start, goal, 0.001)

    # explorer.follow_path(path)

    rospy.loginfo("Brain ended")


if __name__ == "__main__":
    main()
