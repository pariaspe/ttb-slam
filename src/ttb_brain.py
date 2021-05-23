import rospy
from nav_msgs.srv import LoadMap
from geometry_msgs.msg import PoseStamped, PointStamped
import actionlib
from actionlib_msgs.msg import GoalID
from ttb_slam.msg import ExploreAction, ExploreGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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
        anw = anw.lower()
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

    sub_point = rospy.Subscriber("/clicked_point", PointStamped, getPoint)

    # Waiting for Explorer
    explorer_client = actionlib.SimpleActionClient("/explorer", ExploreAction)
    explorer_client.wait_for_server()

    # Waiting for Navigator
    navigator_client = actionlib.SimpleActionClient("/navigator", MoveBaseAction)
    navigator_client.wait_for_server()

    newMap = bool(ask_user("> Do you want to explore a new map?", ["y", "n"]) == "y")
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

    keepExploring = True

    while keepExploring:
        print("> Select Goal Point in rviz map:")
        while goal_point is None:
            time.sleep(0.1)

            # TODO Hacer un while en el que se puedan elegir mas puntos
            # TODO Anhadir un timeout

        goal_pose = PoseStamped()
        goal_pose.pose.position.x = goal_point.point.x
        goal_pose.pose.position.y = goal_point.point.y

        # Navigator
        rospy.loginfo("Starting navigation.")
        goal = MoveBaseGoal(target_pose=goal_pose)
        navigator_client.send_goal(goal)
        navigator_client.wait_for_result()
        result = navigator_client.get_result()
        print('navigator result is ',result)
        keepExploring = False
        #goal_point = None
        keepExploring = bool(ask_user("> Do you want to navigate to other point? publish point and introduce y", ["y", "n"]) == "y")

    rospy.loginfo("Brain ended")


if __name__ == "__main__":
    main()
