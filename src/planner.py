#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import Path


class Planner:
    def __init__(self):
        rospy.init_node("Planner")
        rospy.loginfo("Node initialized")

        rospy.wait_for_service("my_map/get")
        self.map_client = rospy.ServiceProxy("my_map/get", GetMap)

        self.path_srv = rospy.Service("planner/path/get", GetPlan, self.get_path)

    def get_path(self, req):
        start = req.start
        end = req.goal
        tolerance = req.tolerance

        map_ = self.map_client()
        # TODO calc path
        path = Path()
        return path


if __name__ == "__main__":
    planner = Planner()
    rospy.spin()
