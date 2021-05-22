#!/usr/bin/env python
import rospy
import actionlib
from ttb_control.turtlebot_control import MyTurtlebot
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback


class Navigator:
    RATE = 0.02

    def __init__(self):
        rospy.init_node("Navigator")

        self.turtle = MyTurtlebot(headless=True)

        # Waiting for Planner
        rospy.wait_for_service("/planner/path/get")
        self.get_path = rospy.ServiceProxy("/planner/path/get", GetPlan)

        self.server = actionlib.SimpleActionServer("/navigator", MoveBaseAction, self.do_navigate, False)
        self.server.start()

    def do_navigate(self, goal):
        pose = self.turtle.get_estimated_pose()
        rospy.loginfo("Initial Point: [{}, {}]".format(pose.position.x, pose.position.y))
        rospy.loginfo("Goal Point: [{}, {}]".format(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
        path = self.get_path(PoseStamped(pose=pose), goal.target_pose, 0.001)

        self.follow_path(path)

        self.server.set_succeeded()

    def follow_path(self, path):
        for pose in path.plan.poses:
            print("Going to", pose.pose.position.x, pose.pose.position.y)
            self.turtle.set_pos(pose.pose.position.x, pose.pose.position.y)
            self.server.publish_feedback(MoveBaseFeedback(base_position=pose))
        self.turtle.stop()


if __name__ == "__main__":
    navigator = Navigator()
    rospy.spin()
