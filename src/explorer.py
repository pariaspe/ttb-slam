from ttb_slam.turtlebot_control import MyTurtlebot
from bump_go import bump_go


class Explorer:
    RATE = 0.02

    def __init__(self):
        self.turtle = MyTurtlebot()

    def do_bump_go(self, timeout=None):
        bump_go(self.turtle, timeout)
        self.turtle.stop()

    def follow_path(self, path):
        for pose in path.plan.poses:
            print("Going to", pose.pose.position.x, pose.pose.position.y)
            self.turtle.set_pos(pose.pose.position.x, pose.pose.position.y)
        self.turtle.stop()
    
    def send_pos(self):
        return self.turtle.get_estimated_pose().position.x, self.turtle.get_estimated_pose().position.y