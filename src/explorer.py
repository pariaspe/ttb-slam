import rospy
import tf
from ttb_slam.turtlebot_control import MyTurtlebot

from math import cos, sin
import time
import random

from utils import quat_to_euler, calc_rel_pose


class Explorer:
    RATE = 0.02

    def __init__(self):
        self.turtle = MyTurtlebot()

    def do_bump_go(self, timeout=None):
        start_time = time.time()
        while self.turtle.is_running():
            pose = self.turtle.get_estimated_pose()
            dists = self.turtle.get_frontal_dist()

            # Maquina de estados
            if any(i < 0.50 for i in dists):  # FRENTE A OBSTACULO
                print("OBSTACULO DETECTADO EN ({0}, {1})".format(*calc_rel_pose(pose, dists[22])))
                # print("TTB at [{x}, {y}]".format(x=pose.position.x, y=pose.position.y))
                self.turtle.stop()

                yaw_rate = random.uniform(-1, 1)  # velocidad aleatoria
                self.turtle.set_vel(az=yaw_rate)
                time.sleep(2)  # giro 5 seg
                self.turtle.stop()
            else:  # AVANCE
                self.turtle.set_vel(vx=0.3)

            if timeout is not None and time.time() - start_time > timeout:
                break

            time.sleep(self.RATE)
        self.turtle.stop()

    def follow_path(self, path):
        for pose in path.plan.poses:
            print("Going to", pose.pose.position.x, pose.pose.position.y)
            self.turtle.set_pos(pose.pose.position.x, pose.pose.position.y)
        self.turtle.stop()
    
    def send_pos(self):
        return self.turtle.get_estimated_pose().position.x, self.turtle.get_estimated_pose().position.y