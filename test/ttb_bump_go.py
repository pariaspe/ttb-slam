import time
from ttb_slam.turtlebot_control import MyTurtlebot
import random
import tf
from math import sin, cos

RATE = 0.02


def quat_to_euler(orientation):
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(quat)  # roll, pitch, yaw


def calc_rel_pose(pose, dist):
    # OJO, no se si estara bien
    _, _, yaw = quat_to_euler(pose.orientation)
    return pose.position.x + cos(yaw)*dist, pose.position.y + sin(yaw)*dist


def main():
    print("Test Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    while turtle.is_running():
        pose = turtle.get_estimated_pose()
        dists = turtle.get_frontal_dist()
    
        # Maquina de estados
        if any(i < 0.50 for i in dists):  # FRENTE A OBSTACULO
            print("OBSTACULO DETECTADO EN ({0}, {1})".format(*calc_rel_pose(pose, dists[22])))
            # print("TTB at [{x}, {y}]".format(x=pose.position.x, y=pose.position.y))
            turtle.stop()

            yaw_rate = random.uniform(-1, 1)  # velocidad aleatoria
            turtle.set_vel(az=yaw_rate)
            time.sleep(2)  # giro 5 seg
            turtle.stop()
        else:  # AVANCE
            turtle.set_vel(vx=0.3)

        time.sleep(RATE)

    print("Test Finished")


if __name__ == "__main__":
    main()
