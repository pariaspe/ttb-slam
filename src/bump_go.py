import time
from ttb_control.turtlebot_control import MyTurtlebot
import random

from utils import quat_to_euler, calc_rel_pose

RATE = 0.02


def bump_go(turtle, timeout=None, check_connectivity=None):
    check_timeout = 10
    timer = time.time()
    start_time = time.time()
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

        if check_connectivity is not None and time.time() - timer > check_timeout:
            # Check if map is completed, if not, add 60s exploration
            # Check whether the ttb arrived to the initial position
            if check_connectivity():
                print('map is COMPLETE')
                break
            else:
                print('map is incomplete, keep exploring')
            timer = time.time()

        if timeout is not None and time.time() - start_time > timeout:
            break

        time.sleep(RATE)
    turtle.stop()


def main():
    print("Test Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    bump_go(turtle)
    turtle.stop()

    print("Test Finished")


if __name__ == "__main__":
    main()
