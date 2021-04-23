import time
from ttb_slam import MyTurtlebot
import random


RATE = 0.02

def main():
    print("Test Started")

    turtle = MyTurtlebot()
    time.sleep(2)

    while turtle.is_running():
        # Maquina de estados
        dists = turtle.get_frontal_dist()
        if any(i < 0.50 for i in dists):  # FRENTE A OBSTACULO
            print("OBSTACULO DETECTADO")
            turtle.stop()

            yaw_rate = random.uniform(-1, 1)  # velocidad aleatoria
            turtle.set_vel(az=yaw_rate)
            time.sleep(5)  # giro 5 seg
            turtle.stop()
        else:  # AVANCE
            turtle.set_vel(vx=0.3)

        time.sleep(RATE)

    print("Test Finished")

if __name__ == "__main__":
    main()
