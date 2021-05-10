import time
from ttb_slam import MyTurtlebot

print("Test Started")

turtle = MyTurtlebot()
time.sleep(2)

turtle.set_vel(az=0.5)
time.sleep(5)
turtle.stop()

turtle.set_vel(az=-0.5)
time.sleep(5)
turtle.stop()

print("Test Finished")
