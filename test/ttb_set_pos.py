import time
from ttb_slam import MyTurtlebot

print("Test Started")

turtle = MyTurtlebot()
time.sleep(2)

turtle.set_pos(x=2.5, y=2.5)

print("Test Finished")
