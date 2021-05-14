import time
from ttb_slam import MyTurtlebot

print("Test Started")

poses = [(2.5, 5.5), (7.5, 5.5), (7.5, 2.5)]

turtle = MyTurtlebot()
time.sleep(2)

for p in poses:
    turtle.set_pos(x=p[0], y=p[1])

print("Test Finished")
