# turtlebot3-slam

## Dependencies

```
sudo apt-get install ros-$ROSDISTRO-turtlebot3-bringup ros-$ROSDISTRO-turtlebot3-description ros-$ROSDISTRO-turtlebot3-gazebo ros-$ROSDISTRO-turtlebot3-msgs
```

## How to install ROS pkg
```
roscd && cd src
ln -s <path-to-cloned-repo> .
cd ..
catkin build
```

## How to launch
```
roslaunch ttb_slam <file>.launch
```

```
roscd ttb_slam && cd test
python2 <test-file>.py
```

## ARUCO Models Setup
```bash
roscd ttb_slam && cd assets/aruco_marks
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD
```

You can also add the export to your .bashrc

## DEMO Videos:

 [![DEMO EASY MAP](https://img.youtube.com/vi/RQzRqY-mfJk&ab_channel=PabloRoca/default.jpg)](https://www.youtube.com/watch?v=RQzRqY-mfJk&ab_channel=PabloRoca)
 
 [![DEMO MEDIUM MAP](https://img.youtube.com/vi/EA03E64It7I&ab_channel=PabloRoca/default.jpg)](https://www.youtube.com/watch?v=EA03E64It7I&ab_channel=PabloRoca)
 
 [![DEMO HARD MAP](https://img.youtube.com/vi/ilHbzyHuSp8&ab_channel=PabloRoca/default.jpg)](https://www.youtube.com/watch?v=ilHbzyHuSp8&ab_channel=PabloRoca)
 
 [![DEMO loading map](https://img.youtube.com/vi/LLQQiBQZgXA&ab_channel=PabloRoca/default.jpg)](https://www.youtube.com/watch?v=LLQQiBQZgXA&ab_channel=PabloRoca)

 [![DEMO VISUAL MARKERS 1](https://img.youtube.com/vi/Udp_SFlDTek&ab_channel=PabloRoca/default.jpg)](https://www.youtube.com/watch?v=ilHbzyHuSp8&ab_channel=PabloRoca)
 
 [![DEMO VISUAL MARKERS 2](https://img.youtube.com/vi/h6QtdQYMn84&ab_channel=PabloRoca/default.jpg)](https://www.youtube.com/watch?v=h6QtdQYMn84&ab_channel=PabloRoca)
