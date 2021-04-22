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
roslaunch turtlebot3_slam <file>.launch
```

```
roscd turtlebot3_slam && cd test
python2 <test-file>.py
```
