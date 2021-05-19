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
