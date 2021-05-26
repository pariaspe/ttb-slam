cmake_minimum_required(VERSION 2.8.3)
project(ttb_slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  cv_bridge
  image_geometry
  message_filters
)

add_message_files(
    FILES
)

add_action_files(DIRECTORY action
    FILES
    Explore.action
)

catkin_python_setup()

generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

catkin_package()


install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
