cmake_minimum_required(VERSION 3.0.2)
project(planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    tf2 
    tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/trajectory_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 