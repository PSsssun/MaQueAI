cmake_minimum_required(VERSION 3.0.2)
project(localization)

## C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

## 消息定义
add_message_files(
  FILES
  LocalizationStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    tf2 
    tf2_ros
    tf2_geometry_msgs
    message_runtime
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## 定位处理库
add_library(${PROJECT_NAME}
  src/slam_processor.cpp
  src/pose_filter.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## 定位节点
add_executable(slam_processor_node src/slam_processor_node.cpp)
add_dependencies(slam_processor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_processor_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## Python脚本（用于调试和可视化）
catkin_install_python(PROGRAMS
  scripts/slam_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 