cmake_minimum_required(VERSION 3.0.2)
project(control)

## C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 编译优化
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3 -march=native")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  message_generation
)

find_package(Eigen3 REQUIRED)

## 动态重配置
generate_dynamic_reconfigure_options(
  cfg/ControllerParams.cfg
)

## 消息定义
add_message_files(
  FILES
  ControllerStatus.msg
  ControlCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
    dynamic_reconfigure
    message_runtime
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## 控制器库
add_library(${PROJECT_NAME}
  src/pid_controller.cpp
  src/trajectory_tracker.cpp
  src/safety_monitor.cpp
)
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## 轨迹跟踪节点
add_executable(trajectory_tracker_node src/trajectory_tracker_node.cpp)
add_dependencies(trajectory_tracker_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(trajectory_tracker_node ${PROJECT_NAME} ${catkin_LIBRARIES}) 