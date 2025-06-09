cmake_minimum_required(VERSION 3.0.2)
project(interface)

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
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

## 消息定义
add_message_files(
  FILES
  MAVROSStatus.msg
  FlightState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    mavros_msgs
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

## MAVROS接口库
add_library(${PROJECT_NAME}
  src/mavros_bridge.cpp
  src/coordinate_transform.cpp
  src/safety_checker.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## MAVROS桥接节点
add_executable(mavros_bridge_node src/mavros_bridge_node.cpp)
add_dependencies(mavros_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavros_bridge_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## 位置发布节点 (SLAM -> PX4)
add_executable(vision_pose_publisher_node src/vision_pose_publisher_node.cpp)
add_dependencies(vision_pose_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_pose_publisher_node ${PROJECT_NAME} ${catkin_LIBRARIES}) 