cmake_minimum_required(VERSION 3.0.2)
project(simulation)

## C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 编译优化
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  gazebo_ros
  gazebo_plugins
  gazebo_msgs
  mavros
  mavros_msgs
  rviz
  rosbag
  message_generation
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

## 消息定义
add_message_files(
  FILES
  SimulationStatus.msg
  TestResult.msg
  ScenarioCommand.msg
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
    rospy
    std_msgs 
    geometry_msgs 
    nav_msgs 
    sensor_msgs
    tf2 
    tf2_ros
    gazebo_ros
    mavros
    mavros_msgs
    message_runtime
  DEPENDS 
    GAZEBO
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## 仿真测试库
add_library(${PROJECT_NAME}
  src/simulation_manager.cpp
  src/test_scenario.cpp
  src/data_logger.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## 仿真管理节点
add_executable(simulation_manager_node src/simulation_manager_node.cpp)
add_dependencies(simulation_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulation_manager_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## 测试执行器
add_executable(test_executor_node src/test_executor_node.cpp)
add_dependencies(test_executor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_executor_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## 安装Python脚本
catkin_install_python(PROGRAMS
  scripts/run_simulation.py
  scripts/scenario_executor.py
  scripts/result_analyzer.py
  scripts/visualization_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 