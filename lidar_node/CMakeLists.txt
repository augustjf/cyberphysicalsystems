cmake_minimum_required(VERSION 3.0.2)
project(lidar_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${lidar_node_INCLUDE_DIRS}
)

add_executable(lidar_node src/lidar_node.cpp)

target_link_libraries(lidar_node
  ${catkin_LIBRARIES}
)

add_dependencies(lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})