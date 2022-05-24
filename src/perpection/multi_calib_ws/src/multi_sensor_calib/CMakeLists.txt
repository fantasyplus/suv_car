cmake_minimum_required(VERSION 3.0.2)
project(multi_sensor_calib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES multi_sensor_calib
 CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
 
add_executable(${PROJECT_NAME}_node src/multi_sensor_calib_node.cpp src/multi_sensor_calib_core.cpp)
 
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
