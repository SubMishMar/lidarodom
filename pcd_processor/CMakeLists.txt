cmake_minimum_required(VERSION 2.8.3)
project(pcd_processor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(libpointmatcher CONFIG)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  libpointmatcher
  libpointmatcher_ros 
)

catkin_package(
  INCLUDE_DIRS include
)

catkin_package (
	CATKIN_DEPENDS libpointmatcher_ros
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${libpointmatcher_INCLUDE_DIRS}
)

link_directories(
	include
)

add_executable(pcd_processor src/pcd_processing_node.cpp)
add_dependencies(pcd_processor ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_processor ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES})
