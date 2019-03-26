cmake_minimum_required(VERSION 2.8.3)
project(jackal_velodyne)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
)

add_executable(tf_publisher src/tf_publisher.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})

add_executable(get_set_robot_position src/get_set_robot_position.cpp)
target_link_libraries(get_set_robot_position ${catkin_LIBRARIES})