cmake_minimum_required(VERSION 2.8.3)
project(mpc_trajectory_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc_trajectory_publisher
#  CATKIN_DEPENDS geometry_msgs mavlink_msgs nav_msgs roscpp
#  DEPENDS system_lib
)

set(SRC ${SRC} src/main.cpp)

add_executable (mpc_trajectory_publisher ${SRC} )
target_link_libraries (mpc_trajectory_publisher ${catkin_LIBRARIES} )
