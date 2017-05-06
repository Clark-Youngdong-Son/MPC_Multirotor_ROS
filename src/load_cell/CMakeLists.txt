cmake_minimum_required(VERSION 2.8.3)
project(load_cell)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(load_cell src/main.cpp)

target_link_libraries(load_cell
  ${catkin_LIBRARIES}
)

