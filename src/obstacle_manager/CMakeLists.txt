cmake_minimum_required(VERSION 2.8.3)
project(obstacle_manager)

set(CMAKE_CXX_FLAGS "-std=gnu++11")
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  random_numbers
  roscpp
)

catkin_package(
	DEPENDS Eigen
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)
include_directories(${includeDirs})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

set(SRC ${SRC} src/main.cpp)
set(SRC ${SRC} src/ObstacleManager.cpp)
set(SRC ${SRC} include/ObstacleManager.h)

add_executable(obstacle_manager ${SRC})
target_link_libraries(obstacle_manager ${catkin_LIBRARIES})
target_link_libraries(obstacle_manager ${Eigen_LIBRARIES})
