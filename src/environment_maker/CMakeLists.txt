cmake_minimum_required(VERSION 2.8.3)
project(environment_maker)

find_package(catkin REQUIRED COMPONENTS
		roscpp
)

catkin_package(
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )

add_executable (environment_maker ${SRC} )
target_link_libraries (environment_maker ${catkin_LIBRARIES} )
