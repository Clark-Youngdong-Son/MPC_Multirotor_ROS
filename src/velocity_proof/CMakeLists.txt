cmake_minimum_required(VERSION 2.8.3)
project(velocity_proof)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED cmake_modules)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        mavros_msgs
        geometry_msgs
        message_generation
)

catkin_package(
	DEPENDS
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )

set(SRC ${SRC} src/main.cpp)
add_executable (velocity_proof ${SRC} )
target_link_libraries (velocity_proof ${catkin_LIBRARIES} )

