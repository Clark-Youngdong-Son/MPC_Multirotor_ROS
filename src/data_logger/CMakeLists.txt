cmake_minimum_required(VERSION 2.8.3)
project(data_logger CXX C)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS 
		roscpp 
		rospy 
		std_msgs 
		sensor_msgs
		geometry_msgs
		image_transport
		cv_bridge
)

catkin_package(
		CATKIN_DEPENDS
		message_runtime
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp)
set(SRC ${SRC} src/data_logger.cpp)
set(SRC ${SRC} src/message_handler.cpp)
set(SRC ${SRC} include/data_logger.h)
set(SRC ${SRC} include/message_handler.h)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )

add_executable (data_logger ${SRC})
target_link_libraries (data_logger ${catkin_LIBRARIES} ${LIBS})

add_dependencies(data_logger mavros_msgs_generate_messages_cpp)
