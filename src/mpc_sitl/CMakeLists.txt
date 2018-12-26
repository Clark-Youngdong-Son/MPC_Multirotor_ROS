cmake_minimum_required(VERSION 2.8.3)
project(mpc_sitl)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
		roscpp
		mavros_msgs
		nav_msgs
		geometry_msgs
)
#find_package( OpenMP REQUIRED)
#if(OPENMP_FOUND)
#message("OPENMP FOUND")
#set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set (CMAKE_EXR_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()
set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp")

catkin_package(
		DEPENDS Eigen
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp)
set(SRC ${SRC} src/Mpc.cpp)
set(SRC ${SRC} src/MPCDataLogger.cpp)
set(SRC ${SRC} include/Mpc.h)
set(SRC ${SRC} include/MPCDataLogger.h)
set(SRC ${SRC} include/Params.h)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

add_executable (mpc_sitl ${SRC} )
target_link_libraries (mpc_sitl ${catkin_LIBRARIES} )
target_link_libraries (mpc_sitl ${Eigen_LIBRARIES} )
