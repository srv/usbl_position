cmake_minimum_required(VERSION 2.8.3)
project(usbl_position)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin and any catkin packages
find_package(catkin REQUIRED
	roscpp
	rospy
	std_msgs
	geometry_msgs
	evologics_ros_sync
  auv_msgs
	tf
	pose_cov_ops
	utils
	cmake_modules
	message_filters
	)

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  rospy
  roscpp
  evologics_ros_sync
  auv_msgs)

## Build node
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(usbl_position src/usbl_position.cpp)
target_link_libraries(usbl_position ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(usbl_position ${catkin_EXPORTED_TARGETS})

add_executable(usbl_projection src/usbl_projection.cpp)
target_link_libraries(usbl_projection ${catkin_LIBRARIES})
add_dependencies(usbl_projection ${catkin_EXPORTED_TARGETS})
