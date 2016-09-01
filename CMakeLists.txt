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
	evologics_ros
	safety
	tf
	pose_cov_ops
	utils
	cmake_modules
	message_filters
	)

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

catkin_package()

## Build node
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(usbl_position src/usbl_position.cpp)
target_link_libraries(usbl_position ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(usbl_position evologics_ros_gencpp)

add_executable(usbl_projection src/usbl_projection.cpp)
target_link_libraries(usbl_projection ${catkin_LIBRARIES})
add_dependencies(usbl_projection evologics_ros_gencpp)

add_executable(plot_path src/plot_path.cpp)
target_link_libraries(plot_path ${catkin_LIBRARIES})
add_dependencies(plot_path evologics_ros_gencpp)

add_executable(nav_sat_pub src/nav_sat_pub.cpp)
target_link_libraries(nav_sat_pub ${catkin_LIBRARIES})

add_executable(usbl_analysis src/usbl_analysis.cpp)
target_link_libraries(usbl_analysis ${catkin_LIBRARIES})

add_executable(usbl_comm src/usbl_comm.cpp)
target_link_libraries(usbl_comm ${catkin_LIBRARIES})
add_dependencies(usbl_comm evologics_ros_gencpp)

add_executable(traj2bag src/traj2bag.cpp)
target_link_libraries(traj2bag ${catkin_LIBRARIES})
add_dependencies(traj2bag safety_gencpp)

add_executable(geo2ned src/geo2ned.cpp)
target_link_libraries(geo2ned ${catkin_LIBRARIES})