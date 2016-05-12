cmake_minimum_required(VERSION 2.8.3)
project(usbl_position)

## Find catkin and any catkin packages
find_package(catkin REQUIRED
	roscpp
	rospy
	std_msgs
	geometry_msgs
	evologics_ros
	tf
	pose_cov_ops
	utils
	cmake_modules
	message_filters
	)

# Dependencies - Eigen:
find_package(Eigen REQUIRED)

catkin_package()

## Build node
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(usbl_position src/usbl_position.cpp)
target_link_libraries(usbl_position ${catkin_LIBRARIES} ${eigen_LIBRARIES})
add_dependencies(usbl_position usbl_position_gencpp evologics_ros_gencpp)

add_executable(usbl_projection src/usbl_projection.cpp)
target_link_libraries(usbl_projection ${catkin_LIBRARIES})
add_dependencies(usbl_projection usbl_position_gencpp evologics_ros_gencpp)

add_executable(plot_path src/plot_path.cpp)
target_link_libraries(plot_path ${catkin_LIBRARIES})
add_dependencies(plot_path usbl_position_gencpp evologics_ros_gencpp)

add_executable(nav_sat_pub src/nav_sat_pub.cpp)
target_link_libraries(nav_sat_pub ${catkin_LIBRARIES})

add_executable(usbl_analysis src/usbl_analysis.cpp)
target_link_libraries(usbl_analysis ${catkin_LIBRARIES})

add_executable(usbl_comm src/usbl_comm.cpp)
target_link_libraries(usbl_comm ${catkin_LIBRARIES})

add_executable(traj2bag src/traj2bag.cpp)
target_link_libraries(traj2bag ${catkin_LIBRARIES})