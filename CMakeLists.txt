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
add_dependencies(usbl_position usbl_position_gencpp)

add_executable(usbl_projection src/usbl_projection.cpp)
target_link_libraries(usbl_projection ${catkin_LIBRARIES})
add_dependencies(usbl_projection usbl_position_gencpp)


add_executable(plot_path src/plot_path.cpp)
target_link_libraries(plot_path ${catkin_LIBRARIES})
add_dependencies(plot_path usbl_position_gencpp)

