cmake_minimum_required(VERSION 2.8.3)
project(usbl_position)

## Find catkin and any catkin packages
find_package(catkin REQUIRED 
	roscpp 
	rospy 
	std_msgs 
	geometry_msgs
	evologics_ros
	)

catkin_package(CATKIN_DEPENDS 
	message_runtime
	rospy
  	roscpp
  	std_msgs
  	std_srvs
  	geometry_msgs
	evologics_ros
  	)


## Build node
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(plot_position_node src/plot_position_node.cpp)
target_link_libraries(plot_position_node ${catkin_LIBRARIES})
add_dependencies(plot_position_node usbl_position_gencpp)

add_executable(plot_long src/plot_long.cpp)
target_link_libraries(plot_long ${catkin_LIBRARIES})
add_dependencies(plot_long usbl_position_gencpp)

add_executable(plot_path src/plot_path.cpp)
target_link_libraries(plot_path ${catkin_LIBRARIES})
add_dependencies(plot_path usbl_position_gencpp)

add_executable(usbl_projection src/usbl_projection.cpp)
target_link_libraries(usbl_projection ${catkin_LIBRARIES})
add_dependencies(usbl_projection usbl_position_gencpp)

add_executable(usbl_position_node src/usbl_position_node.cpp)
target_link_libraries(usbl_position_node ${catkin_LIBRARIES})
add_dependencies(usbl_position_node usbl_position_gencpp)

add_executable(ping_outmsg src/ping_outmsg.cpp)
target_link_libraries(ping_outmsg ${catkin_LIBRARIES})
add_dependencies(ping_outmsg usbl_position_gencpp)

add_executable(ping_string src/ping_string.cpp)
target_link_libraries(ping_string ${catkin_LIBRARIES})
add_dependencies(ping_string usbl_position_gencpp)