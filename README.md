# usbl_position
The usbl_position node transforms an USBL relative position into a world NED reference frame, to be launched in the ground station.
The usbl_projction node updates an USBL delayed measurement using odometry estimations, to be launched in the AUV.

MELODIC dependencies
sudo apt-get install ros-melodic-pose-cov-ops

In the CmakeLists.txt replace:
auv_msgs for  cola2_msgs
utils for cola2_lib

In usbl_position.cpp replace:
#include "auv_msgs/NavSts.h" for  #include "cola2_msgs/NavSts.h"
#include "utils/ned.h" for #include <cola2_lib/utils/ned.h>

