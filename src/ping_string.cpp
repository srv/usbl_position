#include "ros/ros.h"
#include "std_msgs/String.h"



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_string");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("ping_string", 1000);
  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "4";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}