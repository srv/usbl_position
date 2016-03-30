#include "ros/ros.h"
#include "evologics_ros/AcousticModemPayload.h"

using namespace std;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_outmsg");

  ros::NodeHandle n;
  int freq_;
  bool ack_;
  string payload_;

  // Get params
  ros::NodeHandle nhp("~");
  nhp.param("freq", freq_, 10);
  nhp.param("payload", payload_, string("hello"));
  nhp.param("ack", ack_, true);

  ros::Publisher chatter_pub = n.advertise<evologics_ros::AcousticModemPayload>("usbl/im/out", 100);
  ros::Rate loop_rate(freq_);


  while (ros::ok())
  {
    evologics_ros::AcousticModemPayload acoustic_msg;
    acoustic_msg.ack = ack_;
    acoustic_msg.address = 2;
    acoustic_msg.payload = payload_;

    chatter_pub.publish(acoustic_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}