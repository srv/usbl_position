#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "evologics_ros/AcousticModemPayload.h"


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>


using namespace std;

class Communication
{
public:
  Communication()
  {
    // Node name
    node_name_ = ros::this_node::getName();
    ROS_INFO_STREAM("[" << node_name_ << "]: Running");

    //Publishers
    pub_modem_in_ = nhp_.advertise<evologics_ros::AcousticModemPayload>("/modem/im/in", 10);
    pub_usbl_in_ = nhp_.advertise<evologics_ros::AcousticModemPayload>("/usbl/im/in", 10);

    //Subscribers
    sub_modem_in_ = nh_.subscribe("/usbl/im/out", 1, &Communication::modemInCb, this);
    sub_usbl_in_ = nh_.subscribe("/modem/im/out", 1, &Communication::usblInCb, this);

    //
  }

  void modemInCb(const evologics_ros::AcousticModemPayload& out)
  {
    // ROS_INFO_STREAM("/usbl/im/out --> /modem/im/in:  " << out.payload);
    pub_modem_in_.publish(out);
  }

  void usblInCb(const evologics_ros::AcousticModemPayload& out)
  {
    // ROS_INFO_STREAM("/modem/im/out --> /usbl/im/in:  " << out.payload);
    pub_usbl_in_.publish(out);
  }

private:
  string node_name_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Publisher pub_modem_in_;
  ros::Publisher pub_usbl_in_;
  ros::Subscriber sub_modem_in_;
  ros::Subscriber sub_usbl_in_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_comm");

  Communication usbl_comm;

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}