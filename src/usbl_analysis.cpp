#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <cmath>


using namespace std;

class Analysis
{
public:
  Analysis() : nhp_("~")
  {
    // Node name
    node_name_ = ros::this_node::getName();
    ROS_INFO_STREAM("[" << node_name_ << "]: Running");

    //Publishers
    pub_error_modem_raw_ = nhp_.advertise<geometry_msgs::Vector3Stamped>("/error_modem_raw", 10);
    pub_error_ekf_odom_ = nhp_.advertise<geometry_msgs::Vector3Stamped>("/error_ekf_odom", 10);
    pub_error_ekf_map_ = nhp_.advertise<geometry_msgs::Vector3Stamped>("/error_ekf_map", 10);

    // Subscribers
    sub_usbl_ =     nh_.subscribe("/sensors/modem_raw", 1, &Analysis::usblCallback, this);
    sub_ekf_odom_ =  nh_.subscribe("/ekf_odom/odometry", 1, &Analysis::ekfOdomCallback, this);
    sub_ekf_map_ =  nh_.subscribe("/ekf_map/odometry", 1, &Analysis::ekfMapCallback, this);
    sub_gtruth_ =  nh_.subscribe("/sparus/ros_odom_to_pat", 1, &Analysis::gtruthCallback, this);
  }

  void ekfOdomCallback(const nav_msgs::Odometry& ekf_odom)
  {
    ekf_odom_ = ekf_odom;
  }

  void ekfMapCallback(const nav_msgs::Odometry& ekf_map)
  {
    ekf_map_ = ekf_map;
  }

  void gtruthCallback(const nav_msgs::Odometry& gtruth)
  {
    gtruth_ = gtruth;
  }


  void usblCallback(const geometry_msgs::PoseWithCovarianceStamped& modem_raw) 
  {
    // Get vars
    nav_msgs::Odometry gtruth = gtruth_;
    nav_msgs::Odometry ekf_odom = ekf_odom_;
    nav_msgs::Odometry ekf_map = ekf_map_;

    // Modem_raw
    geometry_msgs::Vector3Stamped error_modem_raw;
    getError(modem_raw, gtruth, error_modem_raw);
    pub_error_modem_raw_.publish(error_modem_raw);

    // Ekf_odom
    geometry_msgs::Vector3Stamped error_ekf_odom;
    getError(ekf_odom, gtruth, error_ekf_odom);
    pub_error_ekf_odom_.publish(error_ekf_odom);

    // Ekf_map
    geometry_msgs::Vector3Stamped error_ekf_map;
    getError(ekf_map, gtruth, error_ekf_map);
    pub_error_ekf_map_.publish(error_ekf_map);


  }

  template <typename T>
  void getError(const T& var, const nav_msgs::Odometry& gtruth, geometry_msgs::Vector3Stamped& error)
  {
    error.header = gtruth.header;
    error.vector.x = var.pose.pose.position.x - gtruth.pose.pose.position.x;
    error.vector.y = var.pose.pose.position.y - gtruth.pose.pose.position.y;
    error.vector.z = var.pose.pose.position.z - gtruth.pose.pose.position.z;
  }

private:
  string node_name_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Publisher pub_error_modem_raw_;
  ros::Publisher pub_error_ekf_odom_;
  ros::Publisher pub_error_ekf_map_;

  ros::Subscriber sub_gtruth_;
  ros::Subscriber sub_usbl_;
  ros::Subscriber sub_ekf_odom_;
  ros::Subscriber sub_ekf_map_;

  nav_msgs::Odometry gtruth_;
  nav_msgs::Odometry ekf_odom_;
  nav_msgs::Odometry ekf_map_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_analysis");

  Analysis usbl_analysis;

  ros::spin();
  ros::shutdown();
  return 0;
}