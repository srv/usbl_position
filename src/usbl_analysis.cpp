#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>


using namespace std;

class Analysis
{
public:
  Analysis(ros::NodeHandle nh) : nh_(nh), nhp_("~")
  {
    // Node name
    node_name_ = ros::this_node::getName();
    ROS_INFO_STREAM("[" << node_name_ << "]: Running");

    //Publishers
    // pub_modem_ = nhp_.advertise<geometry_msgs::PoseWithCovarianceStamped>("modem_delayed", 10);
  }

  void modemRawCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& modem_raw,
                  const nav_msgs::Odometry::ConstPtr& ekf_map,
                  const nav_msgs::Odometry::ConstPtr& gtruth)
  {
    // Modem_raw - GT distance
    double dist_raw_gt;
    getDistance(modem_raw, gtruth, dist_raw_gt);

    // Ekf_map - GT distance
    double dist_ekfmap_gt;
    getDistance(ekf_map, gtruth, dist_ekfmap_gt);

    ROS_INFO_STREAM("dist_raw_gt: " << dist_raw_gt);
    ROS_INFO_STREAM("dist_ekfmap_gt: " << dist_ekfmap_gt);

  }

  template <typename T1, typename T2>
  void getDistance(const T1& var1, const T2& var2, double distance)
  {
    double x1 = var1->pose.pose.position.x;
    double y1 = var1->pose.pose.position.y;
    double x2 = var2->pose.pose.position.x;
    double y2 = var2->pose.pose.position.y;
    ROS_INFO_STREAM("VAR1: x = " << x1 << "  /  y = " << y1);
    ROS_INFO_STREAM("VAR2: x = " << x2 << "  /  y = " << y2);

    distance = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
  }

private:
  string node_name_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  //ros::Publisher pub_modem_;


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_analysis");

  ros::NodeHandle nh;
  Analysis usbl_analysis(nh);

  // SYNC_1 (Modem Raw) //
      // Message sync
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_modem_raw(nh, "/sensors/modem_raw", 20);
  message_filters::Subscriber<nav_msgs::Odometry> sub_ekf_map(nh, "/ekf_map/odometry", 50);
  message_filters::Subscriber<nav_msgs::Odometry> sub_gtruth(nh, "/sparus/pat_to_ros_odom", 50);
      // Define syncs
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped,
                                                          nav_msgs::Odometry,
                                                          nav_msgs::Odometry> 
                                                          sync_pool_1;
  message_filters::Synchronizer<sync_pool_1> sync_1(sync_pool_1(50), sub_modem_raw, sub_ekf_map, sub_gtruth);
  sync_1.registerCallback(boost::bind(&Analysis::modemRawCb, &usbl_analysis, _1, _2, _3));


  ros::spin();
  ros::shutdown();
  return 0;
}