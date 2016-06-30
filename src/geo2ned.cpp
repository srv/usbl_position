#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <sensor_msgs/NavSatFix.h>
#include "utils/ned.h"


using namespace std;

class Geo2ned
{
public:
  Geo2ned() : nhp_("~")
  {
    // Node name
    node_name_ = ros::this_node::getName();
    ROS_INFO_STREAM("[" << node_name_ << "]: Running");

    //Publishers
    pub_ned_ = nhp_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ned", 10);

    // Subscribers
    sub_geo_ =     nh_.subscribe("/geo", 1, &Geo2ned::geoCb, this);

  }

  void geoCb(const sensor_msgs::NavSatFix::ConstPtr& geo)
  {
  // Read the need origin from parameter server
    double ned_origin_lat, ned_origin_lon;
    if (!getNedOrigin(ned_origin_lat, ned_origin_lon))
    {
        ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the ned origin from the parameter server.");
        return;
    }
    ned_ = new Ned(ned_origin_lat, ned_origin_lon, 0.0);

    // Buoy to NED
    double north, east, depth;
    ned_->geodetic2Ned(geo->latitude, geo->longitude, 0.0, north, east, depth);
    geometry_msgs::PoseWithCovarianceStamped ned;

    // Publish buoy NED
    ned.header.stamp = geo->header.stamp;
    ned.pose.pose.position.x = north;
    ned.pose.pose.position.y = east;
    ned.pose.pose.position.z = depth;
    for (int i = 0; i < 9; ++i) ned.pose.covariance[i] = geo->position_covariance[i];
    pub_ned_.publish(ned);
  }

protected:
  bool getNedOrigin(double& ned_origin_lat, double& ned_origin_lon)
  {
    const string param_ned_origin_lat = "/navigator/ned_origin_lat";
    const string param_ned_origin_lon = "/navigator/ned_origin_lon";

    if (nh_.hasParam(param_ned_origin_lat) && nh_.hasParam(param_ned_origin_lon))
    {
        nh_.getParamCached(param_ned_origin_lat, ned_origin_lat);
        nh_.getParamCached(param_ned_origin_lon, ned_origin_lon);
        return true;
    }
    else
        return false;
  }


private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher pub_ned_;
  ros::Subscriber sub_geo_;


  string node_name_;

  Ned* ned_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "geo2ned");

  Geo2ned geo2ned;

  ros::spin();
  ros::shutdown();

  return 0;
}