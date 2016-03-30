#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <algorithm>

#include <tf/transform_listener.h>

using namespace std;

class positioning
{
public:
  vector<double> timestamps;
  vector<double> hist_x;
  vector<double> hist_y;
  vector<double> hist_z;

  bool ekf_init;
  positioning() : ekf_init(false)
  {
    //Subscribers
    sub_usbl = n.subscribe("/sensors/modem", 1, &positioning::usblCallback, this);
    sub_odom = n.subscribe("/ekf_odom/odometry", 1, &positioning::odomCallback, this);

    //Publishers
    pub_modem_position = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("sensors/modem_update", 1); 
  }

  void odomCallback(const nav_msgs::Odometry& odom)
  {
    mutex_.lock();
    timestamps.push_back(odom.header.stamp.toSec());
    hist_x.push_back(odom.pose.pose.position.x);
    hist_y.push_back(odom.pose.pose.position.y);
    hist_z.push_back(odom.pose.pose.position.z);
    ekf_init=true;

    if (timestamps.size() > 1000) 
    {
      timestamps.erase(timestamps.begin());
      hist_x.erase(hist_x.begin());
      hist_y.erase(hist_y.begin());
      hist_z.erase(hist_z.begin());
    }
    mutex_.unlock();
  }

  void usblCallback(const geometry_msgs::PoseWithCovarianceStamped& usbl)
  {
    // Wait to odometry msgs to start
    if (ekf_init==false) return; 

    // Find timestamp
    mutex_.lock();
    double instant = usbl.header.stamp.toSec();
    size_t i;
    for (i = timestamps.size() - 1; i > 0; i--) // TODO
    {
      if (instant > timestamps[i]) break;
    }
    if (i == 0) {
      mutex_.unlock();
      return;
    } 

    //Odometry evolution since the measurement instant
    double delta_odom_x = hist_x.back() - hist_x[i];
    double delta_odom_y = hist_y.back() - hist_y[i];
    double delta_odom_z = hist_z.back() - hist_z[i];

    // Transform from /map to /modem_origin

    //Updated position
    geometry_msgs::PoseWithCovarianceStamped position;
    position.header.stamp = ros::Time::now();
    position.header.frame_id = usbl.header.frame_id;
    position.pose.pose.position.x = usbl.pose.pose.position.x + delta_odom_x;
    position.pose.pose.position.y = usbl.pose.pose.position.y + delta_odom_y;
    position.pose.pose.position.z = usbl.pose.pose.position.z + delta_odom_z;
    position.pose.covariance = usbl.pose.covariance; //TODO: fusion covariances, odom and usbl
    pub_modem_position.publish(position);

    //Reshape historial
    timestamps.erase(timestamps.begin() , timestamps.begin() + i);
    hist_x.erase(hist_x.begin() , hist_x.begin() + i);
    hist_y.erase(hist_y.begin() , hist_y.begin() + i);
    hist_z.erase(hist_z.begin() , hist_z.begin() + i);
    mutex_.unlock();
  }

private:
  ros::NodeHandle n; 
  ros::Subscriber sub_usbl;
  ros::Subscriber sub_odom;
  ros::Publisher pub_modem_position;
  boost::mutex mutex_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_position_node");

  ROS_INFO("Initialize usbl positioning class");
  positioning usbl_positioning;

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}