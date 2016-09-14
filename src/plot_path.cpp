#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_listener.h>

using namespace std;


class PlotPath
{
public:
  PlotPath()
  {
    // Node name
    node_name_ = ros::this_node::getName();

    // Topics
    pub_ = nhp_.advertise<nav_msgs::Path>("topic_out", 10);
    sub_ = nh_.subscribe("topic_in", 10, &PlotPath::mapCallback, this);
  }


  void mapCallback(const nav_msgs::Odometry& msg)
  {
    geometry_msgs::PoseStamped position;
    position.pose = msg.pose.pose;
    position.header.stamp = msg.header.stamp;
    position.header.frame_id = "map";

    path_.header.stamp = msg.header.stamp;
    path_.header.frame_id = "map";
    path_.poses.push_back(position);
    pub_.publish(path_);
  }


private:
  ros::NodeHandle nh_; 
  ros::NodeHandle nhp_;

  ros::Publisher pub_;
  ros::Subscriber sub_;


  nav_msgs::Path path_;

  string node_name_;
  string topic_in_;
  string topic_out_;

};//End of class PlotPath


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "plot_path");

  PlotPath Path;

  ros::spin();
  return 0;
}

