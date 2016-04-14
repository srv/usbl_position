
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"

using namespace std;


class plotCovariance
{
public:
  plotCovariance()
  {
    // Get params
    ros::NodeHandle nhp("~");
    nhp.param("topic_name", name_, string("/ekf_odom/"));
    nhp.param("sensor", sensor_, string("/modem"));
    nhp.param("id", id_, 2);


    //Topic you want to publish
    path_pub = n.advertise<nav_msgs::Path>(sensor_ + "/path", 10);
    // cov_pub = n.advertise<visualization_msgs::Marker>(sensor_ + "/cov", 10);

    //Topic you want to subscribe
    pose_sub = n.subscribe(name_, 10, &plotCovariance::poseCallback, this);
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& position)
  {
    geometry_msgs::PoseStamped p;

    p.pose.position.x = position.pose.pose.position.x;
    p.pose.position.y = position.pose.pose.position.y;
    p.pose.position.z = position.pose.pose.position.z;

    path.header.stamp = position.header.stamp;
    path.header.frame_id = "map";
    path.poses.push_back(p);
    path_pub.publish(path);

    // visualization_msgs::Marker marker;
    // marker.header.stamp = position.header.stamp;
    // marker.header.frame_id = "map";
    // marker.ns = sensor_ + "_pose_cov";
    // marker.id = id_;
    // marker.type = 2;  // SPHERE
    // marker.action = 0; // ADD
    // marker.pose.position.x = position.pose.pose.position.x;
    // marker.pose.position.y = position.pose.pose.position.y;
    // marker.pose.position.z = position.pose.pose.position.z;
    // marker.scale.x = position.pose.covariance[0];
    // marker.scale.y = position.pose.covariance[7];
    // marker.scale.z = position.pose.covariance[14];
    // marker.color.r = 1.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // marker.color.a = 0.5;
    // cov_pub.publish(marker);
  }


private:
  ros::NodeHandle n; 
  ros::Publisher path_pub;
  ros::Publisher cov_pub;
  ros::Subscriber pose_sub;

  nav_msgs::Path path;
  string name_, sensor_;
  int id_;


};//End of class SubscribeAndPublish


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "plot_cov");

  plotCovariance plotCov;

  ros::spin();
  return 0;
}

