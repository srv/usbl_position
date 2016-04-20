
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"

#include <tf/transform_listener.h>

using namespace std;


class PlotPath
{
public:
  PlotPath()
  {
    // Node name
    node_name_ = ros::this_node::getName();


    //Topic you want to publish
    path_pub = nhp_.advertise<nav_msgs::Path>("sensors/modem_raw_path", 10);
    // cov_pub = n.advertise<visualization_msgs::Marker>(node_name_+"/path" + "/cov", 10);

    //Topic you want to subscribe
    pose_sub = nh_.subscribe("sensors/modem_raw", 10, &PlotPath::poseCallback, this);
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& position)
  {
    // TF map-modem_origin
    tf::StampedTransform  map2origin;
    try
    {
      listener_.waitForTransform("/map", "modem_origin", ros::Time(0), ros::Duration(2));
      listener_.lookupTransform("/map", "modem_origin", ros::Time(0), map2origin);
    }
    catch (tf::TransformException ex){
      ROS_ERROR_STREAM("Received an exception trying to transform a USBL point: " << ex.what());
    }

    //TF modem_origin-sparus
    tf::Transform  origin2sparus;
    tf::Vector3 origin2sparus_v(position.pose.pose.position.x, position.pose.pose.position.y, position.pose.pose.position.z);
    tf::Quaternion origin2sparus_q(0.0, 0.0, 0.0, 1);
    origin2sparus.setOrigin(origin2sparus_v);
    origin2sparus.setRotation(origin2sparus_q);

    //TF map-sparus
    tf::Transform  map2sparus = map2origin * origin2sparus;

    geometry_msgs::PoseStamped p;
    p.pose.position.x = map2sparus.getOrigin().x();
    p.pose.position.y = map2sparus.getOrigin().y();
    p.pose.position.z = map2sparus.getOrigin().z();

    path.header.stamp = position.header.stamp;
    path.header.frame_id = "map";
    path.poses.push_back(p);
    path_pub.publish(path);

    // visualization_msgs::Marker marker;
    // marker.header.stamp = position.header.stamp;
    // marker.header.frame_id = "map";
    // marker.ns = node_name_+"/path" + "_pose_cov";
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
  ros::NodeHandle nh_; 
  ros::NodeHandle nhp_;
  ros::Publisher path_pub;
  ros::Publisher cov_pub;
  ros::Subscriber pose_sub;

  tf::TransformListener listener_;

  nav_msgs::Path path;
  int id_;

  string node_name_;

};//End of class PlotPath


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "plot_path");

  PlotPath SAPObject;

  ros::spin();
  return 0;
}

