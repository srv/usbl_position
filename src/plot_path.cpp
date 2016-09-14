
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
    marker_id_delayed_ = 1;
    marker_id_raw_ = 1;

    //Topic you want to publish
    pub_raw_ = nhp_.advertise<visualization_msgs::MarkerArray>("plot/modem_raw", 10);
    pub_delayed_ = nhp_.advertise<visualization_msgs::MarkerArray>("plot/modem_delayed", 10);
    pub_ekfodom_ = nhp_.advertise<nav_msgs::Path>("plot/ekf_odom", 10);
    pub_ekfmap_ = nhp_.advertise<nav_msgs::Path>("plot/ekf_map", 10);


    //Topic you want to subscribe
    sub_delayed_ = nh_.subscribe("/sensors/modem_delayed_acoustic", 10, &PlotPath::delayedCallback, this);
    sub_raw_ = nh_.subscribe("/sensors/modem_raw", 10, &PlotPath::rawCallback, this);
    sub_ekfodom_ = nh_.subscribe("/ekf_odom/odometry", 10, &PlotPath::odomCallback, this);
    sub_ekfmap_ = nh_.subscribe("/ekf_map/odometry", 10, &PlotPath::mapCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry& ekfodom)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose = ekfodom.pose.pose;
    pose.header.stamp = ekfodom.header.stamp;
    pose.header.frame_id = "map";

    path_ekfodom_.header.stamp = ekfodom.header.stamp;
    path_ekfodom_.header.frame_id = "map";
    path_ekfodom_.poses.push_back(pose);
    pub_ekfodom_.publish(path_ekfodom_);
  }

  void mapCallback(const nav_msgs::Odometry& ekfmap)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose = ekfmap.pose.pose;
    pose.header.stamp = ekfmap.header.stamp;
    pose.header.frame_id = "map";

    path_ekfmap_.header.stamp = ekfmap.header.stamp;
    path_ekfmap_.header.frame_id = "map";
    path_ekfmap_.poses.push_back(pose);
    pub_ekfmap_.publish(path_ekfmap_);
  }

  void delayedCallback(const geometry_msgs::PoseWithCovarianceStamped& delayed)
  {
    //Transform position from modem_origin reference frame to map
    geometry_msgs::PoseStamped trans_delayed;
    changeFrame(delayed,trans_delayed);

    //Create marker and publish
    visualization_msgs::Marker marker_delayed;
    createMarker(trans_delayed, marker_delayed, marker_id_delayed_, "blue", 0.2);
    marker_id_delayed_++;
    array_delayed_.markers.push_back(marker_delayed);
    pub_delayed_.publish(array_delayed_);
  }

  void rawCallback(const geometry_msgs::PoseWithCovarianceStamped& raw)
  {
    //Transform position from modem_origin reference frame to map
    geometry_msgs::PoseStamped trans_raw;
    changeFrame(raw,trans_raw);

    //Create marker and publish
    visualization_msgs::Marker marker_raw;
    createMarker(trans_raw, marker_raw, marker_id_raw_, "green", 0.5);
    marker_id_raw_++;
    array_raw_.markers.push_back(marker_raw);
    pub_raw_.publish(array_raw_);
  }

  void changeFrame(const geometry_msgs::PoseWithCovarianceStamped& position,
                  geometry_msgs::PoseStamped& transformed_position)
  {
    // TF map-modem_origin
    tf::StampedTransform  map2origin;
    getModemOriginTF(map2origin);

    // TF modem_origin-sparus
    tf::Transform origin2sparus;
    getSparusTF(position, origin2sparus);

    //TF map-sparus
    tf::Transform map2sparus = map2origin * origin2sparus;

    //Build message
    transformed_position.header.stamp = position.header.stamp;
    transformed_position.header.frame_id = "map";
    transformed_position.pose.position.x = map2sparus.getOrigin().x();
    transformed_position.pose.position.y = map2sparus.getOrigin().y();
    transformed_position.pose.position.z = map2sparus.getOrigin().z();
  }

  void getModemOriginTF(tf::StampedTransform& map2origin)
  {
    try
    {
      listener_.waitForTransform("/map", "modem_origin", ros::Time(0), ros::Duration(2));
      listener_.lookupTransform("/map", "modem_origin", ros::Time(0), map2origin);
    }
    catch (tf::TransformException ex){
      ROS_ERROR_STREAM("[" << node_name_ << "]: Received an exception trying to transform a USBL point: " << ex.what());
    }
  }

  void getSparusTF(const geometry_msgs::PoseWithCovarianceStamped& position,
                   tf::Transform& origin2sparus)
  {
    tf::Vector3 origin2sparus_v(position.pose.pose.position.x, position.pose.pose.position.y, position.pose.pose.position.z);
    tf::Quaternion origin2sparus_q(0.0, 0.0, 0.0, 1);
    origin2sparus.setOrigin(origin2sparus_v);
    origin2sparus.setRotation(origin2sparus_q);
  }

  void createMarker(const geometry_msgs::PoseStamped& transformed_position,
                          visualization_msgs::Marker& marker,
                          int& id, string color, float size)
  {
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    // marker.ns = "modem_delayed_fix";
    marker.pose = transformed_position.pose;
    marker.type = 2;  // SPHERE
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    if (color == "red")
    {
      marker.color.r = 1.0;
    }
    else 
    {
      if (color == "green")
      {
        marker.color.g = 1.0;
      }
      else
      {
        marker.color.b = 1.0;
      }
    }
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    marker.id = id;

    marker.action = 0; // ADD
    marker.lifetime = ros::Duration(600);
  }

private:
  ros::NodeHandle nh_; 
  ros::NodeHandle nhp_;

  ros::Publisher pub_raw_;
  ros::Publisher pub_delayed_;
  ros::Publisher pub_ekfodom_;
  ros::Publisher pub_ekfmap_;

  ros::Subscriber sub_delayed_;
  ros::Subscriber sub_raw_;
  ros::Subscriber sub_ekfodom_;
  ros::Subscriber sub_ekfmap_;


  tf::TransformListener listener_;

  nav_msgs::Path path_ekfodom_;
  nav_msgs::Path path_ekfmap_;
  visualization_msgs::MarkerArray array_delayed_;
  visualization_msgs::MarkerArray array_raw_;

  string node_name_;
  int marker_id_delayed_;
  int marker_id_raw_;

};//End of class PlotPath


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "plot_path");

  PlotPath Path;

  ros::spin();
  return 0;
}

