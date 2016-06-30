
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
    marker_id_ = 1;

    //Topic you want to publish
    raw_path_pub_ = nhp_.advertise<nav_msgs::Path>("plot/modem_raw_path", 10);
    delayed_path_pub_ = nhp_.advertise<nav_msgs::Path>("plot/modem_delayed_path", 10);
    delayed_marker_pub_ = nhp_.advertise<visualization_msgs::MarkerArray>("plot/modem_delayed_array", 10);
    odom_pub_ = nhp_.advertise<nav_msgs::Odometry>("plot/ekf_odom", 10);


    //Topic you want to subscribe
    delayed_sub_ = nh_.subscribe("/sensors/modem_delayed_acoustic", 10, &PlotPath::delayedCallback, this);
    raw_sub_ = nh_.subscribe("/sensors/modem_raw", 10, &PlotPath::rawCallback, this);
    odom_sub_ = nh_.subscribe("/ekf_odom/odometry", 10, &PlotPath::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry& odom)
  {
    nav_msgs::Odometry transformed_odom = odom;
    transformed_odom.header.frame_id = "map";
    odom_pub_.publish(transformed_odom);
  }

  void delayedCallback(const geometry_msgs::PoseWithCovarianceStamped& delayed)
  {
    visualization_msgs::Marker marker;
    createMarker(delayed, marker);
    array_.markers.push_back(marker);

    delayed_marker_pub_.publish(array_);
  }

  // void delayedCallback(const geometry_msgs::PoseWithCovarianceStamped& delayed)
  // {
  //   //Transform position from modem_origin reference frame to map
  //   geometry_msgs::PoseStamped transformed_position;
  //   changeFrame(delayed,transformed_position);


  //   delayed_path_.header.stamp = delayed.header.stamp;
  //   delayed_path_.header.frame_id = "map";

  //   //Save positions and publish path
  //   delayed_path_.poses.push_back(transformed_position);
  //   delayed_path_pub_.publish(delayed_path_);

  //   // Marker
    
  //   createMarker(transformed_position);
  // }

  void rawCallback(const geometry_msgs::PoseWithCovarianceStamped& raw)
  {
    //Transform position from modem_origin reference frame to map
    geometry_msgs::PoseStamped transformed_position;
    changeFrame(raw,transformed_position);


    raw_path_.header.stamp = raw.header.stamp;
    raw_path_.header.frame_id = "map";

    //Save positions and publish
    raw_path_.poses.push_back(transformed_position);
    raw_path_pub_.publish(raw_path_);
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

  void changeFrame(const geometry_msgs::PoseWithCovarianceStamped& position,
                  geometry_msgs::PoseStamped& transformed_position)
  {
    // TF map-modem_origin
    tf::StampedTransform  map2origin;
    getModemOriginTF(map2origin);

    // TF modem_origin-sparus
    tf::Transform  origin2sparus;
    getSparusTF(position, origin2sparus);

    //TF map-sparus
    tf::Transform map2sparus = map2origin * origin2sparus;

    //Build message
    transformed_position.pose.position.x = map2sparus.getOrigin().x();
    transformed_position.pose.position.y = map2sparus.getOrigin().y();
    transformed_position.pose.position.z = map2sparus.getOrigin().z();
  }

  void createMarker(const geometry_msgs::PoseWithCovarianceStamped& transformed_position,
                    visualization_msgs::Marker& marker)
  {
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    // marker.ns = "modem_delayed_fix";
    marker.id = marker_id_;
    marker.pose = transformed_position.pose.pose;
    marker.type = 2;  // SPHERE
    marker.action = 0; // ADD
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.lifetime = ros::Duration(600);
    marker_id_++;
  }

private:
  ros::NodeHandle nh_; 
  ros::NodeHandle nhp_;

  ros::Publisher raw_path_pub_;
  ros::Publisher delayed_path_pub_;
  ros::Publisher delayed_marker_pub_;
  ros::Publisher odom_pub_;

  ros::Subscriber delayed_sub_;
  ros::Subscriber raw_sub_;
  ros::Subscriber odom_sub_;

  tf::TransformListener listener_;

  nav_msgs::Path delayed_path_;
  nav_msgs::Path raw_path_;
  int id_;
  visualization_msgs::MarkerArray array_;

  string node_name_;
  int marker_id_;

};//End of class PlotPath


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "plot_path");

  PlotPath SAPObject;

  ros::spin();
  return 0;
}

