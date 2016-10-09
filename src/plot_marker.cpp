#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_listener.h>

using namespace std;


class PlotMarker
{
public:
  PlotMarker(): nhp_("~")
  {
    // Get params
    nhp_.param("r", red_, 0.0);
    nhp_.param("g", green_, 0.0);
    nhp_.param("b", blue_, 0.0);
    nhp_.param("a", alpha_, 1.0);
    nhp_.param("size", size_, 0.5);
    nhp_.param("duration", duration_, 30);
    nhp_.param("frame_id", frame_id_, string(""));

    // Node name
    node_name_ = ros::this_node::getName();
    marker_id_ = 1;

    // Topics
    pub_ = nhp_.advertise<visualization_msgs::MarkerArray>("/topic_out", 10);
    sub_ = nh_.subscribe("/topic_in", 10, &PlotMarker::callback, this);
  }


  void callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    //Transform position from modem_origin reference frame to map
    geometry_msgs::PoseStamped trans;
    changeFrame(msg,trans);

    //Create marker and publish
    visualization_msgs::Marker marker;
    createMarker(trans, marker);
    
    array_.markers.push_back(marker);
    pub_.publish(array_);
  }


  void changeFrame(const geometry_msgs::PoseWithCovarianceStamped& position,
                  geometry_msgs::PoseStamped& trans)
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
    trans.header.stamp = position.header.stamp;
    trans.header.frame_id = "map";
    trans.pose.position.x = map2sparus.getOrigin().x();
    trans.pose.position.y = map2sparus.getOrigin().y();
    trans.pose.position.z = map2sparus.getOrigin().z();
  }

  void getModemOriginTF(tf::StampedTransform& map2origin)
  {
    try
    {
      listener_.waitForTransform("/map", frame_id_, ros::Time(0), ros::Duration(6));
      listener_.lookupTransform("/map", frame_id_, ros::Time(0), map2origin);
    }
    catch (tf::TransformException ex){
      ROS_ERROR_STREAM("[" << node_name_ << "]: Received an exception trying to transform marker position: " << ex.what());
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

  void createMarker(const geometry_msgs::PoseStamped& trans,
                          visualization_msgs::Marker& marker)
  {
    marker.header.frame_id = "map";
    marker.pose = trans.pose;
    marker.type = 2;  // SPHERE
    marker.color.r = red_;
    marker.color.g = green_;
    marker.color.b = blue_;
    marker.color.a = alpha_;
    marker.scale.x = size_;
    marker.scale.y = size_;
    marker.scale.z = size_;

    marker.id = marker_id_;
    marker_id_++;

    marker.action = 0; // ADD
    marker.lifetime = ros::Duration(600);
  }

private:
  ros::NodeHandle nh_; 
  ros::NodeHandle nhp_;

  ros::Publisher pub_;
  ros::Subscriber sub_;

  tf::TransformListener listener_;

  visualization_msgs::MarkerArray array_;

  string node_name_;
  double red_;
  double green_;
  double blue_;
  double alpha_;
  double size_;
  int duration_;
  int marker_id_;
  string frame_id_;

};//End of class PlotPath


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "plot_marker");

  PlotMarker Marker;

  ros::spin();
  return 0;
}
