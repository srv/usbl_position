
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"

visualization_msgs::Marker points, line_strip;


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    // USBL Marker
    points.header.frame_id = line_strip.header.frame_id = "/usbl";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    points.id = 0; 
    points.type = visualization_msgs::Marker::POINTS;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.color.b = 1.0;
    line_strip.color.a = 0.6;


    //Topic you want to publish
    marker_pub = n.advertise<visualization_msgs::Marker>("usbl_marker", 10);

    //Topic you want to subscribe
    point_sub = n.subscribe("usbl/modem_position", 10, &SubscribeAndPublish::posCallback, this);
  }

  void posCallback(const geometry_msgs::PoseWithCovarianceStamped& position)
  {
    points.scale.x = 0.2 * position.pose.covariance[0];
    points.scale.y = 0.2 * position.pose.covariance[7];


    geometry_msgs::Point p;
    p.x = position.pose.pose.position.x;
    p.y = position.pose.pose.position.y;
    p.z = position.pose.pose.position.z;
    points.points.push_back(p);
    ROS_INFO("Position: %f, %f, %f", p.x, p.y, p.z);
    marker_pub.publish(points);
  }


private:
  ros::NodeHandle n; 
  ros::Publisher marker_pub;
  ros::Subscriber point_sub;

};//End of class SubscribeAndPublish


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "usbl_plot");

  SubscribeAndPublish SAPObject;


  ros::spin();

  return 0;
}

