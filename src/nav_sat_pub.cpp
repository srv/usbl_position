#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <sensor_msgs/Range.h>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_sat_pub");

  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<sensor_msgs::NavSatFix>("/sensors/buoy", 1);
  ros::Rate loop_rate(5);

  // Publish the gps over the ned origin
  double ned_origin_lat, ned_origin_lon;
  bool ned_catched = false;

  while (ros::ok())
  {
    if (!ned_catched)
    {
      if (n.hasParam("/navigator/ned_origin_lat") && n.hasParam("/navigator/ned_origin_lon"))
      {
        n.getParam("/navigator/ned_origin_lat", ned_origin_lat);
        n.getParam("/navigator/ned_origin_lon", ned_origin_lon);
        ned_catched = true;
      }
    }

    sensor_msgs::NavSatFix nav;
    nav.header.stamp = ros::Time::now();
    nav.header.frame_id = "buoy";
    nav.latitude = 39.6431;
    nav.longitude = 2.64712;
    pub1.publish(nav);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();

  return 0;
}