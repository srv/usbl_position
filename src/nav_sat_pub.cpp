#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_sat_pub");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::NavSatFix>("/sensors/buoy", 1);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    sensor_msgs::NavSatFix nav;
    nav.header.stamp = ros::Time::now();
    nav.header.frame_id = "buoy";
    nav.latitude = 39.6431;
    nav.longitude = 2.64705;
    pub.publish(nav);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();

  return 0;
}