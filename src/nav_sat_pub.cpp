#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <sensor_msgs/Range.h>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_sat_pub");

  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<sensor_msgs::NavSatFix>("/sensors/buoy", 1);
  ros::Publisher pub2 = n.advertise<sensor_msgs::Range>("/sensors/altitude", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sensor_msgs::NavSatFix nav;
    nav.header.stamp = ros::Time::now();
    nav.header.frame_id = "buoy";
    nav.latitude = 39.7180333333;
    nav.longitude = 2.58710666667;
    pub1.publish(nav);

    sensor_msgs::Range alt;
    alt.header.stamp = ros::Time::now();
    alt.header.frame_id = "sparus2";
    alt.range = 1.5;
    pub2.publish(alt);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();

  return 0;
}