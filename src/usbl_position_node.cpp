#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "evologics_ros/AcousticModemUSBLLONG.h"
#include "evologics_ros/AcousticModemUSBLANGLES.h"
#include <cmath>


//TODO: Which is the best place to locate varible declarations?
double depth = 1;
double sigma_depth = 0.1;
double MIN_DEPTH = 0.1;

class positioning
{
public:
  positioning()
  {
    //Subscribers
    sub_usbllong = n.subscribe("usbl/usbllong", 10, &positioning::longCallback, this);
    sub_usblangles = n.subscribe("usbl/usblangles", 10, &positioning::anglesCallback, this);
    sub_depth = n.subscribe("sensors/depth_raw", 10, &positioning::depthCallback, this);

    //Publishers
    pub_modem_position = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("usbl/modem_position", 10); 
    ROS_INFO("Advertise");

  }

  void longCallback(const evologics_ros::AcousticModemUSBLLONG& usbllong)
  {
    // Modem position
    geometry_msgs::PoseWithCovarianceStamped position;
    position.header.frame_id = "/usbl";
    position.header.stamp = usbllong.header.stamp; //menos propagation time
    position.pose.pose.position.x = (float)usbllong.N;
    position.pose.pose.position.y = (float)usbllong.E;
    position.pose.pose.position.z = (float)usbllong.U;
    position.pose.covariance[0] = (float)pow(usbllong.accuracy,2);
    position.pose.covariance[7] = (float)pow(usbllong.accuracy,2);
    position.pose.covariance[13] = (float)pow(usbllong.accuracy,2);
    pub_modem_position.publish(position);
  }

  void anglesCallback(const evologics_ros::AcousticModemUSBLANGLES& usblangles)
  {
    if (depth >= MIN_DEPTH)
    {
      //ROS_INFO_STREAM("Bearing: " << usblangles.bearing << "\t Elevation: " << usblangles.elevation << "\t Accuracy: " << usblangles.accuracy );
      
      double x, y, z;
      spheric2cartesian(usblangles.bearing, usblangles.elevation, depth, x, y, z);
      
      double sigma_x, sigma_y, sigma_z;
      getCovarianceAngles(usblangles.bearing, usblangles.elevation, depth, usblangles.accuracy, sigma_x, sigma_y, sigma_z);

      // Modem ray
      geometry_msgs::PoseWithCovarianceStamped position;
      position.header.frame_id = "/usbl";
      position.header.stamp = usblangles.header.stamp;
      position.pose.pose.position.x = x;
      position.pose.pose.position.y = y;
      position.pose.pose.position.z = z;
      position.pose.covariance[0] = sigma_x;
      position.pose.covariance[7] = sigma_y;
      position.pose.covariance[14] = sigma_z;
      pub_modem_position.publish(position);
    }
    else
      ROS_WARN("Not enough depth to estimate USBLangles position");
  }


  void depthCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
  {
    depth = pose.pose.pose.position.z;
    sigma_depth = pose.pose.covariance[14];
  }

  void spheric2cartesian(const double bearing, 
                         const double elevation, 
                         const double depth, 
                         double& x, 
                         double& y, 
                         double& z)
  {
    //x = depth * tan(elevation) * cos(bearing);
    //y = depth * tan(elevation) * sin(bearing);
    x = depth * sin(bearing) / tan(elevation);
    y = depth * cos(bearing) / tan(elevation);
    z = depth; //TODO: Integrate depth of the USBL
  }

  double getCovarianceAngles(const double bearing, 
                             const double elevation, 
                             const double depth, 
                             const double accuracy, 
                             double& sigma_x, 
                             double& sigma_y,
                             double& sigma_z)
  {
    //Extreme coordinates of the ellipse
    double x_11, y_11, z_11;
    double x_12, y_12, z_12;
    double x_21, y_21, z_21;
    double x_22, y_22, z_22;

    spheric2cartesian(bearing           , elevation + accuracy, depth, x_11, y_11, z_11);
    spheric2cartesian(bearing           , elevation - accuracy, depth, x_12, y_12, z_12);
    spheric2cartesian(bearing + accuracy, elevation           , depth, x_21, y_21, z_21);
    spheric2cartesian(bearing - accuracy, elevation           , depth, x_22, y_22, z_22);

    //Ellipse axis
    double axis_1 = sqrt(pow(x_12-x_11,2)+pow(y_12-y_11,2));
    double axis_2 = sqrt(pow(x_22-x_21,2)+pow(y_22-y_21,2));
    double a;
    double b;

    if (axis_1>axis_2)
    {
      a = axis_1;
      b = axis_2;
    }
    else
    {
      a = axis_2;
      b = axis_1;
    }

    sigma_x = 2 * sqrt(pow(a * sin(bearing),2) + pow(b * cos(bearing),2));
    sigma_y = 2 * sqrt(pow(a * cos(bearing),2) + pow(b * sin(bearing),2));
    //sigma_x = 2 * sqrt(pow(a * cos(bearing),2) + pow(b * sin(bearing),2));
    //sigma_y = 2 * sqrt(pow(a * sin(bearing),2) + pow(b * cos(bearing),2));
    sigma_z = sigma_depth;
  }

private:
  ros::NodeHandle n; 
  ros::Subscriber sub_usbllong;
  ros::Subscriber sub_usblangles;
  ros::Subscriber sub_depth;
  ros::Publisher pub_modem_position;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_position_node");

  ROS_INFO("Initialize usbl positioning class");
  positioning usbl_positioning;

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}