#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "auv_msgs/NavSts.h"
#include "evologics_ros/AcousticModemUSBLLONG.h"
#include "evologics_ros/AcousticModemUSBLANGLES.h"
#include <cmath>

#include <sensor_msgs/NavSatFix.h>
#include "utils/ned.h"
#include <pose_cov_ops/pose_cov_ops.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

using namespace std;

class Position
{
public:
  Position(ros::NodeHandle nh) : nh_(nh), nhp_("~"), buoy2usbl_catched_(false)
  {
    ROS_INFO_STREAM("[" << node_name_ << "]: Running");

    // Node name
    node_name_ = ros::this_node::getName();

    // Get Params
    nhp_.param("frames/map", frame_map_, string("map"));
    nhp_.param("frames/sensors/usbl", frame_usbl_, string("usbl"));
    nhp_.param("frames/sensors/buoy", frame_buoy_, string("buoy"));

    //Publishers
    pub_modem_ = nhp_.advertise<geometry_msgs::PoseWithCovarianceStamped>("modem_delayed", 10);

    modem_on_ = nh_.serviceClient<std_srvs::Empty>("/modem_on");
    while (!modem_on_.waitForExistence()) {
      ROS_INFO_STREAM_ONCE("[" << node_name_ << "]: Waiting for /modem_on service to be available.");
    }
    ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("[" << node_name_ << "]: Calling /modem_on service...");
    std_srvs::Empty srv;
    modem_on_.call(srv);

    ROS_INFO_STREAM("[" << node_name_ << "]: /modem_on service called!");
  }

  void usbllongCallback(const evologics_ros::AcousticModemUSBLLONG::ConstPtr& usbllong,
                        const sensor_msgs::NavSatFix::ConstPtr& nav)
  {
    // Get the static transform from buoy to usbl (just once)
    if (!buoy2usbl_catched_)
    {
      if (getStaticTransform(frame_buoy_, frame_usbl_, buoy2usbl_))
        buoy2usbl_catched_ = true;
      else
        return;
    }

    // Modem position
    geometry_msgs::PoseWithCovariance usbl2modem;
    usbl2modem.pose.position.x = (float)usbllong->N;
    usbl2modem.pose.position.y = (float)usbllong->E;
    usbl2modem.pose.position.z = (float)usbllong->U;
    usbl2modem.covariance[0] = (float)pow(usbllong->accuracy,2);
    usbl2modem.covariance[7] = (float)pow(usbllong->accuracy,2);
    usbl2modem.covariance[13] = (float)pow(usbllong->accuracy,2);

    geometry_msgs::PoseWithCovariance origin2buoy;
    if (!getBuoyPose(nav, origin2buoy)) return;

    transformAndPublish(usbl2modem, origin2buoy, usbllong->header.stamp);
  }

protected:


  bool getBuoyPose(const sensor_msgs::NavSatFix::ConstPtr& nav,
                   geometry_msgs::PoseWithCovariance& origin2buoy)
  {
    // Read the need origin from parameter server
    double ned_origin_lat, ned_origin_lon;
    if (!getNedOrigin(ned_origin_lat, ned_origin_lon))
    {
        ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the ned origin from the parameter server.");
        return false;
    }
    ned_ = new Ned(ned_origin_lat, ned_origin_lon, 0.0);

    // Buoy to NED
    double north_buoy, east_buoy, depth_buoy;
    ned_->geodetic2Ned(nav->latitude, nav->longitude, 0.0, north_buoy, east_buoy, depth_buoy);

    // TF Origin - Buoy
    origin2buoy.pose.position.x = north_buoy;
    origin2buoy.pose.position.y = east_buoy;
    origin2buoy.pose.position.z = 0.0;
    //origin2buoy_.covariance =                               // TODO: Insert the covariance of the gps

    // Publish TF
    tf::Transform tf_buoy;
    tf::Vector3 tf_buoy_v(north_buoy, east_buoy, 0.0);
    tf::Quaternion tf_buoy_q(0.0, 0.0, 0.0, 1);
    tf_buoy.setOrigin(tf_buoy_v);
    tf_buoy.setRotation(tf_buoy_q);
    broadcaster_.sendTransform(tf::StampedTransform(tf_buoy, nav->header.stamp, frame_map_, frame_buoy_));

    return true;
  }


  void spheric2cartesian(const double& bearing,
                         const double& elevation,
                         const double& depth,
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

  double getCovarianceAngles(const double& bearing,
                             const double& elevation,
                             const double& depth,
                             const double& accuracy,
                             double& sigma_x,
                             double& sigma_y)
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

    sigma_x = 2 * sqrt(pow(a * sin(bearing),2) + pow(b * cos(bearing),2)); // TODO: Check
    sigma_y = 2 * sqrt(pow(a * cos(bearing),2) + pow(b * sin(bearing),2));
    //sigma_x = 2 * sqrt(pow(a * cos(bearing),2) + pow(b * sin(bearing),2));
    //sigma_y = 2 * sqrt(pow(a * sin(bearing),2) + pow(b * cos(bearing),2));
  }

  bool getNedOrigin(double& ned_origin_lat, double& ned_origin_lon)
  {
    const string param_ned_origin_lat = "/navigator/ned_origin_lat";
    const string param_ned_origin_lon = "/navigator/ned_origin_lon";

    if (nh_.hasParam(param_ned_origin_lat) && nh_.hasParam(param_ned_origin_lon))
    {
        nh_.getParamCached(param_ned_origin_lat, ned_origin_lat);
        nh_.getParamCached(param_ned_origin_lon, ned_origin_lon);
        return true;
    }
    else
        return false;
  }

  bool getStaticTransform(const string& target_frame,
                          const string& source_frame,
                          geometry_msgs::Pose& msg)
  {
    tf::StampedTransform  transform;
    try
    {
      listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
      msg.position.x = transform.getOrigin().x();
      msg.position.y = transform.getOrigin().y();
      msg.position.z = transform.getOrigin().z();
      msg.orientation.x = transform.getRotation().x();
      msg.orientation.y = transform.getRotation().y();
      msg.orientation.z = transform.getRotation().z();
      msg.orientation.w = transform.getRotation().w();
      return true;
    }
    catch (tf::TransformException ex){
      ROS_ERROR_STREAM("[" << node_name_ << "]: Received an exception trying to transform a USBL point: " << ex.what());
      return false;
    }
  }

  void usblTransform(const geometry_msgs::PoseWithCovariance& usbl2modem,
                     const geometry_msgs::PoseWithCovariance& origin2buoy,
                           geometry_msgs::PoseWithCovariance& origin2modem)
  {
    geometry_msgs::PoseWithCovariance origin2usbl;
    pose_cov_ops::compose(origin2buoy, buoy2usbl_, origin2usbl);
    pose_cov_ops::compose(origin2usbl, usbl2modem, origin2modem);
  }

  void transformAndPublish(const geometry_msgs::PoseWithCovariance& usbl2modem,
                           const geometry_msgs::PoseWithCovariance& origin2buoy,
                           const ros::Time& stamp)
  {
    geometry_msgs::PoseWithCovariance origin2modem;
    usblTransform(usbl2modem, origin2buoy, origin2modem);

    geometry_msgs::PoseWithCovarianceStamped modem;
    modem.header.frame_id = frame_map_;
    modem.header.stamp = stamp;
    modem.pose = origin2modem;

    pub_modem_.publish(modem);
  }

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher pub_modem_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;

  string node_name_;

  ros::ServiceClient modem_on_;

  bool buoy2usbl_catched_;
  geometry_msgs::Pose buoy2usbl_;
  Ned* ned_;

  string frame_map_;
  string frame_buoy_;
  string frame_usbl_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_position");

  ros::NodeHandle nh;
  Position usbl_positioning(nh);

  // Message sync
  message_filters::Subscriber<evologics_ros::AcousticModemUSBLLONG> usbllong_sub(nh, "/sensors/usbllong",      20);
  message_filters::Subscriber<sensor_msgs::NavSatFix> buoy_1_sub(nh, "/sensors/buoy", 50);

  // Define syncs
  typedef message_filters::sync_policies::ApproximateTime<evologics_ros::AcousticModemUSBLLONG,
                                                          sensor_msgs::NavSatFix> sync_pol1;
  message_filters::Synchronizer<sync_pol1> sync1(sync_pol1(50), usbllong_sub, buoy_1_sub);
  sync1.registerCallback(boost::bind(&Position::usbllongCallback, &usbl_positioning, _1, _2));

  ros::spin();

  ros::shutdown();

  return 0;
}