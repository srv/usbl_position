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
    // Node name
    node_name_ = ros::this_node::getName();
    ROS_INFO_STREAM("[" << node_name_ << "]: Running");


    // Get Params
    nhp_.param("frames/map", frame_map_, string("map"));
    nhp_.param("frames/sensors/usbl", frame_usbl_, string("usbl"));
    nhp_.param("frames/sensors/buoy", frame_buoy_, string("buoy"));
    nhp_.param("sensors/usbl/covariance", cov_usbl_, 3.0);

    //Publishers
    pub_modem_ = nhp_.advertise<geometry_msgs::PoseWithCovarianceStamped>("modem_delayed", 10);
    pub_buoy_ = nhp_.advertise<geometry_msgs::PoseWithCovarianceStamped>("buoy_ned", 10);
  }

  void usbllongCallback(const evologics_ros::AcousticModemUSBLLONG::ConstPtr& usbllong,
                        const sensor_msgs::NavSatFix::ConstPtr& buoy)
  {
    // Get the static transform from buoy to usbl (just once)
    if (!buoy2usbl_catched_)
    {
      if (getStaticTransform(frame_buoy_, frame_usbl_, buoy2usbl_))
        buoy2usbl_catched_ = true;
      else
        return;
    }

    // Conditions
    if (!checkMsgQuality(usbllong)) return;

    geometry_msgs::PoseWithCovariance origin2buoy;
    if (!getBuoyPose(buoy, origin2buoy)) return;

    // Compute Modem position
    geometry_msgs::PoseWithCovariance usbl2modem;
    usbl2modem.pose.position.x = (float)usbllong->N;
    usbl2modem.pose.position.y = (float)usbllong->E;
    usbl2modem.pose.position.z = (float)usbllong->U;
    usbl2modem.covariance[0] = cov_usbl_;
    usbl2modem.covariance[7] = cov_usbl_;
    usbl2modem.covariance[14] = cov_usbl_;
    transformAndPublish(usbl2modem, origin2buoy, usbllong->header.stamp);
  }

protected:

  bool checkMsgQuality(const evologics_ros::AcousticModemUSBLLONG::ConstPtr& usbllong) // TODO filtr also the gps
  {
    bool flag = true;
    //The signal strength is acceptable when measured RSSI values lie between -20 dB and -85 dB.
    float rssi = usbllong->rssi;

    if ( rssi >= -85 || rssi < -20)
    {
      flag = true;
    }
    else
    {
      ROS_WARN_STREAM("[" << node_name_ << "]: The signal strength is not acceptable: rssi = " << rssi <<" < -20dB).");
      flag = false;
    }

    //An acoustic link is considered weak if the Signal Integrity Level value is less than 100
    float integrity = usbllong->integrity;
    if (integrity < 100)
    {
      ROS_WARN_STREAM("[" << node_name_ << "]: Signal Integrity Level is not acceptable: integrity = " << integrity <<" (integrity < 100).");
      flag = false;
    }
    return flag;
  }

  bool getBuoyPose(const sensor_msgs::NavSatFix::ConstPtr& buoy,
                   geometry_msgs::PoseWithCovariance& origin2buoy)
  {
    // Read NED Origin from parameter server
    double ned_origin_lat, ned_origin_lon;
    if (!getNedOrigin(ned_origin_lat, ned_origin_lon))
    {
        ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the ned origin from the parameter server.");
        return false;
    }
    ned_ = new Ned(ned_origin_lat, ned_origin_lon, 0.0);

    // Buoy to NED
    double north_buoy, east_buoy, depth_buoy;
    ned_->geodetic2Ned(buoy->latitude, buoy->longitude, 0.0, north_buoy, east_buoy, depth_buoy);

    // TF Origin - Buoy
    origin2buoy.pose.position.x = north_buoy;
    origin2buoy.pose.position.y = east_buoy;
    origin2buoy.pose.position.z = 0.0;
    origin2buoy.covariance[0] = buoy->position_covariance[0];
    origin2buoy.covariance[7] = buoy->position_covariance[4];
    origin2buoy.covariance[14] = buoy->position_covariance[8];

    // Publish buoy NED
    geometry_msgs::PoseWithCovarianceStamped buoy_ned;
    buoy_ned.header.stamp = buoy->header.stamp;
    buoy_ned.pose.pose.position.x = north_buoy;
    buoy_ned.pose.pose.position.y = east_buoy;
    buoy_ned.pose.pose.position.z = depth_buoy;
    buoy_ned.pose.covariance = origin2buoy.covariance;

    pub_buoy_.publish(buoy_ned);

    // Publish  Buoy TF
    tf::Transform tf_buoy;
    tf::Vector3 tf_buoy_v(north_buoy, east_buoy, 0.0);
    tf::Quaternion tf_buoy_q(0.0, 0.0, 0.0, 1);
    tf_buoy.setOrigin(tf_buoy_v);
    tf_buoy.setRotation(tf_buoy_q);
    broadcaster_.sendTransform(tf::StampedTransform(tf_buoy, buoy->header.stamp, frame_map_, frame_buoy_));
    return true;
  }

// USBLANGLES
  // void spheric2cartesian(const double& bearing,
  //                        const double& elevation,
  //                        const double& depth,
  //                        double& x,
  //                        double& y,
  //                        double& z)
  // {
  //   //x = depth * tan(elevation) * cos(bearing);
  //   //y = depth * tan(elevation) * sin(bearing);
  //   x = depth * sin(bearing) / tan(elevation);
  //   y = depth * cos(bearing) / tan(elevation);
  //   z = depth; //TODO: Integrate depth of the USBL
  // }

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
  ros::Publisher pub_buoy_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;

  string node_name_;

  bool buoy2usbl_catched_;
  geometry_msgs::Pose buoy2usbl_;
  Ned* ned_;

  string frame_map_;
  string frame_buoy_;
  string frame_usbl_;
  double cov_usbl_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_position");

  ros::NodeHandle nh;
  Position usbl_positioning(nh);

  // Message sync
  message_filters::Subscriber<evologics_ros::AcousticModemUSBLLONG> usbllong_sub(nh, "/sensors/usbllong", 50);
  message_filters::Subscriber<sensor_msgs::NavSatFix> buoy_sub(nh, "/sensors/buoy", 50);

  // Define syncs
  typedef message_filters::sync_policies::ApproximateTime<evologics_ros::AcousticModemUSBLLONG,
                                                          sensor_msgs::NavSatFix> sync_pool;
  message_filters::Synchronizer<sync_pool> sync(sync_pool(50), usbllong_sub, buoy_sub);
  sync.registerCallback(boost::bind(&Position::usbllongCallback, &usbl_positioning, _1, _2));

  ros::spin();

  ros::shutdown();

  return 0;
}