#include "ros/ros.h"
#include <cmath>
#include "utils/ned.h"
#include <pose_cov_ops/pose_cov_ops.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "auv_msgs/NavSts.h"
#include "evologics_ros_sync/EvologicsUsbllong.h"
#include "sensor_msgs/NavSatFix.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_srvs/Empty.h"


using namespace std;

class Position
{
public:
  Position(ros::NodeHandle nh) : nh_(nh), nhp_("~")
  {
    // Node name
    node_name_ = ros::this_node::getName();
    ROS_INFO_STREAM("[" << node_name_ << "]: Running");

    // Get Params
    nh_.param("/frames/map", frame_map_, string("map"));
    nh_.param("/frames/sensors/usbl", frame_usbl_, string("usbl"));
    nh_.param("/sensors/usbl/covariance", cov_usbl_, 6.0);
    nh_.param("/sensors/usbl/rssi_max", rssi_max_, -20.0);
    nh_.param("/sensors/usbl/rssi_min", rssi_min_, -85.0);
    nh_.param("/sensors/usbl/integrity_min", integrity_min_, 100.0);

    // Subscribers
    sub_usbllong_ = nh_.subscribe("/sensors/usbllong", 1, &Position::UsbllongCb, this);

    //Publishers
    pub_modem_ = nhp_.advertise<geometry_msgs::PoseWithCovarianceStamped>("modem_delayed", 10);
  }

  void UsbllongCb(const evologics_ros_sync::EvologicsUsbllong::ConstPtr &usbllong)
  {
    ROS_INFO_STREAM("[" << node_name_ << "]: Received usbllong msg");
    if (!CheckMsgQuality(usbllong)) return;

    ROS_INFO_STREAM("[" << node_name_ << "]: Get USBL pose");
    geometry_msgs::PoseWithCovariance usbl_pose;
    if (!GetPose(usbl_pose)) return;

    ROS_INFO_STREAM("[" << node_name_ << "]: Get Modem pose");
    geometry_msgs::PoseWithCovariance modem_pose_relative;
    geometry_msgs::PoseWithCovariance modem_pose;
    GetPoseMsg(usbllong, modem_pose_relative);
    Transform(usbl_pose, modem_pose_relative, modem_pose);

    ROS_INFO_STREAM("[" << node_name_ << "]: Publish modem delayed position");
    Publish(modem_pose, usbllong->header.stamp);
  }

protected:
  bool CheckMsgQuality(const evologics_ros_sync::EvologicsUsbllong::ConstPtr& usbllong)
  {
    //The signal strength is acceptable when measured RSSI values lie between -20 dB and -85 dB.
    float rssi = usbllong->rssi;
    if ( rssi <= rssi_min_ || rssi > rssi_max_)
    {
      ROS_WARN_STREAM("[" << node_name_ << "]: The signal strength is not acceptable: rssi = " << rssi);
      return false;
    }
    //An acoustic link is considered weak if the Signal Integrity Level value is less than 100
    float integrity = usbllong->integrity;
    if (integrity < integrity_min_)
    {
      ROS_WARN_STREAM("[" << node_name_ << "]: Signal Integrity Level is not acceptable: integrity = " << integrity);
      return false;
    }
    return true;
  }

  bool GetPose(geometry_msgs::PoseWithCovariance& usbl_pose)
  {
    tf::StampedTransform transform;
    try
    {
      listener_.lookupTransform(frame_map_, frame_usbl_, ros::Time(0), transform);
      usbl_pose.pose.position.x = transform.getOrigin().x();
      usbl_pose.pose.position.y = transform.getOrigin().y();
      usbl_pose.pose.position.z = transform.getOrigin().z();
      usbl_pose.pose.orientation.x = transform.getRotation().x();
      usbl_pose.pose.orientation.y = transform.getRotation().y();
      usbl_pose.pose.orientation.z = transform.getRotation().z();
      usbl_pose.pose.orientation.w = transform.getRotation().w();
      return true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR_STREAM("[" << node_name_ << "]: Received an exception trying to retrieve the USBL pose: " << ex.what());
      return false;
    }
  }

  bool GetPoseMsg(const evologics_ros_sync::EvologicsUsbllong::ConstPtr& usbllong,
                  geometry_msgs::PoseWithCovariance& modem_pose_relative)
  {
    modem_pose_relative.pose.position.x = (float)usbllong->N;
    modem_pose_relative.pose.position.y = (float)usbllong->E;
    modem_pose_relative.pose.position.z = (float)usbllong->D;
    modem_pose_relative.covariance[0] = cov_usbl_;
    modem_pose_relative.covariance[7] = cov_usbl_;
    modem_pose_relative.covariance[14] = cov_usbl_;
  }

  void Transform(const geometry_msgs::PoseWithCovariance& usbl_pose,
                 const geometry_msgs::PoseWithCovariance& modem_pose_relative,
                 geometry_msgs::PoseWithCovariance& modem_pose)
  {
    pose_cov_ops::compose(usbl_pose, modem_pose_relative, modem_pose);
  }

  void Publish(const geometry_msgs::PoseWithCovariance& modem_pose,
               const ros::Time& stamp)
  {
    geometry_msgs::PoseWithCovarianceStamped modem_delayed;
    modem_delayed.header.frame_id = frame_map_;
    modem_delayed.header.stamp = stamp;
    modem_delayed.pose = modem_pose;
    pub_modem_.publish(modem_delayed);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber sub_usbllong_;
  ros::Publisher pub_modem_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;
  string node_name_;
  string frame_map_;
  string frame_usbl_;
  double cov_usbl_;
  double rssi_max_;
  double rssi_min_;
  double integrity_min_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_position");

  ros::NodeHandle nh;
  Position usbl_positioning(nh);

  ros::spin();

  ros::shutdown();

  return 0;
}
