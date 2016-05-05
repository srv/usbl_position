#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"

#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <algorithm>
#include <vector>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pose_cov_ops/pose_cov_ops.h>


using namespace std;

class Projection
{
public:

  Projection() : nhp_("~"), ekf_init_(false), sparus2modem_catched_(false), sync_init_(false)
  {
    // Node name
    node_name_ = ros::this_node::getName();

    // Get Params
    nh_.param("frames/base_link", frame_base_link_, string("sparus2"));
    nh_.param("frames/sensors/modem", frame_modem_, string("modem"));
    nh_.param("frames/sensors/origin_suffix", frame_suffix_, string("origin"));

    nhp_.param("sync_time_th", sync_time_th_, 0.1);
    nhp_.param("sync_dist_th", sync_dist_th_, 1.5);

    // Subscribers
    sub_usbl_ =     nh_.subscribe("/sensors/modem_delayed_acoustic", 1, &Projection::usblCallback, this);
    sub_ekfOdom_ =  nh_.subscribe("/ekf_odom/odometry", 1, &Projection::ekfOdomCallback, this);
    sub_ekfMap_ =   nh_.subscribe("/ekf_map/odometry", 1, &Projection::ekfMapCallback, this);
    sub_gt_ =       nh_.subscribe("/sparus/ros_odom_to_pat", 1, &Projection::gtCallback, this);

    // Publishers
    pub_modem_position_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("sensors/modem_raw", 1);
  }

  // Get odometry error EKF_MAP vs GroundTruth from simulator
  double getOdomError(double& r_er, double& p_er, double& y_er)
  {
    tf::Quaternion gtOdom_q(gt_odom_.pose.pose.orientation.x,
                            gt_odom_.pose.pose.orientation.y,
                            gt_odom_.pose.pose.orientation.z,
                            gt_odom_.pose.pose.orientation.w);

    tf::Quaternion ekfMap_q(ekf_map_.pose.pose.orientation.x,
                            ekf_map_.pose.pose.orientation.y,
                            ekf_map_.pose.pose.orientation.z,
                            ekf_map_.pose.pose.orientation.w);
    double roll_ekf;
    double pitch_ekf;
    double yaw_ekf;

    double roll_gt;
    double pitch_gt;
    double yaw_gt;

    tf::Matrix3x3 mat_ekf(ekfMap_q);
    tf::Matrix3x3 mat_gt(gtOdom_q);
    mat_ekf.getRPY(roll_ekf, pitch_ekf, yaw_ekf);
    mat_gt.getRPY(roll_gt, pitch_gt, yaw_gt);
    double rad2deg = 180.0/M_PI;

    r_er = fabs(roll_ekf  - roll_gt) *rad2deg;
    p_er = fabs(pitch_ekf - pitch_gt)*rad2deg;
    y_er = fabs(yaw_ekf   - yaw_gt)  *rad2deg;

    return sqrt(r_er*r_er + p_er*p_er + y_er*y_er);
  }

  // Find odometry at the time of arrival of the usbl msg from the hist. trough linear interpolation
  bool findOdom(const double& usbl_stamp,
                nav_msgs::Odometry& odom_for_usbl,
                nav_msgs::Odometry& last_odom,
                double& last_odom_stamp)
  {
    // Absolute value of the difference
    vector<double> abs_stamp;
    mutex_.lock();
    for (uint j=0; j<odom_stamps_.size(); j++)
      abs_stamp.push_back(fabs(usbl_stamp - odom_stamps_[j]));

    vector<double>::iterator result = min_element(abs_stamp.begin(), abs_stamp.end());
    int min_idx = distance(abs_stamp.begin(), result);

    // Sync threshold between usbl and odometry measures
    if (abs_stamp[min_idx] > sync_time_th_)
    {
      ROS_WARN_STREAM("[" << node_name_ << "]: No sync between odom and usbl found!");
      mutex_.unlock();
      return false;
    }

    // Get sides
    int min_idx_1 = min_idx;
    int min_idx_2 = min_idx + 1;
    if (odom_stamps_[min_idx] > usbl_stamp){
      int min_idx_1 = min_idx - 1;
      int min_idx_2 = min_idx;
    }

    // Neighbor Odometry Messages
    nav_msgs::Odometry odom_1 = odom_hist_[min_idx_1];
    nav_msgs::Odometry odom_2 = odom_hist_[min_idx_2];

    // Interpolation of neighbor odoms near msg arrival
    float prop = abs_stamp[min_idx_1] / (abs_stamp[min_idx_1] + abs_stamp[min_idx_2]);
    odomInterpolation(odom_1, odom_2, prop, odom_for_usbl);

    // Reshape hist.
    odom_stamps_.erase(odom_stamps_.begin() , odom_stamps_.begin() + min_idx);
    odom_hist_.erase(odom_hist_.begin() , odom_hist_.begin() + min_idx);
    last_odom = odom_hist_.back();
    last_odom_stamp = odom_stamps_.back();
    mutex_.unlock();
    return true;
  }

  void odomInterpolation(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2,
                         const float& prop, nav_msgs::Odometry& odom)
  {
    tf::Vector3 odom1_v(odom1.pose.pose.position.x, odom1.pose.pose.position.y, odom1.pose.pose.position.z);
    tf::Vector3 odom2_v(odom2.pose.pose.position.x, odom2.pose.pose.position.y, odom2.pose.pose.position.z);
    tf::Vector3 odom_v = odom1_v.lerp(odom2_v, prop);

    tf::Quaternion odom1_q(odom1.pose.pose.orientation.x, odom1.pose.pose.orientation.y, odom1.pose.pose.orientation.z, odom1.pose.pose.orientation.w);
    tf::Quaternion odom2_q(odom2.pose.pose.orientation.x, odom2.pose.pose.orientation.y, odom2.pose.pose.orientation.z, odom2.pose.pose.orientation.w);
    tf::Quaternion odom_q = odom1_q.slerp(odom2_q, prop);

    for (uint i=0; i<odom.twist.covariance.size(); i++)
      odom.twist.covariance[i] = odom1.twist.covariance[i]*(1-prop) + odom2.twist.covariance[i]*prop;

    odom.pose.pose.position.x = odom_v.x();
    odom.pose.pose.position.y = odom_v.y();
    odom.pose.pose.position.z = odom_v.z();
    odom.pose.pose.orientation.x = odom_q.x();
    odom.pose.pose.orientation.y = odom_q.y();
    odom.pose.pose.orientation.z = odom_q.z();
    odom.pose.pose.orientation.w = odom_q.w();
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
      ROS_ERROR("Received an exception trying to transform a USBL point: %s", ex.what());
      return false;
    }
  }

  void gtCallback(const nav_msgs::Odometry& odom)
  {
    gt_odom_ = odom;
  }

  void ekfMapCallback(const nav_msgs::Odometry& odom)
  {
    ekf_map_ = odom;
  }

  void ekfOdomCallback(const nav_msgs::Odometry& odom)
  {
    // Store odometry values and stamps
    mutex_.lock();
    odom_stamps_.push_back(odom.header.stamp.toSec());
    odom_hist_.push_back(odom);
    ekf_init_ = true;

    // Delete older
    if (odom_stamps_.size() > 1000)
    {
      odom_stamps_.erase(odom_stamps_.begin());
      odom_hist_.erase(odom_hist_.begin());
    }

    mutex_.unlock();
  }


  void usblCallback(const geometry_msgs::PoseWithCovarianceStamped& usbl_msg)
  {
    // Wait to odometry msgs to start
    if (ekf_init_ == false) return;

    if (!sparus2modem_catched_)
    {
      if (getStaticTransform(frame_base_link_, frame_modem_, sparus2modem_))
        sparus2modem_catched_ = true;
      else
        return;
    }

    // Measurement timestamp
    double usbl_stamp = usbl_msg.header.stamp.toSec();

    // Get Old Odometry
    nav_msgs::Odometry odom_at_usbl_stamp;
    nav_msgs::Odometry last_odom;
    double last_odom_stamp;
    if (!findOdom(usbl_stamp, odom_at_usbl_stamp, last_odom, last_odom_stamp))
      return;

    // Extract positions
    tf::Vector3 p_usbl(usbl_msg.pose.pose.position.x,
                       usbl_msg.pose.pose.position.y,
                       usbl_msg.pose.pose.position.z);
    tf::Vector3 p_odom(odom_at_usbl_stamp.pose.pose.position.x,
                       odom_at_usbl_stamp.pose.pose.position.y,
                       odom_at_usbl_stamp.pose.pose.position.z);

    // Distance threshold between usbl and odometry measures
    if (!sync_init_)
    {
      last_usbl_sync_ = p_usbl;
      last_odom_sync_ = p_odom;

      sync_init_ = true;
      return;
    }
    else
    {
      // Check distance (x,y)
      tf::Vector3 usbl_displacement = last_usbl_sync_ - p_usbl;
      tf::Vector3 odom_displacement = last_odom_sync_ - p_odom;
      tf::Vector3 d = usbl_displacement - odom_displacement;
      double dist = sqrt(d.x()*d.x() + d.y()*d.y());

      // Update
      last_usbl_sync_ = p_usbl;
      last_odom_sync_ = p_odom;

      if (dist > sync_dist_th_)
      {
        ROS_WARN_STREAM("[" << node_name_ << "]: Big distance between usbl position and ekf position: " << dist << "m (threshold: " << sync_dist_th_ << "m).");
        return;
      }
    }

    // Delta Odom
    geometry_msgs::Pose sparus_A = odom_at_usbl_stamp.pose.pose;
    geometry_msgs::Pose sparus_B = last_odom.pose.pose;
    geometry_msgs::Pose modem_A;
    geometry_msgs::Pose modem_B;
    pose_cov_ops::compose(sparus_A, sparus2modem_, modem_A);
    pose_cov_ops::compose(sparus_B, sparus2modem_, modem_B);

    geometry_msgs::Pose delta_odom;
    pose_cov_ops::inverseCompose(modem_B, modem_A, delta_odom);

    // USBL correction
    geometry_msgs::Pose modem_A_new;
    modem_A_new = usbl_msg.pose.pose;
    // Approximate covariances as USBL covariances, odometric covariances are so small //TODO:add justification from odom vel integration
    modem_A_new.orientation = odom_at_usbl_stamp.pose.pose.orientation;

    geometry_msgs::Pose modem_B_new;
    pose_cov_ops::compose(modem_A_new, delta_odom, modem_B_new);

    // Create message
    geometry_msgs::PoseWithCovarianceStamped modem_update;
    modem_update.header.frame_id = frame_modem_ + "_" + frame_suffix_;
    modem_update.header.stamp = ros::Time(last_odom_stamp);
    modem_update.pose.pose = modem_B_new;
    modem_update.pose.covariance[0] = usbl_msg.pose.covariance[0];
    modem_update.pose.covariance[7] = usbl_msg.pose.covariance[7];
    modem_update.pose.covariance[14] = usbl_msg.pose.covariance[14];
    pub_modem_position_.publish(modem_update);
  }


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber sub_usbl_;
  ros::Subscriber sub_ekfOdom_;
  ros::Subscriber sub_ekfMap_;
  ros::Subscriber sub_gt_;
  ros::Publisher pub_modem_position_;

  string node_name_;
  string frame_suffix_;
  string frame_modem_;
  string frame_base_link_;

  boost::mutex mutex_;
  tf::TransformBroadcaster br_;
  vector<double> odom_stamps_;
  vector<nav_msgs::Odometry> odom_hist_;
  tf::TransformListener listener_;
  bool ekf_init_;
  nav_msgs::Odometry gt_odom_;
  nav_msgs::Odometry ekf_map_;
  geometry_msgs::Pose sparus2modem_;
  bool sparus2modem_catched_;

  bool sync_init_;
  tf::Vector3 last_odom_sync_;
  tf::Vector3 last_usbl_sync_;

  double sync_time_th_;
  double sync_dist_th_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_position_node");

  ROS_INFO("Initialize usbl Projection class");
  Projection usbl_positioning;

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}