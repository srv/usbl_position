#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"

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

  Projection() : ekf_init_(false), sparus2modem_catched_(false), sync_init_(false), used_positions_(0)
  {
    // Node name
    node_name_ = ros::this_node::getName();

    // Get Params
    nh_.param("frames/base_link", frame_base_link_, string("sparus2"));
    nh_.param("frames/sensors/modem", frame_modem_, string("modem"));
    nh_.param("frames/sensors/origin_suffix", frame_suffix_, string("origin"));

    nh_.param("sensors/usbl/sync_time_th", sync_time_th_, 0.1);
    nh_.param("sensors/usbl/sync_prop_th", sync_prop_th_, 0.5);
    nh_.param("sensors/usbl/sync_disp_th", sync_disp_th_, 0.7);
    nh_.param("sensors/usbl/odom_queue_len", odom_queue_len_, 1000);
    nh_.param("sensors/usbl/percentage_queue_len", percentage_queue_len_, 100);
    nh_.param("sensors/usbl/covariance", usbl_cov_, 4.0);


    // Subscribers
    sub_usbl_ =     nh_.subscribe("modem_delayed", 1, &Projection::usblCallback, this);
    sub_ekfOdom_ =  nh_.subscribe("ekf_odom", 1, &Projection::ekfOdomCallback, this);
    sub_ekfMap_ =  nh_.subscribe("ekf_map", 1, &Projection::ekfMapCallback, this);

    // Publishers
    pub_modem_position_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("modem_raw", 1);
    pub_modem_used_positions_per_ = nh_.advertise<std_msgs::Float64>("used_positions_per", 1);
    pub_modem_position_delay_ = nh_.advertise<std_msgs::Float64>("position_delay", 1);
  }


  // Find odometry at the time of arrival of the usbl msg from the hist. trough linear interpolation
  bool findOdom(const double& usbl_stamp,
                nav_msgs::Odometry& odom_for_usbl,
                nav_msgs::Odometry& last_odom,
                double& last_odom_stamp)
  {
    // Absolute value of the differencez
    vector<double> abs_stamp;
    mutex_.lock();
    for (uint j=0; j<odom_stamps_.size(); j++)
      abs_stamp.push_back(fabs(usbl_stamp - odom_stamps_[j]));

    vector<double>::iterator result = min_element(abs_stamp.begin(), abs_stamp.end());
    int min_idx = distance(abs_stamp.begin(), result);

    // Sync threshold between usbl and odometry measures
    if (abs_stamp[min_idx] > sync_time_th_)
    {
      ROS_WARN_STREAM("[" << node_name_ << "]: No sync between odom and usbl found!  " << abs_stamp[min_idx] << " > " << sync_time_th_);
      ROS_WARN_STREAM("[" << node_name_ << "]: No sync between odom and usbl found!  Size = " << odom_stamps_.size());
      mutex_.unlock();
      return false;
    }

    // Get sides
    int min_idx_1 = min_idx - 1;
    int min_idx_2 = min_idx;
    if (odom_stamps_[min_idx] < usbl_stamp)
    {
      min_idx_1 = min_idx;
      min_idx_2 = min_idx + 1;
    }

    if (min_idx < (int)odom_stamps_.size() - 1) // If inside the historial (interpolate)
    {
      // Neighbor Odometry Messages
      nav_msgs::Odometry odom_1 = odom_hist_[min_idx_1];
      nav_msgs::Odometry odom_2 = odom_hist_[min_idx_2];
      // Interpolation of neighbor odoms near msg arrival
      float prop = abs_stamp[min_idx_1] / (abs_stamp[min_idx_1] + abs_stamp[min_idx_2]);
      odomInterpolation(odom_1, odom_2, prop, odom_for_usbl);
    }
    else  // If at the end of the historial (pick the last one)
    {
      odom_for_usbl = odom_hist_[min_idx];
    }

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
      ROS_ERROR_STREAM("[" << node_name_ << "]: Received an exception trying to transform a USBL point: " << ex.what());
      return false;
    }
  }

  void ekfOdomCallback(const nav_msgs::Odometry& odom)
  {
    // Store odometry values and stamps
    mutex_.lock();
    odom_stamps_.push_back(odom.header.stamp.toSec());
    odom_hist_.push_back(odom);
    ekf_init_ = true;

    // Delete older
    if ((int)odom_stamps_.size() > odom_queue_len_)
    {
      odom_stamps_.erase(odom_stamps_.begin());
      odom_hist_.erase(odom_hist_.begin());
    }

    mutex_.unlock();
  }

  void ekfMapCallback(const nav_msgs::Odometry& map) //TODO: May be less consuming to syncronize with a gps measurement
  {
    ekf_map_ = map;
  }

  void getPercentage()
  {
    used_positions_.back() = 1;

    float sum = 0;
    for (uint j=0; j<used_positions_.size(); j++)
      sum = sum + used_positions_[j];

    std_msgs::Float64 used_positions_per;
    used_positions_per.data = sum / used_positions_.size() *100;

    pub_modem_used_positions_per_.publish(used_positions_per);
  }

  void getDelay(const double& odom, const double& usbl)
  {
    std_msgs::Float64 delay;
    delay.data = odom - usbl;
    pub_modem_position_delay_.publish(delay);
  }

  void usblCallback(const geometry_msgs::PoseWithCovarianceStamped& usbl_msg)
  {
    used_positions_.push_back(0);
    if ((int)used_positions_.size() > percentage_queue_len_)
      used_positions_.erase(used_positions_.begin());


    // Wait for odometry msgs to start
    if (ekf_init_ == false) return;


    // Get static Transform sparus2modem only once
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

      ROS_WARN_STREAM("[" << node_name_ << "]: Restart sync");
      sync_init_ = true;
      return;
    }
    else
    {
      // Check distance (x,y)
      tf::Vector3 usbl_displacement = p_usbl - last_usbl_sync_;
      tf::Vector3 odom_displacement = p_odom - last_odom_sync_;
      tf::Vector3 d = usbl_displacement - odom_displacement;

      double dist = sqrt(d.x()*d.x() + d.y()*d.y());
      double odom_disp = sqrt(odom_displacement.x()*odom_displacement.x() + odom_displacement.y()*odom_displacement.y());

      //ROS_INFO_STREAM("USBL: x="<<usbl_displacement.x() << "/ y="<<usbl_displacement.y() <<  "/ z="<<usbl_displacement.z());
      //ROS_INFO_STREAM("USBL: x="<<odom_displacement.x() << "/ y="<<odom_displacement.y() <<  "/ z="<<odom_displacement.z());

      // Update
      last_usbl_sync_ = p_usbl;
      last_odom_sync_ = p_odom;

      // Filter by distance
      if (dist > sync_disp_th_ + sync_prop_th_*odom_disp) // TODO: sync_disp_th_ can be extracted from the sensor covariance sync_dist_th_ is a kind of odometry drift
      {
        ROS_WARN_STREAM("[" << node_name_ << "]: Big distance between usbl position and ekf position: " << dist << "m (threshold: " << sync_disp_th_ + sync_prop_th_*odom_disp << "m).");
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
    modem_update.header.frame_id = frame_modem_ + frame_suffix_;
    modem_update.header.stamp = ros::Time(last_odom_stamp);
    modem_update.pose.pose = modem_B_new;
    modem_update.pose.covariance[0] = usbl_cov_;//usbl_msg.pose.covariance[0];
    modem_update.pose.covariance[7] = usbl_cov_;//usbl_msg.pose.covariance[7];
    modem_update.pose.covariance[14] = usbl_cov_;//usbl_msg.pose.covariance[14];
    pub_modem_position_.publish(modem_update);

    getDelay(last_odom_stamp, usbl_stamp);
    getPercentage();
  }


private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_usbl_;
  ros::Subscriber sub_ekfOdom_;
  ros::Subscriber sub_ekfMap_;

  ros::Publisher pub_modem_position_;
  ros::Publisher pub_modem_used_positions_per_;
  ros::Publisher pub_modem_position_delay_;

  string node_name_;
  string frame_suffix_;
  string frame_modem_;
  string frame_base_link_;

  boost::mutex mutex_;
  vector<double> odom_stamps_;
  vector<nav_msgs::Odometry> odom_hist_;
  vector<double>  map_stamps_;
  vector<nav_msgs::Odometry> map_hist_;
  tf::TransformListener listener_;
  bool ekf_init_;
  nav_msgs::Odometry gt_odom_;
  nav_msgs::Odometry ekf_map_;
  geometry_msgs::Pose sparus2modem_;
  bool sparus2modem_catched_;

  bool sync_init_;
  tf::Vector3 last_odom_sync_;
  tf::Vector3 last_usbl_sync_;

  nav_msgs::Odometry last_gps_map_;

  double sync_time_th_;
  double sync_prop_th_;
  double sync_disp_th_;
  double usbl_cov_;

  vector<int> used_positions_;
  int percentage_queue_len_;
  int odom_queue_len_;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usbl_position_node");

  ROS_INFO("Initialize usbl Projection class");
  Projection usbl_positioning;

  ros::spin();
  ros::shutdown();

  return 0;
}
