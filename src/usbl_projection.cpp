#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"

#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <algorithm>
#include <vector>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//#include <mrpt_cov_ops/mrpt_cov_ops.h>


using namespace std;

class positioning
{
public:

  positioning() : ekf_init_(false)
  {
    //Subscribers
    sub_usbl_ = n_.subscribe("/sensors/modem", 1, &positioning::usblCallback, this);
    sub_odom_ = n_.subscribe("/ekf_odom/odometry", 1, &positioning::odomCallback, this);
    sub_gt_ = n_.subscribe("/sparus/ros_odom_to_pat", 1, &positioning::gtCallback, this);

    //Publishers
    pub_modem_position_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("sensors/modem_update", 1); 
  }

  void getOdomError(const tf::Transform& T_ekfOdom) // Get odometry error EKF_ODOM vs GroundTruth from simulator
  {
    tf::Vector3 gtOdom_v(gtOdom_.pose.pose.position.x, gtOdom_.pose.pose.position.y, gtOdom_.pose.pose.position.z);
    tf::Quaternion gtOdom_q(gtOdom_.pose.pose.orientation.x, gtOdom_.pose.pose.orientation.y, gtOdom_.pose.pose.orientation.z, gtOdom_.pose.pose.orientation.w);
    tf::Transform T_gtOdom_(gtOdom_q, gtOdom_v);

    tf::Transform T_odomError = T_ekfOdom * T_gtOdom_;

    // Convert to RPY
    tf::Matrix3x3 mat(T_odomError.getRotation());
    double roll;
    double pitch;
    double yaw;
    mat.getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM("GT-ODOM err ROT: " << roll << " \t " << pitch << "\t " << yaw);
  }

  void gtCallback(const nav_msgs::Odometry& odom)
  {
    gtOdom_ = odom;
  }

  void odomCallback(const nav_msgs::Odometry& odom)
  { 
    mutex_.lock();
    timestamps_.push_back(odom.header.stamp.toSec());
    hist_.push_back(odom);
    ekf_init_ = true;

    if (timestamps_.size() > 1000) 
    {
      timestamps_.erase(timestamps_.begin());
      hist_.erase(hist_.begin());
    }

    mutex_.unlock();
  }

  void usblCallback(const geometry_msgs::PoseWithCovarianceStamped& sparus_old)
  {
    // Wait to odometry msgs to start
    if (ekf_init_==false) return; 

    //// Find matching odom ////
    mutex_.lock();
    double instant = sparus_old.header.stamp.toSec();

    // Absolute value of the difference
    vector<double> abs_stamp;
    for (uint j=0; j<timestamps_.size(); j++)
      abs_stamp.push_back(fabs(instant-timestamps_[j]));

    vector<double>::iterator result = min_element(abs_stamp.begin(), abs_stamp.end());
    int min_idx = distance(abs_stamp.begin(), result);

    // Get sign, 1 to the negative, 2 to the positive
    int min_idx_1 = min_idx;
    int min_idx_2 = min_idx + 1;
    if (timestamps_[min_idx] > 0){ 
      int min_idx_1 = min_idx - 1;
      int min_idx_2 = min_idx;
    } 

    ROS_INFO_STREAM("Time error 1 (find)  : " << abs_stamp[min_idx_1]);
    ROS_INFO_STREAM("Time error 2 (find)  : " << abs_stamp[min_idx_2]);

    if (abs_stamp[min_idx] > 0.05)
    {
      ROS_WARN("NO SYNC BETWEEN ODOM AND USBL FOUND!");
      mutex_.unlock();
      return;
    }

    // Old Odometry Messages
    nav_msgs::Odometry odom_old_1 = hist_[min_idx_1];
    nav_msgs::Odometry odom_old_2 = hist_[min_idx_2];

    // Reshape historial
    timestamps_.erase(timestamps_.begin() , timestamps_.begin() + min_idx);
    hist_.erase(hist_.begin() , hist_.begin() + min_idx);
    mutex_.unlock();

    // Interpolation of old odometries near USBL arrival

    double prop = (abs_stamp[min_idx_1])/(abs_stamp[min_idx_1] + abs_stamp[min_idx_2]);

    tf::Vector3 odom_old_1_v(odom_old_1.pose.pose.position.x, odom_old_1.pose.pose.position.y, odom_old_1.pose.pose.position.z);
    tf::Vector3 odom_old_2_v(odom_old_2.pose.pose.position.x, odom_old_2.pose.pose.position.y, odom_old_2.pose.pose.position.z);
    tf::Vector3 odom_old_v = odom_old_1_v.lerp(odom_old_2_v, prop);

    tf::Quaternion odom_old_1_q(odom_old_1.pose.pose.orientation.x, odom_old_1.pose.pose.orientation.y, odom_old_1.pose.pose.orientation.z, odom_old_1.pose.pose.orientation.w);
    tf::Quaternion odom_old_2_q(odom_old_2.pose.pose.orientation.x, odom_old_2.pose.pose.orientation.y, odom_old_2.pose.pose.orientation.z, odom_old_2.pose.pose.orientation.w);
    tf::Quaternion odom_old_q = odom_old_1_q.slerp(odom_old_2_q, prop);

    // Last Odometry
    nav_msgs::Odometry odom_actual = hist_.back();
    ros::Time time_actual = odom_actual.header.stamp;


    //// Transformations ////
    // Static TF 
    tf::StampedTransform  T_s_m; 
    getTransform("sparus2",time_actual,"modem", time_actual, "map", T_s_m);

    // Odom to modem old
    tf::Transform T_sA(odom_old_q, odom_old_v);
    tf::Transform T_mA = T_sA * T_s_m; 


    // Odom to modem actual
    tf::Vector3 sB_v(odom_actual.pose.pose.position.x, odom_actual.pose.pose.position.y, odom_actual.pose.pose.position.z);
    tf::Quaternion sB_q(odom_actual.pose.pose.orientation.x, odom_actual.pose.pose.orientation.y, odom_actual.pose.pose.orientation.z, odom_actual.pose.pose.orientation.w);
    tf::Transform T_sB(sB_q, sB_v);
    tf::Transform T_mB = T_sB * T_s_m; 

    // Delta Odom: modem old - modem actual
    tf::Transform T_mA_mB = T_mA.inverse() * T_mB;  

    // Odom to modem old updated
    tf::Vector3 v(sparus_old.pose.pose.position.x, sparus_old.pose.pose.position.y, sparus_old.pose.pose.position.z); // Trans usbl old
    tf::Quaternion q(T_mA.getRotation().x(), T_mA.getRotation().y(), T_mA.getRotation().z(), T_mA.getRotation().w()); // Rot odom old
    tf::Transform T_mA_new(q, v);

    // Odom to modem actual updated
    tf::Transform T_mB_new = (T_mA_new * T_mA_mB);

    // Create message
    geometry_msgs::PoseWithCovarianceStamped modem_actual;
    modem_actual.header.frame_id = "modem_origin";
    modem_actual.header.stamp = time_actual;
    modem_actual.pose.pose.position.x = T_mB_new.getOrigin().x();
    modem_actual.pose.pose.position.y = T_mB_new.getOrigin().y();
    modem_actual.pose.pose.position.z = T_mB_new.getOrigin().z();
    modem_actual.pose.pose.orientation.x = T_mB_new.getRotation().x();
    modem_actual.pose.pose.orientation.y = T_mB_new.getRotation().y();
    modem_actual.pose.pose.orientation.z = T_mB_new.getRotation().z();
    modem_actual.pose.pose.orientation.w = T_mB_new.getRotation().w();

    pub_modem_position_.publish(modem_actual);
    getOdomError(T_sB);
    ROS_INFO("------------------------------------------------------------------");

  }

// output        -> string& out      -> foo(s)
// input (copy)  -> string in        -> foo(s)
// input (by ref)-> const string& in -> foo(s)

  void getTransform(const string& target_frame, const ros::Time& target_time,
                    const string& source_frame, const ros::Time& source_time,
                    const string& fixed_frame,  tf::StampedTransform& transform)
  {
    try
    {
      listener_.waitForTransform(target_frame, target_time, source_frame, source_time, fixed_frame, ros::Duration(0.2));
      listener_.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Received an exception trying to transform a USBL point: %s", ex.what());
    }  
  }


  void transformPoint(const string& target_frame, 
                      const ros::Time& target_time,
                      const geometry_msgs::PointStamped& point_in,
                      const string& fixed_frame,
                            geometry_msgs::PointStamped& point_out)
  {
    try{
      listener_.waitForTransform(target_frame, target_time, point_in.header.frame_id, point_in.header.stamp, fixed_frame, ros::Duration(0.2));
      listener_.transformPoint(target_frame, target_time, point_in, fixed_frame, point_out);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a USBL point: %s", ex.what());
    }
  }

private:

  ros::NodeHandle n_; 
  ros::Subscriber sub_usbl_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_gt_;
  ros::Publisher pub_modem_position_;


  boost::mutex mutex_;
  tf::TransformBroadcaster br_;
  vector<double> timestamps_;
  vector<nav_msgs::Odometry> hist_;
  tf::TransformListener listener_;
  bool ekf_init_;
  nav_msgs::Odometry gtOdom_;
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