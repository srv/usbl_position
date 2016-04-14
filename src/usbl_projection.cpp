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

class positioning
{
public:

  positioning() : ekf_init_(false)
  {
    getStaticTransform("sparus2","modem", sparus2modem_);

    //Subscribers
    sub_usbl_ =     n_.subscribe("/sensors/modem", 1, &positioning::usblCallback, this);
    sub_ekfOdom_ =  n_.subscribe("/ekf_odom/odometry", 1, &positioning::ekfOdomCallback, this);
    sub_ekfMap_ =   n_.subscribe("/ekf_map/odometry", 1, &positioning::ekfMapCallback, this);
    sub_gt_ =       n_.subscribe("/sparus/ros_odom_to_pat", 1, &positioning::gtCallback, this);

    //Publishers
    pub_modem_position_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("sensors/modem_update", 1); 
  }

// output        -> string& out      -> foo(s)
// input (copy)  -> string in        -> foo(s)
// input (by ref)-> const string& in -> foo(s)

  void getOdomError() // Get odometry error EKF_MAP vs GroundTruth from simulator
  {
    tf::Quaternion gtOdom_q(gtOdom_.pose.pose.orientation.x, gtOdom_.pose.pose.orientation.y, gtOdom_.pose.pose.orientation.z, gtOdom_.pose.pose.orientation.w);
    tf::Quaternion ekfMap_q(ekfMap_.pose.pose.orientation.x, ekfMap_.pose.pose.orientation.y, ekfMap_.pose.pose.orientation.z, ekfMap_.pose.pose.orientation.w);

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
    double rad2deg = 180/3.1416;

    // ROS_INFO_STREAM("EKF   (deg): " << roll_ekf*rad2deg << " \t " << pitch_ekf*rad2deg << "\t " << yaw_ekf*rad2deg);
    // ROS_INFO_STREAM("GT    (deg): " << roll_gt*rad2deg << " \t " << pitch_gt*rad2deg << "\t " << yaw_gt*rad2deg);
    // ROS_INFO_STREAM("diff  (deg): " << roll_ekf*rad2deg - roll_gt*rad2deg << " \t " <<  pitch_ekf*rad2deg-pitch_gt*rad2deg << "\t " << yaw_ekf*rad2deg-yaw_gt*rad2deg);
  }

  void findOdom(const double& msg_time, nav_msgs::Odometry& odom, vector<nav_msgs::Odometry>& odomVector, double& time_B) //Find odometry at the time of arrival of the usbl msg from the historial trough linear interpolation
  {
    // Absolute value of the difference
    vector<double> abs_stamp;
    mutex_.lock();
    for (uint j=0; j<timestamps_.size(); j++)
      abs_stamp.push_back(fabs(msg_time-timestamps_[j]));

    vector<double>::iterator result = min_element(abs_stamp.begin(), abs_stamp.end());
    int min_idx = distance(abs_stamp.begin(), result);

    // Get side
    int min_idx_1 = min_idx;
    int min_idx_2 = min_idx + 1;
    if (timestamps_[min_idx] > msg_time){ 
      int min_idx_1 = min_idx - 1;
      int min_idx_2 = min_idx;
    } 

    //Threshold
    if (abs_stamp[min_idx] > 0.05) 
    {
      ROS_WARN("NO SYNC BETWEEN ODOM AND USBL FOUND!");
      mutex_.unlock();
      return;
    }

    // Neighbor Odometry Messages
    nav_msgs::Odometry odom_1 = odomHistorial_[min_idx_1];
    nav_msgs::Odometry odom_2 = odomHistorial_[min_idx_2];

    // Interpolation of neightbor odometries near msg arrival
    float prop = (abs_stamp[min_idx_1])/(abs_stamp[min_idx_1] + abs_stamp[min_idx_2]);
    odomInterpolation(odom_1, odom_2, odom, prop); // TODO:prop comp inside

    // Reshape historial
    timestamps_.erase(timestamps_.begin() , timestamps_.begin() + min_idx);
    odomHistorial_.erase(odomHistorial_.begin() , odomHistorial_.begin() + min_idx);
    odomVector = odomHistorial_;
    time_B = timestamps_.back();
    mutex_.unlock();
  }

  void odomInterpolation(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2, 
                               nav_msgs::Odometry& odom,  const float& prop)
  {
    tf::Vector3 odom1_v(odom1.pose.pose.position.x, odom1.pose.pose.position.y, odom1.pose.pose.position.z);
    tf::Vector3 odom2_v(odom2.pose.pose.position.x, odom2.pose.pose.position.y, odom2.pose.pose.position.z);
    tf::Vector3 odom_v = odom1_v.lerp(odom2_v, prop);

    tf::Quaternion odom1_q(odom1.pose.pose.orientation.x, odom1.pose.pose.orientation.y, odom1.pose.pose.orientation.z, odom1.pose.pose.orientation.w);
    tf::Quaternion odom2_q(odom2.pose.pose.orientation.x, odom2.pose.pose.orientation.y, odom2.pose.pose.orientation.z, odom2.pose.pose.orientation.w);
    tf::Quaternion odom_q = odom1_q.slerp(odom2_q, prop);

    //TODO: put in matrix form
    for (int i = 0; i < 36; ++i)
    {
      odom.twist.covariance[i] = odom1.twist.covariance[i]*(1-prop) + odom2.twist.covariance[i]*prop; 
    }

    odom.pose.pose.position.x = odom_v.x();
    odom.pose.pose.position.y = odom_v.y();
    odom.pose.pose.position.z = odom_v.z();
    odom.pose.pose.orientation.x = odom_q.x();
    odom.pose.pose.orientation.y = odom_q.y();
    odom.pose.pose.orientation.z = odom_q.z();
    odom.pose.pose.orientation.w = odom_q.w();
  }

  void getStaticTransform(const string& target_frame,
                          const string& source_frame,
                          geometry_msgs::Pose& msg)
  {
    tf::StampedTransform  transform;
    try
    {
      listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(2));
      listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Received an exception trying to transform a USBL point: %s", ex.what());
    }  
    msg.position.x = transform.getOrigin().x();
    msg.position.y = transform.getOrigin().y();
    msg.position.z = transform.getOrigin().z();
    msg.orientation.x = transform.getRotation().x();
    msg.orientation.y = transform.getRotation().y();
    msg.orientation.z = transform.getRotation().z();
    msg.orientation.w = transform.getRotation().w();
  }

  void getDeltaOdom(const double& time_A, const vector<nav_msgs::Odometry>& odomVector, geometry_msgs::PoseWithCovariance& s1)
  {
    double t0 = time_A;
    double t1, dt, x, y, z, roll, pitch, yaw;
    tf::Quaternion q;
    geometry_msgs::PoseWithCovariance s0, ds;

    for (int i = 0; i < odomVector.size(); ++i)
    {
      t1 = odomVector[i].header.stamp.toSec();
      dt = t1 -t0;
      ds.pose.position.x = odomVector[i].twist.twist.linear.x * dt;
      ds.pose.position.y = odomVector[i].twist.twist.linear.y * dt;
      ds.pose.position.z = odomVector[i].twist.twist.linear.z * dt;
      roll = odomVector[i].twist.twist.angular.x * dt;
      pitch = odomVector[i].twist.twist.angular.y * dt;
      yaw = odomVector[i].twist.twist.angular.z * dt;
      q.setRPY(roll,pitch,yaw);
      ds.pose.orientation.x = q.x();
      ds.pose.orientation.y = q.y();
      ds.pose.orientation.z = q.z();
      ds.pose.orientation.w = q.w();

      //TODO: put in matrix form
      for (int j = 0; j < 36; ++j)
      {
        ds.covariance[j] = odomVector[i].twist.covariance[j]*pow(dt,2); 
      }

      //pose_cov_ops::compose(dsparus, sparus2modem_, dmodem);
      pose_cov_ops::compose(s0, ds, s1);
      s0 = s1;
      t0 = t1;
    }
  }

  void gtCallback(const nav_msgs::Odometry& odom)
  {
    gtOdom_ = odom;
  }

  void ekfMapCallback(const nav_msgs::Odometry& odom)
  {
    ekfMap_ = odom;
  }

  void ekfOdomCallback(const nav_msgs::Odometry& odom)
  { 
    ekfOdom_ = odom;

    mutex_.lock();
    timestamps_.push_back(odom.header.stamp.toSec());
    odomHistorial_.push_back(odom);
    ekf_init_ = true;

    if (timestamps_.size() > 1000) 
    {
      timestamps_.erase(timestamps_.begin());
      odomHistorial_.erase(odomHistorial_.begin());
    }

    mutex_.unlock();
  }


  void usblCallback(const geometry_msgs::PoseWithCovarianceStamped& usbl_msg)
  {
    // Wait to odometry msgs to start
    if (ekf_init_==false) return; 

    //Measurament timestamp
    double time_A = usbl_msg.header.stamp.toSec();

    // Get Old Odometry
    nav_msgs::Odometry odom_A;
    vector<nav_msgs::Odometry> odomVector;
    double timeB;
    findOdom(time_A, odom_A, odomVector, timeB);
    ros::Time time_B(timeB);


    // Delta Odom
    geometry_msgs::PoseWithCovariance delta_odom;
    getDeltaOdom(time_A, odomVector, delta_odom);

    // USBL correction
    geometry_msgs::PoseWithCovariance modem_A_new;
    geometry_msgs::PoseWithCovariance sparus_A_new;
    modem_A_new = usbl_msg.pose;
    pose_cov_ops::inverseCompose(modem_A_new, sparus2modem_, sparus_A_new);

    // USBL update position combined with existing odometry orientation
    sparus_A_new.pose.orientation = odom_A.pose.pose.orientation;
    sparus_A_new.covariance[21,22,23] = odom_A.pose.covariance[21,22,23];
    sparus_A_new.covariance[27,28,29] = odom_A.pose.covariance[27,28,29];
    sparus_A_new.covariance[33,34,35] = odom_A.pose.covariance[33,34,35];


    geometry_msgs::PoseWithCovariance sparus_B_new;
    pose_cov_ops::compose(sparus_A_new, delta_odom, sparus_B_new);

    // TO /modem //TODO: needed?, enough publishing in /map for the filter?? 
    geometry_msgs::PoseWithCovariance modem_B_new;
    pose_cov_ops::compose(sparus_B_new, sparus2modem_, modem_B_new);

    // Create message
    geometry_msgs::PoseWithCovarianceStamped modem_update;
    modem_update.header.frame_id = "modem_origin";
    modem_update.header.stamp = time_B;
    modem_update.pose = modem_B_new;


    pub_modem_position_.publish(modem_update);
    getOdomError();

///


    // geometry_msgs::PoseWithCovariance modem_A; 
    // geometry_msgs::PoseWithCovariance modem_B; 
    // pose_cov_ops::compose(odom_A.pose, sparus2modem_, modem_A);
    // pose_cov_ops::compose(odom_B.pose, sparus2modem_, modem_B);

    // double delta_time = time_B.toSec() - time_A;
    // geometry_msgs::PoseWithCovariance delta_odom;
    // pose_cov_ops::inverseCompose(modem_B, modem_A, delta_odom);

    // // Odom to modem old updated
    // geometry_msgs::PoseWithCovariance modem_A_new;
    // modem_A_new.pose.position = sparus_A.pose.pose.position;
    // modem_A_new.pose.orientation = modem_A.pose.orientation;
    // modem_A_new.covariance[0] = sparus_A.pose.covariance[0];
    // modem_A_new.covariance[7] = sparus_A.pose.covariance[7];
    // modem_A_new.covariance[14] = sparus_A.pose.covariance[14];
    // modem_A_new.covariance[21,22,23] = modem_A.covariance[21,22,23];
    // modem_A_new.covariance[27,28,29] = modem_A.covariance[27,28,29];
    // modem_A_new.covariance[33,34,35] = modem_A.covariance[33,34,35];


    // // Odom to modem actual updated
    // geometry_msgs::PoseWithCovariance modem_B_new;
    // pose_cov_ops::compose(modem_A_new, delta_odom, modem_B_new);

    // // Create message
    // geometry_msgs::PoseWithCovarianceStamped modem_update;
    // modem_update.header.frame_id = "modem_origin";
    // modem_update.header.stamp = time_B;
    // modem_update.pose = modem_B_new;


    // pub_modem_position_.publish(modem_update);
    // getOdomError();


    // ROS_INFO("------------------------------------------------------------------");
    // ROS_INFO_STREAM("ODOM    Covariance[0]   :" << odom_A.pose.covariance[0]);
    // ROS_INFO_STREAM("ODOM    Covariance[7]   :" << odom_A.pose.covariance[7]);
    // ROS_INFO_STREAM("ODOM    Covariance[14]  :" << odom_A.pose.covariance[14]);
    // ROS_INFO_STREAM("ODOM    Covariance[21]  :" << odom_A.pose.covariance[21]);
    // ROS_INFO_STREAM("ODOM    Covariance[28]  :" << odom_A.pose.covariance[28]);
    // ROS_INFO_STREAM("ODOM    Covariance[35]  :" << odom_A.pose.covariance[35]);
    // ROS_INFO("------------------------------------------------------------------");
    // ROS_INFO_STREAM("MAP     Covariance[0]   :" << ekfMap_.pose.covariance[0]);
    // ROS_INFO_STREAM("MAP     Covariance[7]   :" << ekfMap_.pose.covariance[7]);
    // ROS_INFO_STREAM("MAP     Covariance[14]  :" << ekfMap_.pose.covariance[14]);
    // ROS_INFO_STREAM("MAP     Covariance[21]  :" << ekfMap_.pose.covariance[21]);
    // ROS_INFO_STREAM("MAP     Covariance[28]  :" << ekfMap_.pose.covariance[28]);
    // ROS_INFO_STREAM("MAP     Covariance[35]  :" << ekfMap_.pose.covariance[35]);
    // ROS_INFO("------------------------------------------------------------------");
    // ROS_INFO_STREAM("d_ODOM  Covariance[0]   :" << delta_odom.covariance[0]);
    // ROS_INFO_STREAM("d_ODOM  Covariance[7]   :" << delta_odom.covariance[7]);
    // ROS_INFO_STREAM("d_ODOM  Covariance[14]  :" << delta_odom.covariance[14]);
    // ROS_INFO_STREAM("d_ODOM  Covariance[21]  :" << delta_odom.covariance[21]);
    // ROS_INFO_STREAM("d_ODOM  Covariance[28]  :" << delta_odom.covariance[28]);
    // ROS_INFO_STREAM("d_ODOM  Covariance[35]  :" << delta_odom.covariance[35]);
    // ROS_INFO("------------   ------------------------------------------------------");
    // ROS_INFO_STREAM("USBL    Covariance[0]   :" << sparus_A.pose.covariance[0]);
    // ROS_INFO_STREAM("USBL    Covariance[7]   :" << sparus_A.pose.covariance[7]);
    // ROS_INFO_STREAM("USBL    Covariance[14]  :" << sparus_A.pose.covariance[14]);
    // ROS_INFO_STREAM("USBL    Covariance[21]  :" << sparus_A.pose.covariance[21]);
    // ROS_INFO_STREAM("USBL    Covariance[28]  :" << sparus_A.pose.covariance[28]);
    // ROS_INFO_STREAM("USBL    Covariance[35]  :" << sparus_A.pose.covariance[35]);
    // ROS_INFO("------------------------------------------------------------------");
    // ROS_INFO_STREAM("USBL_up Covariance[0]   :" << modem_B.pose.covariance[0]);
    // ROS_INFO_STREAM("USBL_up Covariance[7]   :" << modem_B.pose.covariance[7]);
    // ROS_INFO_STREAM("USBL_up Covariance[14]  :" << modem_B.pose.covariance[14]);
    // ROS_INFO_STREAM("USBL_up Covariance[21]  :" << modem_B.pose.covariance[21]);
    // ROS_INFO_STREAM("USBL_up Covariance[28]  :" << modem_B.pose.covariance[28]);
    // ROS_INFO_STREAM("USBL_up Covariance[35]  :" << modem_B.pose.covariance[35]);
    // ROS_INFO("------------------------------------------------------------------");
    ROS_INFO("__________________________________________________________________");
  }


private:
  ros::NodeHandle n_;

  ros::Subscriber sub_usbl_;
  ros::Subscriber sub_ekfOdom_;
  ros::Subscriber sub_ekfMap_;
  ros::Subscriber sub_gt_;
  ros::Publisher pub_modem_position_;


  boost::mutex mutex_;
  tf::TransformBroadcaster br_;
  vector<double> timestamps_;
  vector<nav_msgs::Odometry> odomHistorial_;
  tf::TransformListener listener_;
  bool ekf_init_;
  nav_msgs::Odometry gtOdom_;
  nav_msgs::Odometry ekfMap_;
  nav_msgs::Odometry ekfOdom_;
  geometry_msgs::Pose  sparus2modem_;
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