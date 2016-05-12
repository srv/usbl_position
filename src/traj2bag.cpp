#include "ros/ros.h"

#include <control/String.h>
#include <std_srvs/Empty.h>

#include <safety/MissionStatus.h>



using namespace std;

class Traj2bag
{
public:
  Traj2bag(): end_(false), iterations_(3)
  {
    // Node name
    node_name_ = ros::this_node::getName();
    ROS_INFO_STREAM("[" << node_name_ << "]: Running");

    // Subscriber
    sub_status_ =  nh_.subscribe("/control/mission_status", 1, &Traj2bag::statusCb, this);


    loadTrajectory();
    
    for (int it = 0; it < iterations_; ++it)
    {
      enableTrajectory();

      // Start Recording
      start_recording_.call(empty_srv_);
      ROS_INFO_STREAM("[" << node_name_ << "]: /start_recording service called!");

      while (!end_)




      // Stop Recording
      stop_recording_.call(empty_srv_);
      ROS_INFO_STREAM("[" << node_name_ << "]: /stop_recording service called!");

      // Reset position
      // respawn simulation nodes
    }
  }

  void statusCb(const safety::MissionStatus& status)
  {
    if (status.current_wp == status.total_wp) 
      end_ = true;
  }

protected:

  void loadTrajectory()
  {
    // Wait for load_trajectory service
    load_trajectory_ = nh_.serviceClient<control::String>("/control/load_trajectory");
    while (!load_trajectory_.waitForExistence()) {
      ROS_INFO_STREAM_ONCE("[" << node_name_ << "]: Waiting for /control/load_trajectory service to be available.");
    }
    ros::Duration(10.0).sleep();
    ROS_INFO_STREAM("[" << node_name_ << "]: Calling /control/load_trajectory service...");

    // Load trajectory
    control::String load;
    load_trajectory_.call(load);
    ROS_INFO_STREAM("[" << node_name_ << "]: /control/enable_trajectory service called!");
  }

  void enableTrajectory()
  {
    // Wait for enable_trajectory service
    enable_trajectory_ = nh_.serviceClient<std_srvs::Empty>("/control/enable_trajectory");
    while (!enable_trajectory_.waitForExistence()) {
      ROS_INFO_STREAM_ONCE("[" << node_name_ << "]: Waiting for /control/enable_trajectory service to be available.");
    }
    ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("[" << node_name_ << "]: Calling /control/enable_trajectory service...");

    //Enable trajectory
    enable_trajectory_.call(empty_srv_);
    ROS_INFO_STREAM("[" << node_name_ << "]: /control/enable_trajectory service called!");
  }

private:
  string node_name_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber sub_status_;

  ros::ServiceClient load_trajectory_;
  ros::ServiceClient enable_trajectory_;

  ros::ServiceClient start_recording_;
  ros::ServiceClient stop_recording_;

  std_srvs::Empty empty_srv_;
  bool end_;
  int iterations_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Traj2bag");

  Traj2bag traj2bag;

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}