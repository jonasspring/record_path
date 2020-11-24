#ifndef RECORD_PATH_H
#define RECORD_PATH_H

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <move_base_lite_msgs/FollowPathGoal.h>
#include <move_base_lite_msgs/FollowPathAction.h>

//#include <path_msgs/FollowPathGoal.h>
//#include <path_msgs/FollowPathAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Odometry.h>


class Record_path
{
public:
  Record_path(ros::NodeHandle &nh_);
  virtual ~Record_path();

  friend int main(int, char**);

  bool drive_back;
  bool send_bag;

protected:
  void poseCallback(const geometry_msgs::PoseStamped pose);
  void trackerCallback(const geometry_msgs::PoseStamped tracker_pose);
  void pathCallback(const ros::MessageEvent<nav_msgs::Path>& event);
  void sendPathFromBag();

  geometry_msgs::Point getClosestPoint(geometry_msgs::Point path_point1, geometry_msgs::Point path_point2, geometry_msgs::Point robot);
  geometry_msgs::PoseStamped getPathPose(nav_msgs::Path path, geometry_msgs::PoseStamped pose);

  void followPathPreemptCallback(move_base_lite_msgs::FollowPathActionResult result);
  void speedCmdCallback(const geometry_msgs::Twist cmd);

private:
  ros::NodeHandle nh;
  ros::Subscriber poseSubscriber;
  ros::Subscriber pathSubscriber;
  ros::Subscriber trackerSubscriber;

  ros::Publisher goalPublisher;

  rosbag::Bag bag;

  std::string bag_name;

  nav_msgs::Path current_path;
  nav_msgs::Path empty_path;
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped last_pose;

  geometry_msgs::PoseStamped current_tracker_pose;
  geometry_msgs::PoseStamped last_tracker_pose;

  move_base_lite_msgs::FollowPathGoal send_goal;

  ros::Time start_time;
  ros::Time end_time;

  double total_distance_driven;
  double total_distance_driven_tracker;

 // boost::shared_ptr<actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction> > follow_path_server_;

  ros::Subscriber preemptSubscriber;
  ros::Subscriber speedSubscriber;

  boost::shared_ptr<actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction> > follow_path_client_;

//  path_msgs::FollowPathGoal send_goal2;
  //actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>  follow_path_client_;

  std::string path_file;
  std::string output_directory;
  double commanded_speed;

};

#endif // RECORD_PATH_H
