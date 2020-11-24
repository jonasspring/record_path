#include "record_path/record_path.h"
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>

Record_path::Record_path(ros::NodeHandle& nh_)
{
  nh = nh_;

  drive_back = false;
  send_bag = false;

  nh.getParam("path_file", path_file);
  nh.getParam("output_directory", output_directory);
  nh.getParam("send_bag", send_bag);
  nh.getParam("drive_back", drive_back);
  nh.getParam("speed", commanded_speed);

  time_t rawtime;
  struct tm * timeinfo;
  char buffer [80];

  time ( &rawtime );

  timeinfo = localtime ( &rawtime );
  strftime (buffer,80,"%Y-%m-%d-T%H-%M-%S",timeinfo);
  std::string name(buffer);
  bag_name = name;

  bag.open((output_directory + bag_name + ".bag"), rosbag::bagmode::Write);
  //bag.open("test.bag", rosbag::bagmode::Write);

  start_time = ros::Time::now();
  end_time = ros::Time::now();

  total_distance_driven = 0;
  total_distance_driven_tracker = 0;

  current_path = empty_path;

  poseSubscriber = nh.subscribe("/robot_pose", 20, &Record_path::poseCallback, this);
  trackerSubscriber = nh.subscribe("/tracker/base_pose_reference", 20, &Record_path::trackerCallback, this);
  pathSubscriber = nh.subscribe("/smooth_path", 20, &Record_path::pathCallback, this);

  goalPublisher = nh.advertise<move_base_lite_msgs::FollowPathActionGoal>("/controller/follow_path/goal", 10, true);

  preemptSubscriber = nh.subscribe("/controller/follow_path/result", 10, &Record_path::followPathPreemptCallback, this);

  speedSubscriber = nh.subscribe("/cmd_vel_raw", 20, &Record_path::speedCmdCallback, this);

  follow_path_client_.reset(new actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>("/controller/follow_path", false));

  if(send_bag){
    ROS_INFO("Send path from bag file");
    sendPathFromBag();
  }

  if(drive_back){
    ROS_INFO("Drive back to start after finishing");
  }

}

Record_path::~Record_path(){
  bag.close();
}

void Record_path::poseCallback(const geometry_msgs::PoseStamped pose){

  last_pose = current_pose;
  current_pose = pose;

  if(!current_path.poses.empty()){
      bag.write("path_pose", ros::Time::now(), getPathPose(current_path, current_pose));

      double dist = sqrt(std::pow(current_pose.pose.position.x - last_pose.pose.position.x, 2) + std::pow(current_pose.pose.position.y - last_pose.pose.position.y , 2)
                         + std::pow(current_pose.pose.position.z - last_pose.pose.position.z , 2));
      total_distance_driven += dist;
  }
  if(!send_goal.target_path.poses.empty()){
      bag.write("goal_pose", ros::Time::now(), getPathPose(send_goal.target_path, current_pose));
  }

  bag.write("robot_pose", ros::Time::now(), current_pose);
}

void Record_path::trackerCallback(const geometry_msgs::PoseStamped tracker_pose){

  last_tracker_pose = current_tracker_pose;
  current_tracker_pose = tracker_pose;

  if(!current_path.poses.empty()){
      bag.write("path_pose_tracker", ros::Time::now(), getPathPose(current_path, current_tracker_pose));

      double dist = sqrt(std::pow(current_tracker_pose.pose.position.x - last_tracker_pose.pose.position.x, 2)
                         + std::pow(current_tracker_pose.pose.position.y - last_tracker_pose.pose.position.y , 2)
                         + std::pow(current_tracker_pose.pose.position.z - last_tracker_pose.pose.position.z , 2));
      total_distance_driven_tracker += dist;
  }
  if(!send_goal.target_path.poses.empty()){
      bag.write("goal_pose_tracker", ros::Time::now(), getPathPose(send_goal.target_path, current_tracker_pose));
  }

  bag.write("tracker_pose", ros::Time::now(), current_tracker_pose);
}

void Record_path::pathCallback(const ros::MessageEvent<nav_msgs::Path>& event){

  nav_msgs::PathConstPtr path = event.getConstMessage();
  current_path = *path;

  bag.write("smooth_path", ros::Time::now(), path);
}

//Sends a path from a bag file containing one instance of the topic /smooth_path to the vehicle controller
void Record_path::sendPathFromBag(){

  rosbag::Bag goal_bag;
  goal_bag.open(path_file, rosbag::bagmode::Read);

  for(rosbag::MessageInstance const m: rosbag::View(goal_bag)){
    //    move_base_lite_msgs::FollowPathActionGoal::ConstPtr goal = m.instantiate<move_base_lite_msgs::FollowPathActionGoal>();

    //    send_goal.target_path = goal->goal.target_path;
    //    send_goal.follow_path_options = goal->goal.follow_path_options;

    nav_msgs::Path::ConstPtr smooth_path = m.instantiate<nav_msgs::Path>();

    send_goal.target_path = *smooth_path;

  }
  send_goal.follow_path_options.desired_speed = commanded_speed;
  send_goal.follow_path_options.is_fixed = false;
  send_goal.follow_path_options.goal_pose_position_tolerance = 0.1;
  send_goal.follow_path_options.goal_pose_angle_tolerance = 3.0;
  send_goal.follow_path_options.rotate_front_to_goal_pose_orientation = false;
  send_goal.follow_path_options.reverse_allowed = true;
  send_goal.follow_path_options.reverse_forced = false;
  send_goal.follow_path_options.reset_stuck_history = true;

  ros::Time time = ros::Time::now();

  send_goal.target_path.header.stamp = time;

  for (int i=0; i< send_goal.target_path.poses.size(); i++){
    send_goal.target_path.poses[i].header.stamp = time;
  }

    send_goal.target_path.header.stamp = time;

    for (int i=0; i< send_goal.target_path.poses.size(); i++){
      send_goal.target_path.poses[i].header.stamp = time;
    }


  follow_path_client_->sendGoal(send_goal);
  start_time = ros::Time::now();

  ROS_INFO("Sent");
  goal_bag.close();

}

geometry_msgs::Point Record_path::getClosestPoint(geometry_msgs::Point path_point1, geometry_msgs::Point path_point2, geometry_msgs::Point robot){
  double l1 = path_point2.x - path_point1.x;
  double l2 = path_point2.y - path_point1.y;

  double r1 = -l2;
  double r2 = l1;

  double lambda = (l2 * (path_point1.x - robot.x) + l1 * (robot.y - path_point1.y) ) / (r1*l2 - r2*l1);

  geometry_msgs::Point closestPoint;
  closestPoint.x = robot.x + lambda * r1;
  closestPoint.y = robot.y + lambda * r2;

  return closestPoint;
}

//return closest Pose on the interpolated path to the current robot pose
geometry_msgs::PoseStamped Record_path::getPathPose(nav_msgs::Path path, geometry_msgs::PoseStamped pose){
  int current = 0;
  double shortest_dist = 999999;
  for(int i = 0; i<path.poses.size(); i++){
    double dist = std::sqrt(std::pow(pose.pose.position.x - path.poses[i].pose.position.x, 2)
                           + std::pow(pose.pose.position.y - path.poses[i].pose.position.y, 2));
    if (dist < shortest_dist){
      shortest_dist = dist;
      current = i;
    }
  }

  int second = current;
  if (current == 0){
    second = 1;
  }
  else{
    if ((current + 1) < path.poses.size()){
      double prev_dx = (path.poses[current - 1].pose.position.x - path.poses[current].pose.position.x);
      double prev_dy = (path.poses[current - 1].pose.position.y - path.poses[current].pose.position.y);
      double next_dx = (path.poses[current + 1].pose.position.x - path.poses[current].pose.position.x);
      double next_dy = (path.poses[current + 1].pose.position.y - path.poses[current].pose.position.y);
      double robot_dx = (pose.pose.position.x - path.poses[current].pose.position.x);
      double robot_dy = (pose.pose.position.y - path.poses[current].pose.position.y);

      double angle_prev = std::acos((prev_dx*robot_dx + prev_dy*robot_dy) / (sqrt(prev_dx*prev_dx + prev_dy*prev_dy) + sqrt(robot_dx*robot_dx + robot_dy*robot_dy) ));
      double angle_next = std::acos((next_dx*robot_dx + next_dy*robot_dy) / (sqrt(next_dx*next_dx + next_dy*next_dy) + sqrt(robot_dx*robot_dx + robot_dy*robot_dy) ));

      if (fabs(angle_prev) < fabs(angle_next)){
        second = current - 1;
      }
      else{
        second = current + 1;
      }
    }
    else{
      second = current - 1;
    }
  }
  geometry_msgs::PoseStamped path_pose;
  path_pose.pose.position = getClosestPoint(path.poses[current].pose.position, path.poses[second].pose.position, pose.pose.position);

  path_pose.pose.position.z = pose.pose.position.z;
  path_pose.header.stamp = pose.header.stamp;
  path_pose.header.frame_id = pose.header.frame_id;

  return path_pose;
}

void Record_path::followPathPreemptCallback(move_base_lite_msgs::FollowPathActionResult result)
{
  end_time = ros::Time::now();
  double total_time = (end_time - start_time).toSec();
  ROS_INFO("Time needed: %f seconds", total_time);
  ROS_INFO("Total distance driven: %f", total_distance_driven);
  ROS_INFO("average speed: %f", total_distance_driven/total_time);

  std_msgs::Float64 total_dist_msg;
  total_dist_msg.data = total_distance_driven;

  std_msgs::Float64 total_dist_tracker_msg;
  total_dist_tracker_msg.data = total_distance_driven_tracker;

  std_msgs::Float64 avg_speed_msg;
  avg_speed_msg.data = total_distance_driven/total_time;

  bag.write("total_distance", ros::Time::now(), total_dist_msg);
  bag.write("total_distance_tracker", ros::Time::now(), total_dist_tracker_msg);
  bag.write("average speed", ros::Time::now(), avg_speed_msg);

  if (send_bag){
    if (drive_back){
      bag.close();
      std::string bag_name_backwards = bag_name;
      bag_name_backwards.append("_backwards");
      bag.open((output_directory + bag_name_backwards + ".bag"), rosbag::bagmode::Write);

      nav_msgs::Path path_backwards;
      for (int i = send_goal.target_path.poses.size() - 1; i >= 0; i--){
        path_backwards.poses.push_back(send_goal.target_path.poses[i]);
      }

      ROS_INFO("Path finished, driving back to start");
      send_goal.target_path.poses = path_backwards.poses;
      follow_path_client_->sendGoal(send_goal);
      start_time = ros::Time::now();
      total_distance_driven = 0;

      drive_back = false;
    }
    else{
      ROS_INFO("Recording finished");
      ros::shutdown();
    }
  }
}

void Record_path::speedCmdCallback(const geometry_msgs::Twist cmd){
  bag.write("cmd_vel_raw", ros::Time::now(), cmd);
}


int main(int argc, char **argv)
{
  std::string param;

  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle nh("~");

  Record_path rec(nh);

  //if (nh.getParam("bag", param)){
    //ROS_INFO("Send path from bag file");
    //rec.sendPathFromBag();
    //rec.send_bag = true;
  //}

//  if (nh.getParam("driveBack", param)){
//    ROS_INFO("Drive back to start after finishing");
//    rec.drive_back = true;
//  }

  while(ros::ok())
  {
    ros::spin();
  }

  ros::shutdown();
  return 0;
}
