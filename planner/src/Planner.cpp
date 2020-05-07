//
// Created by naivehobo on 11/10/19.
//

#include "planner/Planner.h"


Planner::Planner() : private_nh_("~") {
  private_nh_.param<std::string>("map_topic", map_topic_, "map");
  private_nh_.param<std::string>("pose_topic", pose_topic_, "pose");
  private_nh_.param<std::string>("goal_topic", goal_topic_, "goal");
  private_nh_.param("height", height_, 500);
  pose_set_ = false;

  pose_sub_mb = nh_.subscribe("initpose", 1, &Planner::poseMBCallback, this);
  goal_sub_mb = nh_.subscribe("finalgoal", 1, &Planner::goalMBCallback, this);
  
  map_sub_ = nh_.subscribe(map_topic_, 1, &Planner::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic_, 1, &Planner::goalCallback, this);
  pose_sub_ = nh_.subscribe(pose_topic_, 1, &Planner::poseCallback, this);

  // std::cout<<map_topic_;
  goal_pub_ = nh_.advertise<geometry_msgs::Pose2D>(goal_topic_, 1);
  pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(pose_topic_, 1);

  height_ = height_*10;
  // cv::namedWindow("Map");
  ROS_INFO("Planner node initialized");
}

void Planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  map_ = cv::Mat::zeros(map->info.height, map->info.width, CV_8UC3);
  for (int i = 0; i < map->info.height; i++) {
    for (int j = 0; j < map->info.width; j++) {
      unsigned char value = map->data[i * map->info.width + j] != 0 ? (unsigned char) 255 : (unsigned char) 0;
      map_.at<cv::Vec3b>(i, j) = cv::Vec3b(value, value, value);
    }
  }
  map_.copyTo(display_map_);
  ROS_INFO("Received map");
}

void Planner::goalCallback(const geometry_msgs::Pose2D::ConstPtr &goal) {
  goal_.x = (int) goal->x;
  goal_.y = (int) goal->y;
  ROS_INFO("Received goal: (%d, %d)", goal_.x, goal_.y);
  if(pose_set_ && !map_.empty())
    getPlan();
}

void Planner::poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose) {
  
  pose_.x = (int) pose->x;
  pose_.y = (int) pose->y;
  pose_set_ = true;
  ROS_INFO("Received pose : (%d, %d)", pose_.x, pose_.y);
}


void Planner::publishPose() {
  pose_pub_.publish(pose_mb);
  ROS_INFO("Published starting pose: (%lf, %lf)", pose_mb.x, pose_mb.y);
}

void Planner::publishGoal() {
  goal_pub_.publish(goal_mb);
  ROS_INFO("Published goal: (%lf, %lf)", goal_mb.x, goal_mb.y);
}

void Planner::goalMBCallback(const geometry_msgs::PoseStamped::ConstPtr &goal) {
  goal_mb.x = (goal->pose.position.x*10);
  goal_mb.y = (height_ - goal->pose.position.y*10);
  ROS_INFO("Recieved goal: (%lf, %lf)", goal->pose.position.x, goal->pose.position.y);
  // cv::circle(map_, cv::Point((int)goal_mb.x, (int)goal_mb.y), 4, cv::Scalar(0, 0, 255), -1);
  // cv::imshow("Map", map_);
  // cv::waitKey(100);
  publishGoal();
}

void Planner::poseMBCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
  ROS_INFO("Received possgfgsfghs");
  pose_mb.x = (pose->pose.position.x*10);
  pose_mb.y = (height_ - pose->pose.position.y*10);
  ROS_INFO("Recieved pose: (%lf, %lf)", pose->pose.position.x, pose->pose.position.y);
  // cv::circle(map_, cv::Point((int)pose_mb.x, (int)pose_mb.y), 4, cv::Scalar(0, 255, 0), -1);
  // cv::imshow("Map", map_);
  // cv::waitKey(100);
  publishPose();
}
