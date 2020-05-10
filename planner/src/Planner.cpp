//
// Created by naivehobo on 11/10/19.
//

#include "planner/Planner.h"
#include "planner/Num.h"
#include <bits/stdc++.h> 
#include <boost/algorithm/string.hpp>

Planner::Planner() : private_nh_("~") {
  private_nh_.param<std::string>("map_topic", map_topic_, "map");
  private_nh_.param<std::string>("pose_topic", pose_topic_, "pose");
  private_nh_.param<std::string>("goal_topic", goal_topic_, "goal");
  private_nh_.param<std::string>("map_update", map_update_, "map_update");
  private_nh_.param("height", height_, 500);
  pose_set_ = false;

  pose_sub_mb = nh_.subscribe("initpose", 1, &Planner::poseMBCallback, this);
  goal_sub_mb = nh_.subscribe("finalgoal", 1, &Planner::goalMBCallback, this);
  
  map_sub_ = nh_.subscribe(map_topic_, 1, &Planner::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic_, 1, &Planner::goalCallback, this);
  pose_sub_ = nh_.subscribe(pose_topic_, 1, &Planner::poseCallback, this);
  map_update_sub = nh_.subscribe(map_update_, 1, &Planner::updateMapCallback, this);
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
  map_.copyTo(og_map_);
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
  
  // for (int i = 0; i < height_; i++) {
  //   for (int j = 0; j < 1000; j++) {
  //     if( (i <=(goal_mb.y+1) && i> (goal_mb.y-1)) && (j <=(goal_mb.x+1) && j> (goal_mb.y-1)))
  //     {
  //     unsigned char value = (unsigned char) 0;
  //     map_.at<cv::Vec3b>(i, j) = cv::Vec3b(value, value, value);
  //     }
     
      

  //   }
  // }
  // cv::circle(map_, cv::Point((int)goal_mb.x, (int)goal_mb.y), 4, cv::Scalar(0, 0, 255), -1);
  // cv::imshow("Map", map_);
  // cv::waitKey(100);
  map_.copyTo(display_map_);
  publishGoal();
}

void Planner::poseMBCallback(const geometry_msgs::PoseStamped::ConstPtr &pose) {
  // ROS_INFO("Received possgfgsfghs");
  pose_mb.x = (pose->pose.position.x*10);
  pose_mb.y = (height_ - pose->pose.position.y*10);
  ROS_INFO("Recieved pose: (%lf, %lf)", pose->pose.position.x, pose->pose.position.y);
  // cv::circle(map_, cv::Point((int)pose_mb.x, (int)pose_mb.y), 4, cv::Scalar(0, 255, 0), -1);
  // cv::imshow("Map", map_);
  // cv::waitKey(100);

  // for (int i = 0; i < height_; i++) {
  //   for (int j = 0; j < 1000; j++) {
  //     if( (i <=(pose_mb.y+1) && i> (pose_mb.y-1)) && (j <=(pose_mb.x+1) && j> (pose_mb.y-1)))
  //     {
  //     unsigned char value = (unsigned char) 0;
  //     map_.at<cv::Vec3b>(i, j) = cv::Vec3b(value, value, value);
  //     }
     
      

  //   }
  // }
  map_.copyTo(display_map_);
  publishPose();
}

void Planner::updateMapCallback(const planner::Num &update) {
  // ROS_INFO("Recieved update: (%s)",update.data*);
  // ROS_INFO("%s\n", update.data[0].c_str());
  og_map_.copyTo(map_);
  
  for(int k = 0; k< update.data.size(); k++)
  {
  std::string data = update.data[k];  
  std::vector<std::string> result; 
  boost::split(result,data, boost::is_any_of(",")); 
  // ROS_INFO("%d\n", std::stoi(result[0]));
  int x = std::stoi(result[0])*10;
  int y = height_ -  ((std::stoi(result[1]))*10);

  for (int i = 0; i < height_; i++) {
    for (int j = 0; j < 1000; j++) {
      if( (i <=(y+1) && i> (y-1)) && (j <=(x+1) && j> (x-1)))
      {
      unsigned char value = (unsigned char) 255;
      map_.at<cv::Vec3b>(i, j) = cv::Vec3b(value, value, value);
      }
     
      

    }
  }
  }
  map_.copyTo(display_map_);


  // std::cout<<update.data[0];
//   pose_mb.x = (pose->pose.position.x*10);
//   pose_mb.y = (height_ - pose->pose.position.y*10);
//   ROS_INFO("Recieved pose: (%lf, %lf)", pose->pose.position.x, pose->pose.position.y);
//   // cv::circle(map_, cv::Point((int)pose_mb.x, (int)pose_mb.y), 4, cv::Scalar(0, 255, 0), -1);
  // cv::imshow("Map", map_);
  // cv::waitKey(100);
//   publishPose();
}
