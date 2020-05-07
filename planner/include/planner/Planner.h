//
// Created by naivehobo on 11/10/19.
//

#ifndef RRT_PLANNER_PLANNER_H
#define RRT_PLANNER_PLANNER_H

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#include "planner/Graph.h"


class Planner {
 public:

  Planner();
std::string map_topic_;
  virtual std::vector<Vertex> getPlan() = 0;

 protected:
  Vertex goal_;
  Vertex pose_;

  cv::Mat map_;
  cv::Mat display_map_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

 private:
  void publishGoal();
  void publishPose();
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
  void goalCallback(const geometry_msgs::Pose2D::ConstPtr &goal);
  void poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose);
  void goalMBCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);
  void poseMBCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);

  ros::Publisher pose_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber goal_sub_mb;
  ros::Subscriber pose_sub_mb;

  
  std::string goal_topic_;
  std::string pose_topic_;

  bool pose_set_;

  geometry_msgs::Pose2D goal_mb;
  geometry_msgs::Pose2D pose_mb;
  int height_;
  int width_;
  float resolution_;
};

#endif //RRT_PLANNER_PLANNER_H
