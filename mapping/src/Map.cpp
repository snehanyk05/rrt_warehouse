//
// Created by naivehobo on 11/9/19.
//

#include "mapping/Map.h"
#include "ros/time.h"
#include "ros/ros.h"

Map::Map() : private_nh_("~") {

  private_nh_.param<std::string>("map_topic", map_topic_, "map");
  private_nh_.param("height", height_, 500);
  private_nh_.param("width", width_, 500);
  private_nh_.param("resolution", resolution_, 0.1f);

  std::string map_path;
  private_nh_.param<std::string>("load_map", map_path, "none");

  window_ = "Map";

  if(map_path == "none")
    map_ = cv::Mat::zeros(height_, width_, CV_8UC3);
  else {
    map_ = cv::imread(map_path);
    cv::resize(map_,map_, cv::Size(width_*10, height_*10));
    height_ = map_.rows;
    width_ = map_.cols;
    ROS_INFO("Loaded map from: %s", map_path.c_str());
  }

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1);

  ROS_INFO("Mapping node initialized");

if (ros::ok())
{
  sleep(60);
  publishMap();
}

}

Map::~Map() {
  cv::destroyWindow(window_);
}

void Map::publishMap() {

  nav_msgs::OccupancyGrid grid;

  grid.info.map_load_time = ros::Time::now();
  grid.header.frame_id = "map";
  grid.header.stamp = ros::Time::now();

  grid.info.width = (unsigned int) width_;
  grid.info.height = (unsigned int) height_;
  grid.info.resolution = resolution_;

  grid.info.origin.position.x = 0 ;
  grid.info.origin.position.y = height_;
  grid.info.origin.position.z = 0.0;

  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(grid.info.width * grid.info.height);
  
  cv::Mat src = map_.clone();
  cv::Mat se = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * 1.6, 2 * 1.6));
  erode(src, map_, se, cv::Point(-1, -1), 4);
  


  for(unsigned int i = 0; i < grid.info.height; i++) {
    for (unsigned int j = 0; j < grid.info.width; j++) {
      int val = map_.at<cv::Vec3b>(i, j).val[0] > 0 ? 0 : 100;
      grid.data[i * grid.info.width + j] = (char) val;
    }
  }
  // std::cout<<width_<<" ";
  // cv::circle(map_, cv::Point((int)grid.info.origin.position.x, (int)grid.info.origin.position.y), 4, cv::Scalar(0, 0, 255), -1); //red
  // cv::circle(map_, cv::Point((int)width_, (int)0), 4, cv::Scalar(0, 255, 255), -1); //red
  // cv::imshow(window_, map_);
  // cv::waitKey(100);

  map_pub_.publish(grid);
  ROS_INFO("Published map!");
}
