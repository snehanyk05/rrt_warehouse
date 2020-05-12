//
// Created by naivehobo on 11/9/19.
//
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "mapping/Map.h"
#include "ros/time.h"
#include "ros/ros.h"
#include "mapping/Num.h"

using namespace cv;
using namespace std;
// RNG rng(12345);
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

  map_pub_ = nh_.advertise<mapping::Num>(map_topic_, 1);

  ROS_INFO("Mapping node initialized");

if (ros::ok())
{
  sleep(2);
  publishMap();
}

}

Map::~Map() {
  cv::destroyWindow(window_);
}

void Map::publishMap() {
  mapping::Num obst;
  // std::cout<<obst;
  // nav_msgs::OccupancyGrid grid;

  // grid.info.map_load_time = ros::Time::now();
  // grid.header.frame_id = "map";
  // grid.header.stamp = ros::Time::now();

  // grid.info.width = (unsigned int) width_;
  // grid.info.height = (unsigned int) height_;
  // grid.info.resolution = resolution_;

  // grid.info.origin.position.x = 0 ;
  // grid.info.origin.position.y = height_;
  // grid.info.origin.position.z = 0.0;

  // grid.info.origin.orientation.x = 0.0;
  // grid.info.origin.orientation.y = 0.0;
  // grid.info.origin.orientation.z = 0.0;
  // grid.info.origin.orientation.w = 1.0;

  // grid.data.resize(grid.info.width * grid.info.height);
  
  cv::Mat src = map_.clone();
  cv::Mat se = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * 1.6, 2 * 1.6));
  erode(src, map_, se, cv::Point(-1, -1), 4);
  

  // std::vector<Vertex> obst;
  
Mat src_gray;
  cvtColor( map_, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );
  cv::Mat canny_output;
  Canny( src_gray, canny_output, 100, 100*2, 3 );

    // cv::imshow("TEST", canny_output);
  // cv::waitKey(10);
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
      //  std::cout<<"Cont size";
      // std::cout<<contours[i];
      // CvRect boundbox;
      // boundbox = boundingRect(contours[i]);
      // ROS_INFO("Loaded map from: %d", boundbox.x);


       for( int j = 0; j< contours[i].size(); j++ )
       {
         geometry_msgs::Point p;
          p.x = (contours[i][j].x)/10;
          p.y =  (600 - contours[i][j].y)/10;
          obst.obst.push_back(p);
          
          // sleep(2);
       }


  //      for(unsigned int i = 0; i < height_; i++) {
  //   for (unsigned int j = 0; j < width_; j++) {
  //     int val = contours[i].at<cv::Vec3b>(i, j).val[0] > 0 ? 100 : 0;
  //     if(val == 100)
  //     {
  //       geometry_msgs::Point p;
  //     //   p.x = (path[i].x)/10;
  //     // p.y =  (600 - path[i].y)/10;
  //       p.x = j/10;
  //       p.y = (600 - i)/10;
  //       obst.obst.push_back(p);
      
  //     }
  //     // grid.data[i * width + j] = (char) val;
  //   }
  // }
  
  map_pub_.publish(obst);
  obst.obst.clear();
  sleep(5);
     }
  
  // // std::cout<<width_<<" ";
  // // cv::circle(map_, cv::Point((int)grid.info.origin.position.x, (int)grid.info.origin.position.y), 4, cv::Scalar(0, 0, 255), -1); //red
  // // cv::circle(map_, cv::Point((int)width_, (int)0), 4, cv::Scalar(0, 255, 255), -1); //red
  // cv::imshow("TEST", drawing);
  // cv::waitKey(10);


//   map_pub_.publish(obst);
  ROS_INFO("Published map!");
}
