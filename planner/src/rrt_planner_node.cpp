//
// Created by naivehobo on 11/7/19.
//

#include <ros/ros.h>

#include "planner/rrt_planner/RRTPlanner.h"
#include <nav_msgs/Path.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh;
  auto planner = RRTPlanner();
  nav_msgs::Path path;
  std::vector<Vertex> plan ;
  ros::Rate loop_rate(10);
  // bool start_flag = false;
  ros::Publisher path_pub;
  path_pub = nh.advertise<nav_msgs::Path>("nav_path", 10);
  planner.start_flag = false;
  while(ros::ok())
    {
      // std::cout<<"START FLAG "<<planner.start_flag;
        if(planner.start_flag)
        {
            // Start planning path
            plan = planner.queryPlan();
            if(!plan.empty())
            {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();
                for(int i=0;i<plan.size();i++)
                {
                    
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = plan[i].x;
                    pose_stamped.pose.position.y = plan[i].y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);

                ROS_INFO("Find a valid path successfully");
            }
            else
            {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();
                path_pub.publish(path);
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            planner.start_flag = false;
        }

        

        loop_rate.sleep();
        ros::spinOnce();
    }
  ros::spin();

  return 0;
}