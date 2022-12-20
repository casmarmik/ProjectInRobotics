#include "motion_planning.h"

#include <thread>

// Ros standard
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>

// Msg types
#include <pir_msgs/HomographyPose.h>

using namespace planner;

int main(int argc, char** argv)
{
  // Start ROS
  ros::init(argc, argv, "motion_planning_kuka");
  ros::NodeHandle nh("~");
  MotionPlanning planner(nh, "kuka_arm");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok())
  {
    planner.update();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}