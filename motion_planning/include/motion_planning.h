#pragma once

#include <vector>

// Ros standards
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Msg types
#include <pir_msgs/HomographyPose.h>
#include <pir_msgs/Pose3D.h>

namespace planner
{
class MotionPlanning
{
public:
  MotionPlanning() = delete;
  MotionPlanning(const ros::NodeHandle& nh, std::string arm_name);

  void update();

  std::vector<moveit::planning_interface::MoveGroupInterface::Plan>
  plan_motion(geometry_msgs::Pose target_pose, std::vector<double> current_joint_angles);

  bool execute_motion(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan);

  void calculate_3d_to_3d_pose(double x, double y, double angle);

  void calculate_homography_based_pose(double x, double y, double angle, int object);

  void publishTrajectory(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan);

private:
  void homographyPoseCallback(const pir_msgs::HomographyPose::ConstPtr& msg);

  void pose3DCallback(const pir_msgs::Pose3D::ConstPtr& msg);

  ros::Subscriber homography_target_sub_;
  ros::Subscriber target_3d_sub_;
  ros::Publisher publish_trajectory_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  ros::NodeHandle nh_;
  geometry_msgs::Pose target_pose_homography_;
  geometry_msgs::Pose target_pose_3d_;
  bool homography_pose_received_;
  bool pose_3d_received_;
};
}  // namespace planner