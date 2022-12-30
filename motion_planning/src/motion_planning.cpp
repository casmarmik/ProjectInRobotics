#include "motion_planning.h"
#include <Eigen/Geometry>
#include <trajectory_msgs/JointTrajectory.h>

namespace planner
{
MotionPlanning::MotionPlanning(const ros::NodeHandle& nh, std::string arm_name)
{
  nh_ = nh;
  arm_.reset(new moveit::planning_interface::MoveGroupInterface(arm_name));
  arm_->setNumPlanningAttempts(10);
  std::cout << "Planning ID: " << arm_->getPlannerId() << std::endl;

  // I think this can be used to set how fast the robot will move
  // TODO maybe test this
  arm_->setMaxVelocityScalingFactor(0.05);
  arm_->setMaxAccelerationScalingFactor(0.05);

  homography_target_sub_ = nh_.subscribe<pir_msgs::HomographyPose>("/motion_planning/homography_pose", 1000,
                                                                   &MotionPlanning::homographyPoseCallback, this);

  target_3d_sub_ =
      nh_.subscribe<pir_msgs::Pose3D>("/motion_planning/pose3D", 1000, &MotionPlanning::pose3DCallback, this);

  publish_trajectory_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/POSES", 100);

  homography_pose_received_ = false;
  pose_3d_received_ = false;
}

void MotionPlanning::update()
{
  if (homography_pose_received_)
  {
    std::vector<double> current_joints;
    current_joints.push_back(0);
    current_joints.push_back(-1.57);
    current_joints.push_back(1.57);
    current_joints.push_back(0);
    current_joints.push_back(0);
    current_joints.push_back(0);
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan =
        plan_motion(target_pose_homography_, current_joints);
    execute_motion(plan);
    publishTrajectory(plan);
    std::cout << "published code" << std::endl;
    homography_pose_received_ = false;
  }
  else if (pose_3d_received_)
  {
    std::vector<double> current_joints;
    current_joints.push_back(0);
    current_joints.push_back(-1.57);
    current_joints.push_back(1.57);
    current_joints.push_back(0);
    current_joints.push_back(0);
    current_joints.push_back(0);
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan =
        plan_motion(target_pose_3d_, current_joints);
    execute_motion(plan);
    publishTrajectory(plan);
    pose_3d_received_ = false;
  }
}

std::vector<moveit::planning_interface::MoveGroupInterface::Plan>
MotionPlanning::plan_motion(geometry_msgs::Pose target_pose, std::vector<double> current_joint_angles)
{
  // Used to store the trajectory that the robot should execute
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan;
  moveit::planning_interface::MoveGroupInterface::Plan cur_plan;

  // Setup robot state
  const robot_state::JointModelGroup* joint_model_group = arm_->getCurrentState()->getJointModelGroup("kuka_arm");
  moveit::core::RobotStatePtr robot_state = arm_->getCurrentState();

  arm_->setStartState(*robot_state);
  arm_->setJointValueTarget(current_joint_angles);
  bool success = (arm_->plan(cur_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    arm_->execute(cur_plan);
  }
  else
  {
    std::cerr << "Failed to plan a trajectory to the robot's real position" << std::endl;
    return {};
  }

  // Set start state for the planner
  robot_state->setJointGroupPositions(joint_model_group, current_joint_angles);
  arm_->setStartState(*robot_state);

  // Move to pre grasp position
  geometry_msgs::Pose pre_grasp_pose = target_pose;
  pre_grasp_pose.position.z = target_pose.position.z + 0.1;
  std::cout << pre_grasp_pose << std::endl;
  arm_->setPoseTarget(pre_grasp_pose, "tool0");
  success = (arm_->plan(cur_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    plan.push_back(cur_plan);
  }
  else
  {
    std::cerr << "Failed to plan a trajectory from the robot's current position to pre grasp position" << std::endl;
    return {};
  }

  // Set start state for the planner to end position of last trajectory
  std::vector<double> end_joint_positions = cur_plan.trajectory_.joint_trajectory.points.back().positions;
  robot_state->setJointGroupPositions(joint_model_group, end_joint_positions);
  arm_->setStartState(*robot_state);

  // Move to grasp position
  arm_->setPoseTarget(target_pose, "tool0");
  success = (arm_->plan(cur_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    plan.push_back(cur_plan);
    // TODO store the motion in a vector to publish afterwards
  }
  else
  {
    std::cerr << "Failed to plan a trajectory from pre grasp position to grasp position" << std::endl;
    return {};
  }
  std::cout << "done calculating plan" << std::endl;
  return plan;
}

bool MotionPlanning::execute_motion(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan)
{
  for (unsigned int i = 0; i < plan.size(); ++i)
  {
    arm_->execute(plan[i]);
  }
  return true;
}

void MotionPlanning::publishTrajectory(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plan)
{
  trajectory_msgs::JointTrajectory joint_traj;
  joint_traj.joint_names = plan[0].trajectory_.joint_trajectory.joint_names;
  joint_traj.header = plan[0].trajectory_.joint_trajectory.header;
  double cur_time = 0.0;
  for (unsigned int i = 0; i < plan.size(); ++i)
  {
    for (unsigned int j = 0; j < plan[i].trajectory_.joint_trajectory.points.size(); ++j)
    {
      trajectory_msgs::JointTrajectoryPoint cur_point;
      if (i > 0)  // New trajectory means zeroing the time, this is fixed with this, keeping the time counting up
      {
        if (j == 0)
        {
          cur_time = cur_time + 1;
        }
        else if (j > 0)
        {
          cur_time = cur_time + 2;
        }
      }
      else
      {  // First trajectory just use the tome from this
        // cur_time = plan[i].trajectory_.joint_trajectory.points[j].time_from_start.toSec();
        cur_time = cur_time + 1;
      }
      for (unsigned int k = 0; k < plan[i].trajectory_.joint_trajectory.points[j].positions.size(); ++k)
      {
        cur_point.positions.push_back(plan[i].trajectory_.joint_trajectory.points[j].positions[k]);
        cur_point.velocities.push_back(plan[i].trajectory_.joint_trajectory.points[j].velocities[k]);
        cur_point.accelerations.push_back(plan[i].trajectory_.joint_trajectory.points[j].accelerations[k]);
      }
      cur_point.time_from_start.fromSec(cur_time);
      joint_traj.points.push_back(cur_point);
    }
  }
  publish_trajectory_.publish(joint_traj);
}

void MotionPlanning::calculate_3d_to_3d_pose(double x, double y, double angle)
{
  // TODO calculate this

  // This makes the z-axis point downwards
  Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
  rot(2, 2) = -1;
  rot(0, 0) = -1;
  rot(1, 1) = 1;

  // Build rotation matrix from angle rotated around the z axis
  Eigen::Matrix3d rot_angle = Eigen::Matrix3d::Identity();
  rot_angle(0, 0) = cos(angle);
  rot_angle(0, 1) = -sin(angle);
  rot_angle(1, 0) = sin(angle);
  rot_angle(1, 1) = cos(angle);

  // compute final orientation
  rot = rot * rot_angle;
}

void MotionPlanning::calculate_homography_based_pose(double x, double y, double angle, int object)
{
  std::cout << "angle: " << angle << std::endl;
  angle = angle * M_PI/180;
  std::cout << "angle: " << angle << std::endl;
  target_pose_homography_.position.x = x;
  target_pose_homography_.position.y = y;
  if (object == 0)  // screw
  {
    target_pose_homography_.position.z = 115.79 / 1000;
  }
  else if (object == 1)  // plug
  {
    target_pose_homography_.position.z = 114.80 / 1000;
    angle = -1.57;
    target_pose_homography_.position.y = y + 0.015;
  }

  std::cout << target_pose_homography_ << std::endl;

  // This makes the z-axis point downwards
  Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
  rot(0, 1) = -1;
  rot(1, 0) = -1;
  rot(2, 2) = -1;

  // Build rotation matrix from angle rotated around the z axis
  Eigen::Matrix3d rot_angle = Eigen::Matrix3d::Identity();
  rot_angle(0, 0) = cos(angle);
  rot_angle(0, 1) = -sin(angle);
  rot_angle(1, 0) = sin(angle);
  rot_angle(1, 1) = cos(angle);

  // compute final orientation
  rot = rot * rot_angle;

  std::cout << rot << std::endl;

  Eigen::Quaterniond q(rot);
  target_pose_homography_.orientation.x = q.x();
  target_pose_homography_.orientation.y = q.y();
  target_pose_homography_.orientation.z = q.z();
  target_pose_homography_.orientation.w = q.w();
}

void MotionPlanning::homographyPoseCallback(const pir_msgs::HomographyPose::ConstPtr& msg)
{
  calculate_homography_based_pose(msg->x, msg->y, msg->angle, msg->object);

  homography_pose_received_ = true;
}

void MotionPlanning::pose3DCallback(const pir_msgs::Pose3D::ConstPtr& msg)
{
}

}  // namespace planner
