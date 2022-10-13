// Ros standard
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  // Start ROS
  ros::init(argc, argv, "motion_planning_kuka");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Load moveit planner
  moveit::planning_interface::MoveGroupInterface kuka_arm("kuka_arm");
  kuka_arm.setNumPlanningAttempts(10);

  const moveit::core::JointModelGroup* joint_model_group = kuka_arm.getCurrentState()->getJointModelGroup("kuka_arm");

  // Set cartesian movement
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1;
  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.2;
  target_pose1.position.z = 0.6;
  kuka_arm.setPoseTarget(target_pose1);

  // Plan the movement
  moveit::planning_interface::MoveGroupInterface::Plan the_plan;
  bool success = (kuka_arm.plan(the_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    // With this approach you can actually plan multiple movements and then execute them later one by one, and you also
    // dont have to plan from the current robot position
    kuka_arm.execute(the_plan);
  }
  else
  {
    std::cerr << "Failed to plan trajectory" << std::endl;
  }

  moveit::core::RobotStatePtr current_state = kuka_arm.getCurrentState();

  // Plane a movement in joint space
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = 0.1;  // -1/6 turn in radians
  kuka_arm.setJointValueTarget(joint_group_positions);

  // I think this can be used to set how fast the robot will move
  kuka_arm.setMaxVelocityScalingFactor(0.05);
  kuka_arm.setMaxAccelerationScalingFactor(0.05);

  success = (kuka_arm.plan(the_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    // With this approach you can actually plan multiple movements and then execute them later one by one, and you also
    // dont have to plan from the current robot position
    kuka_arm.execute(the_plan);
  }

  kuka_arm.setNamedTarget("home");
  kuka_arm.move();
  kuka_arm.stop();
}