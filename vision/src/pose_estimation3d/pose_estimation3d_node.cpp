#include "pose_estimation3d/pose_estimation3d.h"
// Opencv stuff
#include <opencv2/highgui.hpp>

// Ros stuff
#include <ros/ros.h>
#include <std_msgs/String.h>

class PoseEstimation3DNode
{
public:
  PoseEstimation3DNode(const ros::NodeHandle& nh)
  {
    nh_ = nh;
    // image_path_sub_ = nh_.subscribe<std_msgs::String>("image_pose2d", 1000, &PoseEstimation2DNode::poseEstimationCallback, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber image_path_sub_;
  PoseEstimation3D pose3d_;

  void poseEstimationCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::string image_path = msg->data;
    cv::Mat image = cv::imread(image_path);

    bool visualize = false;
  
    pose3d_.executePoseEstimation(visualize, image_path, "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/templates/screw.ply");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimation2d");
  ros::NodeHandle nh;

  // TODO make publisher based on pose data
  PoseEstimation3DNode pose_estimation3d(nh);

  ros::spin();

  return 0;
}
