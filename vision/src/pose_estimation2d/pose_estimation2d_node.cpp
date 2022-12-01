#include "pose_estimation2d/pose_estimation2d.h"
// Opencv stuff
#include <opencv2/highgui.hpp>

// Ros stuff
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace pose_estimation_2d;

class PoseEstimation2DNode
{
public:
  PoseEstimation2DNode(const ros::NodeHandle& nh)
  {
    nh_ = nh;
    image_path_sub_ =
        nh_.subscribe<std_msgs::String>("image_pose2d", 1000, &PoseEstimation2DNode::poseEstimationCallback, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber image_path_sub_;
  PoseEstimation2D pose2d_;

  void poseEstimationCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::string image_path = msg->data;
    cv::Mat image = cv::imread(image_path);

    cv::Point2f object_center;
    double angle;
    pose2d_.computePoseEstimation(image, object_center, angle);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimation2d");
  ros::NodeHandle nh;

  // TODO make publisher based on pose data
  PoseEstimation2DNode pose_estimation2d(nh);

  ros::spin();

  return 0;
}
