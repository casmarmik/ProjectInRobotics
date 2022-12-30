#include "pose_estimation2d/pose_estimation2d.h"
// Opencv stuff
#include <opencv2/highgui.hpp>

// Ros stuff
#include <ros/ros.h>
#include <pir_msgs/PoseEstimation.h>
#include <pir_msgs/HomographyPose.h>

using namespace pose_estimation_2d;

class PoseEstimation2DNode
{
public:
  PoseEstimation2DNode(const ros::NodeHandle& nh)
  {
    nh_ = nh;
    image_path_sub_ = nh_.subscribe<pir_msgs::PoseEstimation>("/network_node/image_pose2d", 1000,
                                                              &PoseEstimation2DNode::poseEstimationCallback, this);

    pose_estimate_pub_ = nh_.advertise<pir_msgs::HomographyPose>("/motion_planning/homography_pose", 1000);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber image_path_sub_;
  ros::Publisher pose_estimate_pub_;
  PoseEstimation2D pose2d_;

  void poseEstimationCallback(const pir_msgs::PoseEstimation::ConstPtr& msg)
  {
    std::string image_path = msg->filepath;
    uint32_t object = msg->object;
    cv::Mat image = cv::imread(image_path);

    cv::Point2f object_center;
    double angle;
    pose2d_.computePoseEstimation(image, object, object_center, angle);

    // Publish pose estimate for motion planner
    pir_msgs::HomographyPose pose;
    pose.x = (object_center.x + 285.1) / 1000;
    pose.y = (-object_center.y - 272.7) / 1000;
    pose.angle = angle;
    pose.object = object;
    pose_estimate_pub_.publish(pose);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimation2d");
  ros::NodeHandle nh;

  // TODO make publisher based on pose data
  PoseEstimation2DNode pose_estimation2d(nh);
  // PoseEstimation2D pose2d_;
  // cv::Mat image = cv::imread("/home/mads/project_in_robotics/project_in_robotics/vision/data/pose_estimation2d/"
  //                            "plug.jpeg");
  // cv::Point2f object_center;
  // double angle;
  // pose2d_.computePoseEstimation(image, 0, object_center, angle);

  ros::spin();

  return 0;
}
