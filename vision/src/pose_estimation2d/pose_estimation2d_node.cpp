#include "pose_estimation2d/pose_estimation2d.h"
// Opencv stuff
#include <opencv2/highgui.hpp>

// Ros stuff
#include <ros/ros.h>
#include <vision/PoseEstimation.h>
#include <vision/Pose.h>

using namespace pose_estimation_2d;

class PoseEstimation2DNode
{
public:
  PoseEstimation2DNode(const ros::NodeHandle& nh)
  {
    nh_ = nh;
    image_path_sub_ = nh_.subscribe<vision::PoseEstimation>("/network_node/image_pose2d", 1000,
                                                            &PoseEstimation2DNode::poseEstimationCallback, this);

    pose_estimate_pub_ = nh_.advertise<vision::Pose>("/vision/pose", 1000);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber image_path_sub_;
  ros::Publisher pose_estimate_pub_;
  PoseEstimation2D pose2d_;

  void poseEstimationCallback(const vision::PoseEstimation::ConstPtr& msg)
  {
    std::string image_path = msg->filepath;
    uint32_t object = msg->object;
    cv::Mat image = cv::imread(image_path);

    cv::Point2f object_center;
    double angle;
    pose2d_.computePoseEstimation(image, object_center, angle);

    // Publish pose estimate for motion planner
    vision::Pose pose;
    pose.x = object_center.x;
    pose.y = object_center.y;
    pose.angle = angle;
    pose_estimate_pub_.publish(pose);
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
