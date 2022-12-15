#include "pose_estimation3d/pose_estimation3d.h"
// Opencv stuff
#include <opencv2/highgui.hpp>

// Ros stuff
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <fstream>
#include <iostream>


class PoseEstimation3DNode
{
public:
  PoseEstimation3DNode(const ros::NodeHandle& nh)
  {
    nh_ = nh;
    // service_ = nh_.advertiseService("/pose_estimate3d", &PoseEstimation3DNode::poseEstimationCallback, this);
  }

  // bool poseEstimationCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  bool poseEstimationCallback()
  {
    std::string image_path;
    bool visualize = true;

    Eigen::Matrix4f base2cam = Eigen::Matrix4f::Identity();
    base2cam(0,0) = 0.998620015673737 ;
    base2cam(0,1) = 0.0287368404911923;
    base2cam(0,2) = 0.0439574600536561;
    base2cam(0,3) = 0.38593275;
    base2cam(1,0) = -0.00150143419035511;
    base2cam(1,1) = -0.821045219705839;
    base2cam(1,2) = 0.570861185310023;
    base2cam(1,3) = -0.8836908;
    base2cam(2,0) = 0.0524958092723317;
    base2cam(2,1) = -0.570139405055269;
    base2cam(2,2) = -0.819869043696656;
    base2cam(2,3) = 0.5477055;

    std::fstream pose_file("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/tests/poses.txt", std::ios_base::in);

    float x,y,z;

    for(int i = 0;i <= 0; ++i)
    {
      Eigen::Matrix4f pose_gt_temp;
      Eigen::Matrix4f pose_gt = Eigen::Matrix4f::Identity();
      pose_file >> x;
      pose_file >> y;
      x /= 1000;
      y /= 1000;
      z = 0.04;
      Eigen::Affine3f transform(Eigen::Translation3f(x,y,z));
      Eigen::Matrix4f trans = transform.matrix();
      std::cout << "Number: " << i << std::endl;
      std::cout << trans << std::endl;
      pose_gt_temp = trans.inverse() * base2cam;
      pose_gt = pose_gt_temp.inverse();
      image_path = "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/tests/depth/screw/" + std::to_string(i) + ".pcd";
      pose3d_.executePoseEstimation(visualize, image_path, "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/templates/screw.ply", pose_gt);
    }
  
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber image_path_sub_;
  PoseEstimation3D pose3d_;
  ros::ServiceServer service_;

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_estimation2d");
  ros::NodeHandle nh;

  // TODO make publisher based on pose data
  PoseEstimation3DNode pose_estimation3d(nh);
  pose_estimation3d.poseEstimationCallback();

  // ros::spin();

  return 0;
}
