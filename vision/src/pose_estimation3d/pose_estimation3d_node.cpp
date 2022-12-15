#include "pose_estimation3d/pose_estimation3d.h"
// Opencv stuff
#include <opencv2/highgui.hpp>

// Ros stuff
#include <ros/ros.h>
#include <std_srvs/Trigger.h>


class PoseEstimation3DNode
{
public:
  PoseEstimation3DNode(const ros::NodeHandle& nh)
  {
    nh_ = nh;
    service_ = nh_.advertiseService("/pose_estimate3d", &PoseEstimation3DNode::poseEstimationCallback, this);
  }

  bool poseEstimationCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    std::string image_path;
    bool visualize = true;

    for(int i = 0;i <= 2; ++i)
    {
      Eigen::Matrix4f pose_gt;
      image_path = "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/tests/depth/plug/" + std::to_string(i) + ".pcd";
      pose3d_.executePoseEstimation(visualize, image_path, "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/templates/plug.ply", pose_gt);
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

  ros::spin();

  return 0;
}
