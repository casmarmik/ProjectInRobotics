#pragma once

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

namespace pose_estimation_2d
{
class PoseEstimation2D
{
public:
  PoseEstimation2D();

  PoseEstimation2D(std::string homography_path);

  // Compute the pose of an object based on the input image
  void computePoseEstimation(cv::Mat image, cv::Point2f& object_center, double& object_orientation, bool debug = false);

private:
  cv::Mat image_, cropped_image_;
  cv::Mat homography_;

  void findHomography(std::string homography_path);
  void loadHomography(std::string homography_path);

  void cropImage();

  cv::Mat computeBinaryImage(bool show_image = false);

  std::vector<cv::Point> getContour(cv::Mat binary_image, bool show_image = false);

  // Get the angle at which the object is rotated
  double getAngle(std::vector<cv::Point> contour, bool debug = false);
  cv::Point2f getObjectCenter(std::vector<cv::Point> contour, bool debug = false);

private:
};

}  // namespace pose_estimation_2d