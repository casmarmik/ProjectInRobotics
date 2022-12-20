#pragma once

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
  void computePoseEstimation(cv::Mat& image, cv::Mat object_img, int object, cv::Point2f& object_center,
                             double& object_orientation, double& orb_orientation, bool debug = false);

  cv::Mat computeBinaryImage(int object, bool show_image = false);

private:
  cv::Mat image_, cropped_image_;
  cv::Mat homography_;

  void findHomography(std::string homography_path);
  void loadHomography(std::string homography_path);

  void cropImage();

  std::vector<cv::Point> getContour(cv::Mat binary_image, bool show_image = false);

  cv::Rect computeBoundedRectangle(std::vector<cv::Point> contour);

  cv::Mat cropImageBasedOnRectangle(cv::Rect rect);

  double computeAngle(cv::Mat object, cv::Mat rect_img);

  // Get the angle at which the object is rotated
  double getAngle(std::vector<cv::Point> contour, bool debug = false);
  void drawAxis(cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2);
  cv::Point2f getObjectCenter(std::vector<cv::Point> contour, bool debug = false);
};

}  // namespace pose_estimation_2d
