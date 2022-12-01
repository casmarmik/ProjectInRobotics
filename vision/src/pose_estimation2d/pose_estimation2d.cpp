#include "pose_estimation2d/pose_estimation2d.h"
#include <opencv2/highgui.hpp>

#include <opencv2/calib3d.hpp>
#include <iostream>

namespace pose_estimation_2d
{
PoseEstimation2D::PoseEstimation2D()
{
  std::string homography_path = "/home/mads/project_in_robotics/project_in_robotics/vision/data/homography/"
                                "homography_matrix.xml";
  findHomography(homography_path);
}

PoseEstimation2D::PoseEstimation2D(std::string homography_path)
{
  loadHomography(homography_path);
}

void PoseEstimation2D::computePoseEstimation(cv::Mat image, cv::Point2f& object_center, double& object_orientation,
                                             bool debug)
{
  // TODO map to the plane before finding the contours or maybe do some other stuff to handle the offset that is comming
  // when mapping to the plane
  image_ = image;

  // Crop the input image into grasping area
  cropImage();

  // Compute the binary image
  cv::Mat binary_image = computeBinaryImage(debug);

  // Find the contour of the object
  std::vector<cv::Point> contour = getContour(binary_image, debug);

  // Calculate the center of the object in mm
  object_center = getObjectCenter(contour, debug);

  // Calculate the angle of the object
  object_orientation = getAngle(contour, true);
}

void PoseEstimation2D::cropImage()
{
  // TODO does it make sense to warp the image first?
  cv::Mat masked_image = cv::Mat::zeros(image_.size(), image_.type());
  for (unsigned int cols = 207; cols < 458; ++cols)
  {
    for (unsigned int rows = 77; rows < 300; ++rows)
    {
      masked_image.at<cv::Vec3b>(rows, cols)[0] = image_.at<cv::Vec3b>(rows, cols)[0];
      masked_image.at<cv::Vec3b>(rows, cols)[1] = image_.at<cv::Vec3b>(rows, cols)[1];
      masked_image.at<cv::Vec3b>(rows, cols)[2] = image_.at<cv::Vec3b>(rows, cols)[2];
    }
  }
  cropped_image_ = masked_image;
}

void PoseEstimation2D::findHomography(std::string homography_path)
{
  std::stringstream ss;
  ss << "/home/mads/project_in_robotics/project_in_robotics/vision/data/homography/homography.jpeg";
  std::string image_path = cv::samples::findFile(ss.str().c_str());
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);

  float squareSize = 20.4;
  cv::Size boardSize(10, 7);

  std::vector<cv::Point2f> corners;
  std::vector<cv::Point2f> objp;
  for (int i = 0; i < boardSize.height; i++)
  {
    for (int j = 0; j < boardSize.width; j++)
    {
      objp.push_back(cv::Point2f(j * squareSize, i * squareSize));
    }
  }

  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
  bool found = cv::findChessboardCorners(gray, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH);
  if (found)
  {
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    drawChessboardCorners(img, boardSize, corners, found);
  }

  // x 285.1 mm and y -272.7 mm, which is 0.0 in the homography plane
  homography_ = cv::findHomography(corners, objp);

  cv::FileStorage file(homography_path, cv::FileStorage::WRITE);
  file << "homography_matrix" << homography_;
  file.release();
}

void PoseEstimation2D::loadHomography(std::string homography_path)
{
  cv::FileStorage file(homography_path, cv::FileStorage::READ);
  file["homography_matrix"] >> homography_;
  file.release();
}

cv::Mat PoseEstimation2D::computeBinaryImage(bool show_image)
{
  cv::Scalar lower(34, 0, 0);
  cv::Scalar upper(179, 255, 201);

  cv::Mat mask, hsv_image, masked_image;
  cv::cvtColor(cropped_image_, hsv_image, cv::COLOR_BGR2HSV);
  cv::inRange(hsv_image, lower, upper, mask);
  cv::bitwise_and(cropped_image_, cropped_image_, masked_image, mask);

  cv::Mat gray_image, binary_image;
  cv::cvtColor(masked_image, gray_image, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
  cv::threshold(gray_image, binary_image, 10, 255, cv::THRESH_BINARY);

  cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat erode_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

  // Dilate the image
  cv::dilate(binary_image, binary_image, dilate_element);
  cv::dilate(binary_image, binary_image, dilate_element);
  cv::dilate(binary_image, binary_image, dilate_element);
  cv::dilate(binary_image, binary_image, dilate_element);

  // Erode the image
  cv::erode(binary_image, binary_image, erode_element);
  cv::erode(binary_image, binary_image, erode_element);

  if (show_image)
  {
    cv::imshow("binary image", binary_image);
    cv::waitKey();
    cv::destroyAllWindows();
  }
  return binary_image;
}

std::vector<cv::Point> PoseEstimation2D::getContour(cv::Mat binary_image, bool show_image)
{
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  cv::Point2f center_of_mass;
  std::vector<cv::Point> contour;
  if (contours.size() == 1)
  {
    contour = contours[0];
  }
  else if (contours.size() > 1)
  {
    // We take the largest contour as our input
    double max_area = 0;
    int idx = -1;
    for (unsigned int i = 0; i < contours.size(); ++i)
    {
      double area = cv::contourArea(contours[i]);
      if (area > max_area)
      {
        max_area = area;
        idx = i;
      }
    }
    contour = contours[idx];
  }
  else
  {
    std::cerr << "Failed to find any objects, have a look at the thresh holding values" << std::endl;
    return {};
  }

  if (show_image)
  {
    // Loop through all contours to draw them
    for (size_t i = 0; i < contours.size(); i++)
    {
      cv::Scalar color = cv::Scalar(0, 0, 255);
      cv::drawContours(cropped_image_, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
    }
    cv::imshow("Contours", cropped_image_);
    cv::waitKey();
    cv::destroyAllWindows();
  }
  return contour;
}

// Inspired from here https://docs.opencv.org/4.x/d1/dee/tutorial_introduction_to_pca.html
double PoseEstimation2D::getAngle(std::vector<cv::Point> contour, bool debug)
{
  // Construct a buffer used by the pca analysis
  unsigned int sz = static_cast<int>(contour.size());
  cv::Mat data_pts = cv::Mat(sz, 2, CV_64F);
  for (unsigned int i = 0; i < contour.size(); i++)
  {
    data_pts.at<double>(i, 0) = contour[i].x;
    data_pts.at<double>(i, 1) = contour[i].y;
  }

  // PCA analysis
  cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Get the eigen vectors
  std::vector<cv::Point2d> eigen_vecs(2);
  for (int i = 0; i < 2; i++)
  {
    eigen_vecs[i] = cv::Point2d(pca.eigenvectors.at<double>(i, 0), pca.eigenvectors.at<double>(i, 1));
  }

  // Use the eigen vector with the most spread to compute the angle
  double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);
  angle = angle * (180 / M_PI);
  if (debug)
  {
    std::cout << "orientation of binary object: " << angle << std::endl;
  }
  return angle;
}

cv::Point2f PoseEstimation2D::getObjectCenter(std::vector<cv::Point> contour, bool debug)
{
  cv::Moments moments = cv::moments(contour);
  cv::Point center_of_mass;
  center_of_mass.x = (moments.m10 / moments.m00);
  center_of_mass.y = (moments.m01 / moments.m00);
  std::cout << center_of_mass.x << " " << center_of_mass.y << std::endl;

  std::vector<cv::Point2f> image_point;
  std::vector<cv::Point2f> table_point;
  image_point.push_back(center_of_mass);

  cv::perspectiveTransform(image_point, table_point, homography_);

  if (debug)
  {
    cv::circle(cropped_image_, center_of_mass, 5, cv::Scalar(0, 255, 0));
    cv::imshow("Image", cropped_image_);
    cv::waitKey();
    std::cout << "X coordinate: " << table_point[0].x << " y coordinate: " << table_point[0].y << std::endl;
  }
  return table_point[0];
}

}  // namespace pose_estimation_2d