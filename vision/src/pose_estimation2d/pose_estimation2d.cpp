#include "pose_estimation2d/pose_estimation2d.h"
#include <opencv2/highgui.hpp>
#include "opencv2/features2d.hpp"

#include <opencv2/calib3d.hpp>
#include <iostream>

namespace pose_estimation_2d
{
PoseEstimation2D::PoseEstimation2D()
{
  std::string homography_path = "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/homography/homography_matrix.xml";
  findHomography(homography_path);
}

PoseEstimation2D::PoseEstimation2D(std::string homography_path)
{
  loadHomography(homography_path);
}

void PoseEstimation2D::computePoseEstimation(cv::Mat& image, int object, cv::Point2f& object_center,
                                             double& object_orientation, bool debug)
{
  image_ = image;

  // Crop the input image into grasping area
  cropImage();

  // Compute the binary image
  cv::Mat binary_image = computeBinaryImage(object, debug);

  // Find the contour of the object
  std::vector<cv::Point> contour = getContour(binary_image, debug);

  // Calculate the center of the object in mm
  object_center = getObjectCenter(contour, debug);

  // Calculate the angle of the object
  object_orientation = getAngle(contour, debug);

  // ORB based orientation computation (Not used)
  // cv::Rect rect = computeBoundedRectangle(contour);
  // cv::Mat rect_image = cropImageBasedOnRectangle(rect);
  // second_orientation = computeAngle(object_img, rect_image);

  // Doesn't produced better results unfortantely
  // std::vector<cv::Point2f> image_point;
  // std::vector<cv::Point2f> table_point;
  // cv::Point2f center_of_rect = (rect.br() + rect.tl()) * 0.5;
  // image_point.push_back(center_of_rect);

  // cv::perspectiveTransform(image_point, table_point, homography_);
  // object_center = table_point[0];
}

double PoseEstimation2D::computeAngle(cv::Mat object, cv::Mat rect_img)
{
  cv::Mat descriptor_1, descriptor_2;
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;

  cv::Mat object_gray, rect_img_gray;
  cv::cvtColor(object, object_gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(rect_img, rect_img_gray, cv::COLOR_BGR2GRAY);

  cv::Ptr<cv::ORB> detector = cv::ORB::create();

  detector->detectAndCompute(object_gray, cv::noArray(), keypoints_1, descriptor_1);
  detector->detectAndCompute(rect_img_gray, cv::noArray(), keypoints_2, descriptor_2);

  std::vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_L2);  // Euclidian distance

  matcher.match(descriptor_1, descriptor_2, matches);

  // Compute the coordinates for the keypoints
  std::vector<cv::Point2f> object_points, rect_img_points;
  for (unsigned int i = 0; i < matches.size(); ++i)
  {
    cv::Point2f object_point = keypoints_1[matches[i].queryIdx].pt;
    cv::Point2f rect_img_point = keypoints_2[matches[i].trainIdx].pt;
    object_points.push_back(object_point);
    rect_img_points.push_back(rect_img_point);
  }

  cv::Mat affineMapping = cv::estimateAffine2D(object_points, rect_img_points);
  // Compute angle
  double ss = -affineMapping.at<double>(0, 1);
  double sc = affineMapping.at<double>(0, 0);
  double angle = atan2(ss, sc);
  return angle;
}

cv::Mat PoseEstimation2D::cropImageBasedOnRectangle(cv::Rect rect)
{
  cv::Mat masked_image = cv::Mat::zeros(image_.size(), image_.type());

  for (unsigned int cols = rect.tl().x; cols < rect.br().x; ++cols)
  {
    for (unsigned int rows = rect.tl().y; rows < rect.br().y; ++rows)
    {
      masked_image.at<cv::Vec3b>(rows, cols)[0] = image_.at<cv::Vec3b>(rows, cols)[0];
      masked_image.at<cv::Vec3b>(rows, cols)[1] = image_.at<cv::Vec3b>(rows, cols)[1];
      masked_image.at<cv::Vec3b>(rows, cols)[2] = image_.at<cv::Vec3b>(rows, cols)[2];
    }
  }

  return masked_image;
}

void PoseEstimation2D::cropImage()
{
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
  ss << "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/homography/homography.jpeg";
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

cv::Mat PoseEstimation2D::computeBinaryImage(int object, bool show_image)
{
  cv::Scalar lower;
  cv::Scalar upper;
  if (object == 0)  // Screw
  {
    lower = cv::Scalar(34, 0, 0);
    upper = cv::Scalar(179, 255, 160);
  }
  else  // plug
  {
    lower = cv::Scalar(34, 0, 0);
    upper = cv::Scalar(179, 255, 180);
  }

  cv::Mat mask, hsv_image, masked_image;
  cv::cvtColor(cropped_image_, hsv_image, cv::COLOR_BGR2HSV);
  cv::inRange(hsv_image, lower, upper, mask);
  cv::bitwise_and(cropped_image_, cropped_image_, masked_image, mask);

  cv::Mat gray_image, binary_image;
  cv::cvtColor(masked_image, gray_image, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
  cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
  cv::threshold(gray_image, binary_image, 60, 100, cv::THRESH_BINARY);

  if (show_image)
  {
    cv::imshow("binary image", binary_image);
    cv::waitKey();
    cv::destroyAllWindows();
  }

  cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat erode_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

  if (object == 0)  // Screw
  {
    // Dilate the image
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);

    // Erode the image
    cv::erode(binary_image, binary_image, erode_element);
    cv::erode(binary_image, binary_image, erode_element);
  }
  else  // plug
  {
    // Dilate the image
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);
    cv::dilate(binary_image, binary_image, dilate_element);

    // Erode the image
    cv::erode(binary_image, binary_image, erode_element);
    cv::erode(binary_image, binary_image, erode_element);
    // cv::erode(binary_image, binary_image, erode_element);
    // cv::erode(binary_image, binary_image, erode_element);
  }

  if (show_image)
  {
    cv::imshow("binary image", binary_image);
    cv::waitKey();
    cv::destroyAllWindows();
  }
  return binary_image;
}

cv::Rect PoseEstimation2D::computeBoundedRectangle(std::vector<cv::Point> contour)
{
  cv::Rect rect = cv::boundingRect(contour);
  return rect;
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
    // cv::Scalar color = cv::Scalar(0, 0, 255);
    // cv::drawContours(cropped_image_, approx, -1, color, 3);
    // Loop through all contours to draw them
    // for (size_t i = 0; i < contours.size(); i++)
    // {
    //   cv::Scalar color = cv::Scalar(0, 0, 255);
    //   cv::drawContours(cropped_image_, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
    // }
    cv::imshow("Contours", cropped_image_);
    cv::waitKey();
    cv::destroyAllWindows();
  }

  return contour;
}

// Inspired from here https://docs.opencv.org/4.x/d1/dee/tutorial_introduction_to_pca.html
void PoseEstimation2D::drawAxis(cv::Point p, cv::Point q, cv::Scalar colour, const float scale)
{
  double angle = atan2((double)p.y - q.y, (double)p.x - q.x);  // angle in radians
  double hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
  // Here we lengthen the arrow by a factor of scale
  q.x = (int)(p.x - scale * hypotenuse * cos(angle));
  q.y = (int)(p.y - scale * hypotenuse * sin(angle));
  line(image_, p, q, colour, 1, cv::LINE_AA);
  // create the arrow hooks
  p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
  line(image_, p, q, colour, 1, cv::LINE_AA);
  p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
  line(image_, p, q, colour, 1, cv::LINE_AA);
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
  std::vector<double> eigen_val(2);
  for (int i = 0; i < 2; i++)
  {
    eigen_vecs[i] = cv::Point2d(pca.eigenvectors.at<double>(i, 0), pca.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca.eigenvalues.at<double>(i);
  }

  // Store the center of the object
  cv::Point cntr = cv::Point(static_cast<int>(pca.mean.at<double>(0, 0)), static_cast<int>(pca.mean.at<double>(0, 1)));

  // Use the eigen vector with the most spread to compute the angle
  double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);
  angle = angle * (180 / M_PI);
  if (debug)
  {
    // Draw the principal components
    cv::Point p1 = cntr + 0.02 * cv::Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),
                                           static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    cv::Point p2 = cntr - 0.02 * cv::Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]),
                                           static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(cntr, p1, cv::Scalar(0, 0, 255), 10);
    drawAxis(cntr, p2, cv::Scalar(0, 255, 0), 10);
    // line(cropped_image_, cv::Point(0, cropped_image_.rows), cv::Point(200, cropped_image_.rows), cv::Scalar(0, 0,
    // 255),
    //      10);
    // line(cropped_image_, cv::Point(0, cropped_image_.rows), cv::Point(0, cropped_image_.rows - 200),
    //      cv::Scalar(0, 255, 0), 10);
    cv::imshow("Image", image_);
    std::cout << "orientation of binary object: " << -angle << std::endl;
    cv::waitKey();
  }
  return angle;
}

cv::Point2f PoseEstimation2D::getObjectCenter(std::vector<cv::Point> contour, bool debug)
{
  cv::Moments moments = cv::moments(contour);
  cv::Point center_of_mass;
  center_of_mass.x = (moments.m10 / moments.m00);
  center_of_mass.y = (moments.m01 / moments.m00);

  std::vector<cv::Point2f> image_point;
  std::vector<cv::Point2f> table_point;
  image_point.push_back(center_of_mass);

  cv::perspectiveTransform(image_point, table_point, homography_);
  cv::circle(image_, center_of_mass, 5, cv::Scalar(0, 255, 0));
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