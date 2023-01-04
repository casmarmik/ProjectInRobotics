#include "pose_estimation2d/pose_estimation2d.h"

#include <iostream>
#include <fstream>
#include <chrono>

// Opencv stuff
#include <opencv2/highgui.hpp>

using namespace pose_estimation_2d;
using namespace std::chrono;

int main(int argc, char** argv)
{
  PoseEstimation2D pose2d_;

  std::vector<cv::Mat> images_plug;
  std::vector<cv::Mat> images_screw;
  cv::Mat object_plug = cv::imread("/home/mads/project_in_robotics/project_in_robotics/vision/data/pose_estimation2d/"
                                   "template_plug_rect.jpeg");
  cv::Mat object_screw = cv::imread("/home/mads/project_in_robotics/project_in_robotics/vision/data/pose_estimation2d/"
                                    "template_screw_rect.jpeg");

  std::string gt_file_name = "/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/poses.txt";
  std::string line;
  std::ifstream gt_file;
  gt_file.open(gt_file_name);
  std::vector<cv::Point2f> gt_points;
  while (std::getline(gt_file, line))
  {
    // Output the text from the file
    std::stringstream ss(line);
    std::string number;
    std::vector<double> points;
    while (getline(ss, number, ' '))
    {
      points.push_back(atof(number.c_str()));
    }
    cv::Point2f point(points[0], points[1]);
    gt_points.push_back(point);
  }

  std::ofstream out_file_plug, out_file_screw;
  out_file_plug.open("/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/plug_error.txt");
  out_file_screw.open("/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/screw_error.txt");
  cv::Point2f object_center;
  double pca_angle, orb_angle;
  std::vector<double> pca_angle_gt;
  std::vector<double> orb_angle_gt;
  for (unsigned int i = 0; i < 36; ++i)
  {
    if (i % 2 == 0)
    {
      pca_angle_gt.push_back(0);
    }
    else
    {
      pca_angle_gt.push_back(90);
    }
  }
  for (unsigned int i = 0; i < 36; ++i)
  {
    auto start = high_resolution_clock::now();
    std::stringstream ss_plug;
    ss_plug << "/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/plug/" << i << ".jpeg";
    cv::Mat image_plug = cv::imread(ss_plug.str());

    pose2d_.computePoseEstimation(image_plug, 1, object_center, pca_angle, false);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    std::cout << " angle orb plug " << orb_angle << std::endl;
    // std::cout << "gt coordinates " << gt_points[i].x << " " << gt_points[i].y << std::endl;
    // std::cout << "Cordinates plug " << object_center.x + 285.1 << " " << -1 * object_center.y - 272.7 << " "
    //           << std::endl;
    std::cout << "angle plug pca " << pca_angle << " angle orb " << orb_angle << std::endl;
    out_file_plug << std::abs((object_center.x + 285.1) - gt_points[i].x) << ","
                  << std::abs((-1 * object_center.y - 272.7) - gt_points[i].y) << ","
                  << std::abs(pca_angle - pca_angle_gt[i]) << "," << duration.count() << "\n";
    // cv::imshow("image screw", image_plug);

    start = high_resolution_clock::now();
    std::stringstream ss_screw;
    ss_screw << "/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/screw/" << i << ".jpeg";
    cv::Mat image_screw = cv::imread(ss_screw.str());

    pose2d_.computePoseEstimation(image_screw, 0, object_center, pca_angle, false);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    // std::cout << "Cordinates screw " << object_center.x + 285.1 << " " << -1 * object_center.y - 272.7 << " "
    //           << std::endl;
    std::cout << " angle orb screw " << orb_angle << std::endl;
    // cv::imshow("image screw", image_screw);
    // cv::waitKey();

    out_file_screw << std::abs((object_center.x + 285.1) - gt_points[i].x) << ","
                   << std::abs((-1 * object_center.y - 272.7) - gt_points[i].y) << ","
                   << std::abs(pca_angle - pca_angle_gt[i]) << "," << duration.count() << "\n";
  }
  // x 285.1 mm and y -272.7 mm, which is 0.0 in the homography plane (measured manually)

  return 0;
}