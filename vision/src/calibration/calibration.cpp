#include <ros/ros.h>
#include "opencv2/calib3d.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>

void opencv_calibration(std::vector<cv::Mat> R_base2gripper, std::vector<cv::Mat> t_base2gripper,
                        std::vector<cv::Mat> R_t2c, std::vector<cv::Mat> t_t2c)
{
  cv::Mat R_cam2base_est, t_cam2base_est;

  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_ANDREFF);
  std::cout << "CALIB_HAND_EYE_ANDREFF" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est.inv() << std::endl;
  std::cout << "t_cam2base_est:\n" << -R_cam2base_est.inv() * t_cam2base_est << std::endl;
  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_DANIILIDIS);
  std::cout << "CALIB_HAND_EYE_DANIILIDIS" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est.inv() << std::endl;
  std::cout << "t_cam2base_est:\n" << -R_cam2base_est.inv() * t_cam2base_est << std::endl;
  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_HORAUD);
  std::cout << "CALIB_HAND_EYE_HORAUD" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est.inv() << std::endl;
  std::cout << "t_cam2base_est:\n" << -R_cam2base_est.inv() * t_cam2base_est << std::endl;
  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_PARK);
  std::cout << "CALIB_HAND_EYE_PARK" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est.inv() << std::endl;
  std::cout << "t_cam2base_est:\n" << -R_cam2base_est.inv() * t_cam2base_est << std::endl;
  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_TSAI);
  std::cout << "CALIB_HAND_EYE_TSAI" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est.inv() << std::endl;
  std::cout << "t_cam2base_est:\n" << -R_cam2base_est.inv() * t_cam2base_est << std::endl;
}

void calibrate(bool show_images = false)
{
  std::vector<cv::Mat> imgs;
  std::vector<std::vector<double>> coords;
  unsigned int number_of_images = 20;
  for (unsigned int i = 0; i < number_of_images; i++)
  {
    std::stringstream ss;
    ss << "/home/mads/project_in_robotics/project_in_robotics/vision/data/calibration/" << i << ".jpeg";
    std::string image_path = cv::samples::findFile(ss.str().c_str());
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    imgs.push_back(img);
  }

  std::string line;
  std::ifstream myfile("/home/mads/project_in_robotics/project_in_robotics/vision/data/calibration/tcp.txt");
  if (myfile.is_open())
  {
    while (getline(myfile, line))
    {
      std::stringstream ss(line);
      std::string temp;
      std::vector<double> coord;
      while (getline(ss, temp, ','))
      {
        coord.push_back(std::stof(temp));
      }
      coords.push_back(coord);
    }
    myfile.close();
  }

  int nPoses = number_of_images;

  std::vector<cv::Mat> R_base2gripper, t_base2gripper;

  float squareSize = 0.0204;
  cv::Size boardSize(10, 7);
  std::vector<cv::Mat> corners;
  std::vector<std::vector<cv::Point3f>> Q;
  std::vector<cv::Point3f> objp;
  for (int i = 0; i < boardSize.height; i++)
  {
    for (int j = 0; j < boardSize.width; j++)
    {
      objp.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
    }
  }
  std::vector<unsigned int> img_index;
  for (size_t i = 0; i < nPoses; i++)
  {
    cv::Mat gray, temp_corners;
    cv::cvtColor(imgs[i], gray, cv::COLOR_RGB2GRAY);
    bool found = cv::findChessboardCorners(gray, boardSize, temp_corners, cv::CALIB_CB_ADAPTIVE_THRESH);
    if (found)
    {
      cv::cornerSubPix(gray, temp_corners, cv::Size(11, 11), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
      Q.push_back(objp);
      corners.push_back(temp_corners);
      cv::Mat temp;
      std::cout << coords[i][0] << " " << coords[i][1] << " " << coords[i][2] << std::endl;

      temp.push_back(coords[i][0] / 1000.0);
      temp.push_back(coords[i][1] / 1000.0);
      temp.push_back(coords[i][2] / 1000.0);

      t_base2gripper.push_back(temp);
      cv::Mat euler;
      euler.push_back(coords[i][3]);
      euler.push_back(coords[i][4]);
      euler.push_back(coords[i][5]);
      cv::Rodrigues(euler, euler);

      R_base2gripper.push_back(euler);
      img_index.push_back(i);

      // drawChessboardCorners(imgs[i], boardSize, cv::Mat(temp_corners), found);
    }
  }

  cv::Matx33f K(cv::Matx33f::eye());   // intrinsic camera matrix
  cv::Vec<float, 5> k(0, 0, 0, 0, 0);  // distortion coefficients

  std::vector<cv::Mat> rvecs, tvecs;
  int flags =
      cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
  cv::Size frameSize(640, 480);

  float error = cv::calibrateCamera(Q, corners, frameSize, K, k, rvecs, tvecs, flags);

  std::cout << "Reprojection error = " << error << "\nK =\n" << K << "\nk=\n" << k << std::endl;

  std::ofstream pose_file;
  pose_file.open("/home/mads/project_in_robotics/project_in_robotics/vision/data/calibration/camera_poses.txt");

  std::vector<cv::Mat> R_t2c, t_t2c;

  for (int i = 0; i < Q.size(); i++)
  {
    // Compute transformation between object and camera
    cv::Mat R_temp, t_temp;
    cv::solvePnP(cv::Mat(Q[i]), cv::Mat(corners[i]), K, k, R_temp, t_temp);
    cv::Rodrigues(R_temp, R_temp);

    R_t2c.push_back(R_temp);
    t_t2c.push_back(t_temp);

    // Calculate transformation from gripper to object marker
    cv::Matx44f trans(cv::Matx44f::eye());
    trans(0, 3) = -0.083;
    trans(1, 3) = 0.062;
    trans(2, 3) = 0.075;
    // Rotation from tcp to object coordinate system
    trans(0, 0) = 1;
    trans(1, 1) = -1;
    trans(2, 2) = -1;

    cv::Matx44f b2g(cv::Matx44f::eye());
    b2g(0, 0) = R_base2gripper[i].at<double>(0, 0);
    b2g(0, 1) = R_base2gripper[i].at<double>(0, 1);
    b2g(0, 2) = R_base2gripper[i].at<double>(0, 2);
    b2g(1, 0) = R_base2gripper[i].at<double>(1, 0);
    b2g(1, 1) = R_base2gripper[i].at<double>(1, 1);
    b2g(1, 2) = R_base2gripper[i].at<double>(1, 2);
    b2g(2, 0) = R_base2gripper[i].at<double>(2, 0);
    b2g(2, 1) = R_base2gripper[i].at<double>(2, 1);
    b2g(2, 2) = R_base2gripper[i].at<double>(2, 2);
    b2g(0, 3) = t_base2gripper[i].at<double>(0);
    b2g(1, 3) = t_base2gripper[i].at<double>(1);
    b2g(2, 3) = t_base2gripper[i].at<double>(2);

    b2g = b2g * trans;

    R_base2gripper[i].at<double>(0, 0) = b2g(0, 0);
    R_base2gripper[i].at<double>(0, 1) = b2g(0, 1);
    R_base2gripper[i].at<double>(0, 2) = b2g(0, 2);
    R_base2gripper[i].at<double>(1, 0) = b2g(1, 0);
    R_base2gripper[i].at<double>(1, 1) = b2g(1, 1);
    R_base2gripper[i].at<double>(1, 2) = b2g(1, 2);
    R_base2gripper[i].at<double>(2, 0) = b2g(2, 0);
    R_base2gripper[i].at<double>(2, 1) = b2g(2, 1);
    R_base2gripper[i].at<double>(2, 2) = b2g(2, 2);
    t_base2gripper[i].at<double>(0) = b2g(0, 3);
    t_base2gripper[i].at<double>(1) = b2g(1, 3);
    t_base2gripper[i].at<double>(2) = b2g(2, 3);

    // Align object coordinate system with tcp coordinate system
    cv::Matx44f t2c(cv::Matx44f::eye());
    t2c(0, 0) = R_t2c[i].at<double>(0, 0);
    t2c(0, 1) = R_t2c[i].at<double>(0, 1);
    t2c(0, 2) = R_t2c[i].at<double>(0, 2);
    t2c(1, 0) = R_t2c[i].at<double>(1, 0);
    t2c(1, 1) = R_t2c[i].at<double>(1, 1);
    t2c(1, 2) = R_t2c[i].at<double>(1, 2);
    t2c(2, 0) = R_t2c[i].at<double>(2, 0);
    t2c(2, 1) = R_t2c[i].at<double>(2, 1);
    t2c(2, 2) = R_t2c[i].at<double>(2, 2);
    t2c(0, 3) = t_t2c[i].at<double>(0);
    t2c(1, 3) = t_t2c[i].at<double>(1);
    t2c(2, 3) = t_t2c[i].at<double>(2);

    // Inverte object transformation so that we get from object to camera
    t2c = t2c.inv();

    cv::Matx44f camera_pose(cv::Matx44f::eye());

    camera_pose = b2g * t2c;

    std::cout << "res\n" << camera_pose << std::endl;

    // Write the estimated camera pose to a file
    for (unsigned int i = 0; i < camera_pose.rows; ++i)
    {
      for (unsigned int j = 0; j < camera_pose.cols; ++j)
      {
        pose_file << camera_pose(i, j) << ",";
      }
      pose_file << "\n";
    }

    if (show_images)
    {
      // Show images with 3D points projected on to them
      std::vector<cv::Point3f> point_3d;
      point_3d.push_back(cv::Point3f(0.1, 0, 0));
      point_3d.push_back(cv::Point3f(0, 0.1, 0));
      point_3d.push_back(cv::Point3f(0, 0, -0.1));
      point_3d.push_back(cv::Point3f(0, 0, 0));
      std::vector<cv::Point2f> point_2d;
      cv::projectPoints(point_3d, R_temp, t_temp, K, k, point_2d);
      cv::line(imgs[img_index[i]], point_2d[3], point_2d[0], cv::Scalar(0, 0, 255), 5);
      cv::line(imgs[img_index[i]], point_2d[3], point_2d[1], cv::Scalar(0, 255, 0), 5);
      cv::line(imgs[img_index[i]], point_2d[3], point_2d[2], cv::Scalar(255, 0, 0), 5);
      cv::imshow("chessboard detection", imgs[img_index[i]]);
      if (img_index[i] == 5)
      {
        cv::imwrite("chessboard_corners.jpg", imgs[img_index[i]]);
      }
      cv::waitKey(0);
    }
  }
  // opencv_calibration(R_base2gripper, t_base2gripper, R_t2c, t_t2c);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration");

  calibrate(false);

  return 0;
}
