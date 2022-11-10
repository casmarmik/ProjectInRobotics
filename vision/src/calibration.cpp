#include <ros/ros.h>
#include "opencv2/calib3d.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>

void calibrate()
{
  std::vector<cv::Mat> imgs;
  std::vector<cv::Mat> coords;
  for (unsigned int i = 0; i < 10; i++)
  {
    std::stringstream ss;
    ss << "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/calibration/" << i << ".jpeg";
    std::string image_path = cv::samples::findFile(ss.str().c_str());
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    imgs.push_back(img);
  }

  std::string line;
  std::ifstream myfile("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/calibration/tcp_pos.txt");
  if (myfile.is_open())
  {
    while (getline(myfile,line))
    {
      std::stringstream ss(line);
      std::string temp;
      cv::Mat coord;
      while (getline(ss,temp,','))
      {
        coord.push_back(std::stof(temp));
      }
      coords.push_back(coord);
    }
    myfile.close();
  }

  int nPoses = coords.size();

  std::vector<cv::Mat> R_base2gripper, t_base2gripper;

  for (unsigned int i = 0; i < nPoses; i++)
  {
    cv::Mat temp;
    temp.push_back(coords[i].at<float>(0));
    temp.push_back(coords[i].at<float>(1));
    temp.push_back(coords[i].at<float>(2));
    t_base2gripper.push_back(temp);
    cv::Mat euler;
    euler.push_back(coords[i].at<float>(3));
    euler.push_back(coords[i].at<float>(4));
    euler.push_back(coords[i].at<float>(5));
    R_base2gripper.push_back(euler);
  }

  float squareSize = 0.0204;
  cv::Size boardSize(10,7);
  std::vector<cv::Mat> corners(nPoses);
  std::vector<std::vector<cv::Point3f>> Q;
  std::vector<cv::Point3f> objp;
  for(int i = 0; i<boardSize.height; i++)
  {
    for(int j = 0; j<boardSize.width; j++)
    {
      objp.push_back(cv::Point3f(j*squareSize,i*squareSize,0));
    }
  }

  for (size_t i = 0; i < nPoses; i++)
  {
    cv::Mat gray;
    cv::cvtColor(imgs[i], gray, cv::COLOR_RGB2GRAY);
    bool found = cv::findChessboardCorners(gray, boardSize, corners[i], cv::CALIB_CB_ADAPTIVE_THRESH);
    if(found)
    {
      cv::cornerSubPix(gray, corners[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
      Q.push_back(objp);
    }

    // drawChessboardCorners(imgs[i], boardSize, cv::Mat(corners[i]), found);
    // cv::imshow("chessboard detection", imgs[i]);
    // cv::waitKey(0);
  }

  cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
  cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

  std::vector<cv::Mat> rvecs, tvecs;
  int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
              cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
  cv::Size frameSize(640, 480);

  float error = cv::calibrateCamera(Q, corners, frameSize, K, k, rvecs, tvecs, flags);

  // std::cout << "Reprojection error = " << error << "\nK =\n" << K << "\nk=\n" << k << std::endl;

  cv::Mat rvec,tvec;
  cv::solvePnPRansac(Q[4], corners[4], K, k, rvec, tvec);

  cv::Mat r_vec = rvec;
  cv::Mat t_vec = tvec;

  std::cout << "rvec:\n" << rvec << std::endl;
  std::cout << "tvec:\n" << tvec << std::endl;

  // cv::imshow("chessboard detection", imgs[4]);
  // cv::waitKey(0);

  cv::Mat R_cam2base_est, t_cam2base_est;


  cv::Mat R_b2g = R_base2gripper[4];
  cv::Mat t_b2g = t_base2gripper[4];

  std::cout << "R_base2gripper:\n" << R_b2g << std::endl;
  std::cout << "t_base2gripper:\n" << t_b2g << std::endl;

  calibrateHandEye(R_b2g, t_b2g, r_vec, t_vec, R_cam2base_est, t_cam2base_est, cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_ANDREFF);

  std::cout << "R_cam2base_est:\n" << R_cam2base_est << std::endl;
  std::cout << "t_cam2base_est:\n" << t_cam2base_est << std::endl;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration");

  calibrate();

  return 0;
}