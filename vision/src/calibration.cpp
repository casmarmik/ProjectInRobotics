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
    temp.push_back(coords[i].at<float>(0)/1000.0);
    temp.push_back(coords[i].at<float>(1)/1000.0);
    temp.push_back(coords[i].at<float>(2)/1000.0);
    t_base2gripper.push_back(temp);
    cv::Mat euler;
    euler.push_back(coords[i].at<float>(3));
    euler.push_back(coords[i].at<float>(4));
    euler.push_back(coords[i].at<float>(5));
    cv::Rodrigues(euler,euler);
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

    // std::cout << i << std::endl;
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

  
  // std::vector<std::vector<cv::Point3f>> Q_good;
  // std::vector<cv::Mat> corners_good;
  // std::vector<cv::Mat> R_base2gripper_good, t_base2gripper_good;
  // for (size_t i = 0; i < nPoses; i++)
  // {
  //   if (i == 0 || i == 1 || i == 8 || i == 9)
  //   {
  //     Q_good.push_back(Q[i]);
  //     corners_good.push_back(corners[i]);
  //     R_base2gripper_good.push_back(R_base2gripper[i]);
  //     t_base2gripper_good.push_back(t_base2gripper[i]);
  //   }
  // }
  

  float error = cv::calibrateCamera(Q, corners, frameSize, K, k, rvecs, tvecs, flags);

  std::cout << "Reprojection error = " << error << "\nK =\n" << K << "\nk=\n" << k << std::endl;

  std::vector<cv::Mat> R_t2c,t_t2c;

  for (int i = 0; i < Q.size(); i++)
  {
    cv::Mat R_temp, t_temp;
    cv::solvePnPRansac(Q[i], corners[i], K, k, R_temp, t_temp);
    cv::Rodrigues(R_temp,R_temp);
    R_t2c.push_back(R_temp);
    t_t2c.push_back(t_temp);
  }
  
  // cv::imshow("chessboard detection", imgs[4]);
  // cv::waitKey(0);

  cv::Mat R_cam2base_est = cv::Mat::eye(3,3,CV_32F);
  R_cam2base_est.at<float>(1,1) = -1;
  R_cam2base_est.at<float>(2,2) = -1;
  cv::Mat t_cam2base_est;
  t_cam2base_est.push_back(-0.6);
  t_cam2base_est.push_back(0.6);
  t_cam2base_est.push_back(-0.8);

  std::cout << "R_base2gripper:\n" << R_base2gripper[0] << std::endl;
  std::cout << "t_base2gripper:\n" << t_base2gripper[0] << std::endl;
  std::cout << "R_t2c:\n" << R_t2c[0] << std::endl;
  std::cout << "t_t2c:\n" << t_t2c[0] << std::endl;

  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est, cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_DANIILIDIS);

  std::cout << "R_cam2base_est:\n" << R_cam2base_est << std::endl;
  std::cout << "R_cam2base_est.t:\n" << R_cam2base_est.t() << std::endl;
  std::cout << "t_cam2base_est:\n" << -R_cam2base_est.t() * t_cam2base_est << std::endl;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration");

  calibrate();

  return 0;
}