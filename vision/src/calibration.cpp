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
  std::vector<std::vector<double>> coords;
  unsigned int number_of_images = 10;
  for (unsigned int i = 0; i < number_of_images; i++)
  {
    std::stringstream ss;
    ss << "/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/calibration/" << i << ".jpeg";
    std::string image_path = cv::samples::findFile(ss.str().c_str());
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    imgs.push_back(img);
  }

  std::string line;
  std::ifstream myfile("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/calibration/tcp.txt");
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

  int nPoses = coords.size();

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
      // std::cout << coords[i][0] << " " << coords[i][1] << " " << coords[i][2] << std::endl;

      temp.push_back(coords[i][0] / 1000.0);
      temp.push_back(coords[i][1] / 1000.0);
      temp.push_back(coords[i][2] / 1000.0);
      //
      t_base2gripper.push_back(temp);
      cv::Mat euler;
      euler.push_back(coords[i][3]);
      euler.push_back(coords[i][4]);
      euler.push_back(coords[i][5]);
      cv::Rodrigues(euler, euler);

      R_base2gripper.push_back(euler);
      img_index.push_back(i);

      // std::cout << i << std::endl;
      drawChessboardCorners(imgs[i], boardSize, cv::Mat(temp_corners), found);
      // std::cout << temp << std::endl;
      // std::cout << euler << std::endl;
      // cv::imshow("chessboard detection", imgs[i]);
      // if (i > 0)
      //   cv::imshow("chessboard detection previous", imgs[i - 1]);
      // cv::waitKey(0);
    }
  }

  cv::Matx33f K(cv::Matx33f::eye());   // intrinsic camera matrix
  cv::Vec<float, 5> k(0, 0, 0, 0, 0);  // distortion coefficients

  std::vector<cv::Mat> rvecs, tvecs;
  int flags =
      cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
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
  // k[0] = 0.0;
  // k[1] = 0.0;
  // k[2] = 0.0;
  // k[3] = 0.0;
  // k[4] = 0.0;
  // K(0, 0) = 532.3397211426562;
  // K(0, 1) = 0.0;
  // K(0, 2) = 317.7268615646354;
  // K(1, 0) = 0.0;
  // K(1, 1) = 532.9557449170394;
  // K(1, 2) = 246.0449249401688;
  // K(2, 0) = 0.0;
  // K(2, 1) = 0.0;
  // K(2, 2) = 1.0;

  std::cout << "Reprojection error = " << error << "\nK =\n" << K << "\nk=\n" << k << std::endl;

  std::vector<cv::Mat> R_t2c, t_t2c;

  for (int i = 0; i < Q.size(); i++)
  {
    cv::Mat R_temp, t_temp;
    cv::solvePnPRansac(cv::Mat(Q[i]), cv::Mat(corners[i]), K, k, R_temp, t_temp);
    cv::Rodrigues(R_temp, R_temp);
    R_t2c.push_back(R_temp);
    t_t2c.push_back(t_temp);
    std::vector<cv::Point3f> point_3d;
    point_3d.push_back(cv::Point3f(0.1, 0, 0));
    point_3d.push_back(cv::Point3f(0, 0.1, 0));
    point_3d.push_back(cv::Point3f(0, 0, -0.1));
    point_3d.push_back(cv::Point3f(0, 0, 0));
    std::vector<cv::Point2f> point_2d;
    cv::projectPoints(point_3d, R_temp, t_temp, K, k, point_2d);

    cv::line(imgs[img_index[i]], point_2d[3], point_2d[0], (0, 0, 255), 5);
    cv::line(imgs[img_index[i]], point_2d[3], point_2d[1], (255, 255, 0), 5);
    // cv::line(imgs[img_index[i]], point_2d[3], point_2d[2], (255, 0, 0), 5);
    // std::cout << "tmp: " << -R_t2c t_temp << std::endl;
    // cv::imshow("chessboard detection", imgs[img_index[i]]);
    // cv::waitKey(0);

    cv::Matx44f trans(cv::Matx44f::eye()); 
    trans(0,3) = 0;
    trans(1,3) = 0;
    trans(2,3) = 0.075;// Add translation to z of tcp.

    cv::Matx44f b2g(cv::Matx44f::eye());
    b2g(0,0) = R_base2gripper[i].at<double>(0,0);
    b2g(0,1) = R_base2gripper[i].at<double>(0,1);
    b2g(0,2) = R_base2gripper[i].at<double>(0,2);
    b2g(1,0) = R_base2gripper[i].at<double>(1,0);
    b2g(1,1) = R_base2gripper[i].at<double>(1,1);
    b2g(1,2) = R_base2gripper[i].at<double>(1,2);
    b2g(2,0) = R_base2gripper[i].at<double>(2,0);
    b2g(2,1) = R_base2gripper[i].at<double>(2,1);
    b2g(2,2) = R_base2gripper[i].at<double>(2,2);
    b2g(0,3) = t_base2gripper[i].at<double>(0);
    b2g(1,3) = t_base2gripper[i].at<double>(1);
    b2g(2,3) = t_base2gripper[i].at<double>(2);

    std::cout << "trans before:\n" << b2g << std::endl;

    b2g = b2g * trans;


    std::cout << "trans after:\n" << b2g << std::endl;

    R_base2gripper[i].at<double>(0,0) = b2g(0,0);
    R_base2gripper[i].at<double>(0,1) = b2g(0,1);
    R_base2gripper[i].at<double>(0,2) = b2g(0,2);
    R_base2gripper[i].at<double>(1,0) = b2g(1,0);
    R_base2gripper[i].at<double>(1,1) = b2g(1,1);
    R_base2gripper[i].at<double>(1,2) = b2g(1,2);
    R_base2gripper[i].at<double>(2,0) = b2g(2,0);
    R_base2gripper[i].at<double>(2,1) = b2g(2,1);
    R_base2gripper[i].at<double>(2,2) = b2g(2,2);
    t_base2gripper[i].at<double>(0) = b2g(0,3);
    t_base2gripper[i].at<double>(1) = b2g(1,3);
    t_base2gripper[i].at<double>(2) = b2g(2,3);
  }

  // cv::imshow("chessboard detection", imgs[4]);
  // cv::waitKey(0);

  cv::Mat R_cam2base_est, t_cam2base_est;

  // std::cout << "R_base2gripper:\n" << R_base2gripper[0] << std::endl;
  // std::cout << "t_base2gripper:\n" << t_base2gripper[0] << std::endl;
  // std::cout << "R_t2c:\n" << R_t2c[0] << std::endl;
  // std::cout << "t_t2c:\n" << t_t2c[0] << std::endl;

  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_ANDREFF);
  std::cout << "CALIB_HAND_EYE_ANDREFF" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est << std::endl;
  std::cout << "t_cam2base_est:\n" << t_cam2base_est << std::endl;
  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_DANIILIDIS);
  std::cout << "CALIB_HAND_EYE_DANIILIDIS" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est << std::endl;
  std::cout << "t_cam2base_est:\n" << t_cam2base_est << std::endl;
  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_HORAUD);
  std::cout << "CALIB_HAND_EYE_HORAUD" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est << std::endl;
  std::cout << "t_cam2base_est:\n" << t_cam2base_est << std::endl;
  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_PARK);
  std::cout << "CALIB_HAND_EYE_PARK" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est << std::endl;
  std::cout << "t_cam2base_est:\n" << t_cam2base_est << std::endl;
  calibrateHandEye(R_base2gripper, t_base2gripper, R_t2c, t_t2c, R_cam2base_est, t_cam2base_est,
                   cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_TSAI);
  std::cout << "CALIB_HAND_EYE_TSAI" << std::endl;
  std::cout << "R_cam2base_est:\n" << R_cam2base_est << std::endl;
  std::cout << "t_cam2base_est:\n" << t_cam2base_est << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration");

  calibrate();

  return 0;
}
