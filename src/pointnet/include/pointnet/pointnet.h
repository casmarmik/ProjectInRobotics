#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

namespace pointnet
{
class PointNet
{
public:
    PointNet(const ros::NodeHandle& nh);
    virtual ~PointNet() = default;


private:
    void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    ros::NodeHandle n_;
    ros::Subscriber sub_point_;
    std::string CAMERA_TOPIC = "camera/depth/points";
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_;
};
} // namespace pointnet
