#include "pointnet/pointnet.h"


namespace pointnet
{
    PointNet::PointNet(const ros::NodeHandle& nh) : n_(nh)
    {
        sub_point_ = n_.subscribe(CAMERA_TOPIC, 1000, &PointNet::cameraCallback, this);

    }

    void PointNet::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        pcl_cloud_ = temp_cloud;

    }
} // namespace pointnet


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pointnet");

  ros::NodeHandle nh;
  pointnet::PointNet pn(nh);

  
  while (ros::ok())
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }


  return 0;
}