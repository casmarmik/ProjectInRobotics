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

pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(std::string filepath)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filepath, *cloud) == -1) //* load the file
  {
    std::cout << "Couldn't read file " << filepath << std::endl;;
    return NULL;
  }
  return cloud;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pointnet");

  ros::NodeHandle nh;
  pointnet::PointNet pn(nh);

  std::string filepath = "/home/marcus/pir/ros_ws/src/pointnet/data/1665043838010637.pcd";
  
  
  // while (ros::ok())
  // {
  //   ros::Duration(0.01).sleep();
  //   ros::spinOnce();
  // }


  return 0;
}