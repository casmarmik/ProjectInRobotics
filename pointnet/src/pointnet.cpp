#include "pointnet/pointnet.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/spin_image.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Dense>

namespace pointnet
{
PointNet::PointNet(const ros::NodeHandle& nh) : n_(nh)
{
  sub_point_ = n_.subscribe(CAMERA_TOPIC, 1000, &PointNet::cameraCallback, this);
}

void PointNet::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
  pcl_cloud_ = temp_cloud;
}
}  // namespace pointnet

pcl::PointCloud<pcl::PointXYZ>::Ptr readPCDFile(std::string filepath)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1)  //* load the file
  {
    std::cout << "Couldn't read file " << filepath << std::endl;
    ;
    return NULL;
  }
  return cloud;
}

void spatialFilter(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud,
                   pcl::PointCloud<pcl::PointNormal>::Ptr& output_cloud, double min_lim, double max_lim,
                   std::string fieldName)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointNormal> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName(fieldName);
  pass.setFilterLimits(min_lim, max_lim);
  pass.filter(*output_cloud);
}

// Get largest plane from image inspired by: https://answers.ros.org/question/67099/pcl-detecting-planes-in-the-map/
void findLargestPlane(pcl::PointCloud<pcl::PointNormal>::Ptr& input_cloud,
                      pcl::PointCloud<pcl::PointNormal>::Ptr& output_cloud)
{
  // Get segmentation ready
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointNormal> seg;
  pcl::ExtractIndices<pcl::PointNormal> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  // create segmentation
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  // Check result
  if (inliers->indices.size() == 0)
  {
    std::cout << "No inliers" << std::endl;
  }

  // extract inliers
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*output_cloud);
  input_cloud.swap(output_cloud);
  std::cout << "PointCloud representing the planar component: " << output_cloud->width * output_cloud->height
            << " data points removed." << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointnet");

  ros::NodeHandle nh;
  pointnet::PointNet pn(nh);

  // Load point cloud from file
  std::string filepath = "src/project_in_robotics/pointnet/data/backwall.pcd";
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::io::loadPCDFile(filepath, *cloud);

  // Transform point cloud
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(1, 1) = -1;
  transform(2, 2) = -1;
  pcl::transformPointCloud(*cloud, *cloud, transform);

  // Create the voxel grid filtering object
  pcl::VoxelGrid<pcl::PointNormal> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(0.002f, 0.002f, 0.002f);
  vox.filter(*cloud);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(25);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud);

  // Remove all point outside defined limits
  spatialFilter(cloud, cloud, -0.7, 0, "z");

  // Find largest plane to remove table top
  // findLargestPlane(cloud, cloud);

  // TODO
  // Link to pose estimation: https://pcl.readthedocs.io/projects/tutorials/en/pcl-1.11.0/alignment_prerejective.html
  // Need to get point cloud of objects that we want to find

  // Convert cloud for visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*cloud, *cloud_xyz);

  // blocks until the cloud is actually rendered
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(cloud_xyz);
  while (!viewer.wasStopped())
  {
  }
  // while (ros::ok())
  // {
  //   ros::Duration(0.01).sleep();
  //   ros::spinOnce();
  // }

  return 0;
}