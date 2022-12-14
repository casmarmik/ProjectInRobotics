#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/io/ply_io.h>

class PoseEstimation3D
{
public:
  PoseEstimation3D();
  void executePoseEstimation(bool visualize);
private:
  inline float dist_sq(const pcl::Histogram<153> &query, const pcl::Histogram<153> &target);
  void nearest_feature(const pcl::Histogram<153> &query, const pcl::PointCloud<pcl::Histogram<153>> &target, int &idx, float &distsq);
  void spatialFilter(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &output_cloud, double min_lim, double max_lim, std::string fieldName);
  void findLargestPlane(pcl::PointCloud<pcl::PointNormal>::Ptr &input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &output_cloud);
  pcl::PointNormal findCentroid(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud);
  Eigen::Matrix3f findOrientation(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud,  Eigen::Vector4f centroidPN);
};