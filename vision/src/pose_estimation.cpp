#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>

#include <rw/rw.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders.hpp>

// Inspired by lecture 6
inline float dist_sq(const pcl::Histogram<153> &query, const pcl::Histogram<153> &target)
{
  float result = 0.0;
  for (int i = 0; i < pcl::Histogram<153>::descriptorSize(); ++i)
  {
    const float diff = reinterpret_cast<const float *>(&query)[i] - reinterpret_cast<const float *>(&target)[i];
    result += diff * diff;
  }

  return result;
}

// Inspired by lecture 6
void nearest_feature(const pcl::Histogram<153> &query, const pcl::PointCloud<pcl::Histogram<153>> &target, int &idx, float &distsq)
{
  idx = 0;
  distsq = dist_sq(query, target[0]);
  for (size_t i = 1; i < target.size(); ++i)
  {
    const float disti = dist_sq(query, target[i]);
    if (disti < distsq)
    {
      idx = i;
      distsq = disti;
    }
  }
}

// Inspired by preprocessing.cpp
void spatialFilter(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &output_cloud, double min_lim, double max_lim, std::string fieldName)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointNormal> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName(fieldName);
  pass.setFilterLimits(min_lim, max_lim);
  pass.filter(*output_cloud);
}

// Get largest plane from image inspired by: https://answers.ros.org/question/67099/pcl-detecting-planes-in-the-map/
void findLargestPlane(pcl::PointCloud<pcl::PointNormal>::Ptr &input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &output_cloud)
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
  std::cout << "PointCloud representing the planar component: " << output_cloud->width * output_cloud->height << " data points removed." << std::endl;
}

pcl::PointNormal findCentroid(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud)
{
  double max_dist = 0;
  int index1 = -1, index2 = -1;
  // Brute force to find largest distance between two points as there are not that many points
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    for (size_t j = 0; j < input_cloud->size(); j++)
    {
      double dist = pcl::squaredEuclideanDistance(input_cloud->at(i), input_cloud->at(j));
      if (dist > max_dist)
      {
        index1 = i;
        index2 = j;
        max_dist = dist;
      }
    }
  }

  // find centroid from the two farthest points
  pcl::CentroidPoint<pcl::PointNormal> centroid;
  centroid.add(input_cloud->at(index1));
  centroid.add(input_cloud->at(index2));

  std::cout << "point" << index1 << ": " << input_cloud->at(index1) << std::endl;
  std::cout << "point" << index2 << ": " << input_cloud->at(index2) << std::endl;

  pcl::PointNormal centroidPoint;
  centroid.get(centroidPoint);

  std::cout << "Centroid point found at: " << centroidPoint << std::endl;

  return centroidPoint;
}

Eigen::Matrix3f findOrientation(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud, pcl::PointNormal centroidPN)
{
  Eigen::Vector4f centroid;
  centroid[0] = centroidPN.x;
  centroid[1] = centroidPN.y;
  centroid[2] = centroidPN.z;
  centroid[3] = 1;
  Eigen::Matrix3f covariance_matrix;

  // Extract the eigenvalues and eigenvectors
  Eigen::Vector3f eigen_values;
  Eigen::Matrix3f eigen_vectors;

  // Compute the 3x3 covariance matrix
  pcl::computeCovarianceMatrix(*input_cloud, centroid, covariance_matrix);
  pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);

  return eigen_vectors;
}

// Inspired by https://www.embeddedrelated.com/showcode/311.php
double AWGN_generator()
{ /* Generates additive white Gaussian Noise samples with zero mean and a standard deviation of 1. */

  double temp1;
  double temp2;
  double result;
  int p;
  double PI = 3.1415926536;
  p = 1;

  while (p > 0)
  {
    temp2 = (rand() / ((double)RAND_MAX)); /*  rand() function generates an
                                                   integer between 0 and  RAND_MAX,
                                                   which is defined in stdlib.h.
                                               */

    if (temp2 == 0)
    { // temp2 is >= (RAND_MAX / 2)
      p = 1;
    } // end if
    else
    { // temp2 is < (RAND_MAX / 2)
      p = -1;
    } // end else

  } // end while()

  temp1 = cos((2.0 * (double)PI) * rand() / ((double)RAND_MAX));
  result = sqrt(-2.0 * log(temp2)) * temp1;

  return result; // return the generated random sample to the caller
}

void addNormalNoise(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, double std_dev)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointNormal noisePoint;
  for (size_t i = 0; i < cloud->size(); i++)
  {
    noisePoint.x = cloud->at(i).x + std_dev * AWGN_generator();
    noisePoint.y = cloud->at(i).y + std_dev * AWGN_generator();
    noisePoint.z = cloud->at(i).z + std_dev * AWGN_generator();
    cloud_temp->push_back(noisePoint);
  }
  cloud.swap(cloud_temp);
}

void executeSquare(bool visualize)
{
  rw::common::TimerUtil::sleepMs(300);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_p(new pcl::PointCloud<pcl::PointNormal>);

  // Read in the cloud data
  pcl::PCDReader reader;
  reader.read("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/p1.pcd", *cloud);	 // Template
  reader.read("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/p1.pcd", *cloud2); // Target

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointNormal> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud);

  sor.setInputCloud(cloud2);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud2);

  // get coordinates to help spacial filtering
  pcl::PointNormal minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;

  std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
            << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

  // Filter away points not in pick location
  spatialFilter(cloud, cloud_filtered, -1.45, -1.0, "z");
  spatialFilter(cloud_filtered, cloud_filtered, -0.5, 0.15, "y");

  spatialFilter(cloud2, cloud2, -1.45, -1.0, "z");
  spatialFilter(cloud2, cloud2, -0.5, 0.15, "y");

  // Add noise for test
  addNormalNoise(cloud2, noise);

  // make point cloud of only object
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_object(cloud_filtered);

  std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
            << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

  // Find largest plane to remove table top
  findLargestPlane(cloud_object, cloud_p);

  // write cloud with squre for visualization
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal>("cloud_filtered.pcd", *cloud_filtered, false);
  writer.write<pcl::PointNormal>("scene.pcd", *cloud, false);
  writer.write<pcl::PointNormal>("cloud_object.pcd", *cloud_object, false);

  // Compute surface normals
  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
  ne.setKSearch(10);
  // estimate normals for object (square)
  ne.setInputCloud(cloud_object);
  ne.compute(*cloud_object);
  // estimate normals for scene
  ne.setInputCloud(cloud_filtered);
  ne.compute(*cloud_filtered);

  // Compute shape features
  pcl::PointCloud<pcl::Histogram<153>>::Ptr object_features(new pcl::PointCloud<pcl::Histogram<153>>);
  pcl::PointCloud<pcl::Histogram<153>>::Ptr scene_features(new pcl::PointCloud<pcl::Histogram<153>>);

  pcl::SpinImageEstimation<pcl::PointNormal, pcl::PointNormal, pcl::Histogram<153>> spin;
  spin.setRadiusSearch(0.05);

  spin.setInputCloud(cloud_object);
  spin.setInputNormals(cloud_object);
  spin.compute(*object_features);

  spin.setInputCloud(cloud_filtered);
  spin.setInputNormals(cloud_filtered);
  spin.compute(*scene_features);

  // Find feature matches
  std::cout << "Found: " << object_features->size() << " object features" << std::endl;
  pcl::Correspondences corr(object_features->size());
  for (size_t i = 0; i < object_features->size(); ++i)
  {
    corr[i].index_query = i;
    nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
  }

  // Create a k-d tree for pick location
  pcl::search::KdTree<pcl::PointNormal> tree;
  tree.setInputCloud(cloud2);

  // Set RANSAC parameters
  const size_t iter = 5000;
  const float thressq = 0.0225 * 0.0225;

  // Start RANSAC
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned(new pcl::PointCloud<pcl::PointNormal>);
  float penalty = FLT_MAX;
  std::cout << "Starting RANSAC..." << std::endl;
  pcl::common::UniformGenerator<int> gen(0, corr.size() - 1);
  for (size_t i = 0; i < iter; ++i)
  {
    // Sample 3 random correspondences
    std::vector<int> idxobj(3);
    std::vector<int> idxscn(3);
    for (int j = 0; j < 3; ++j)
    {
      const int idx = gen.run();
      idxobj[j] = corr[idx].index_query;
      idxscn[j] = corr[idx].index_match;
    }

    // Estimate transformation
    Eigen::Matrix4f T;
    pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal> est;
    est.estimateRigidTransformation(*cloud_object, idxobj, *cloud2, idxscn, T);

    // Apply pose
    pcl::transformPointCloud(*cloud_object, *object_aligned, T);

    // Validate
    std::vector<std::vector<int>> idx;
    std::vector<std::vector<float>> distsq;
    tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

    // Compute inliers and RMSE
    size_t inliers = 0;
    float rmse = 0;
    for (size_t j = 0; j < distsq.size(); ++j)
    {
      if (distsq[j][0] <= thressq)
      {
        ++inliers, rmse += distsq[j][0];
      }
    }

    rmse = sqrtf(rmse / inliers);

    // Evaluate a penalty function
    const float outlier_rate = 1.0f - float(inliers) / cloud_object->size();
    // const float penaltyi = rmse;
    const float penaltyi = outlier_rate;

    // Update result
    if (penaltyi < penalty)
    {
      std::cout << "\t--> Got a new model with " << inliers << " inliers!" << std::endl;
      penalty = penaltyi;
      pose = T;
      if (inliers == object_features->size())
      {
        std::cout << "All points for model are inliers. Ending RANSAC" << std::endl;
        break;
      }
    }
  }

  pcl::transformPointCloud(*cloud_object, *object_aligned, pose);

  // Compute inliers and RMSE
  std::vector<std::vector<int>> idx;
  std::vector<std::vector<float>> distsq;
  tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
  size_t inliers = 0;
  float rmse = 0;
  for (size_t i = 0; i < distsq.size(); ++i)
  {
    if (distsq[i][0] <= thressq)
    {
      ++inliers, rmse += distsq[i][0];
    }
  }
  rmse = sqrtf(rmse / inliers);

  // Print pose
  std::cout << "Got the following pose:" << std::endl
            << pose << std::endl;
  std::cout << "Inliers: " << inliers << "/" << cloud_object->size() << std::endl;
  std::cout << "RMSE: " << rmse << std::endl;

  // Inspired by lecture 6
  // Create a k-d tree for scene ICP
  pcl::search::KdTree<pcl::PointNormal> treeICP;
  treeICP.setInputCloud(cloud2);

  // Set ICP parameters
  const size_t iter2 = 50;
  const float thressq2 = 0.0255 * 0.0225;

  // Start ICP
  Eigen::Matrix4f pose2 = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointNormal>::Ptr object_alignedICP(new pcl::PointCloud<pcl::PointNormal>(*object_aligned));

  std::cout << "Starting ICP..." << std::endl;
  for (std::size_t i = 0; i < iter2; ++i)
  {
    // 1) Find closest points
    std::vector<std::vector<int>> idx;
    std::vector<std::vector<float>> distsq;
    treeICP.nearestKSearch(*object_alignedICP, std::vector<int>(), 1, idx, distsq);

    // Threshold and create indices for object/scene and compute RMSE
    std::vector<int> idxobj;
    std::vector<int> idxscn;
    for (size_t j = 0; j < idx.size(); ++j)
    {
      if (distsq[j][0] <= thressq2)
      {
        idxobj.push_back(j);
        idxscn.push_back(idx[j][0]);
      }
    }

    // 2) Estimate transformation
    Eigen::Matrix4f T;
    pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal> est;
    est.estimateRigidTransformation(*object_alignedICP, idxobj, *cloud2, idxscn, T);

    // 3) Apply pose
    pcl::transformPointCloud(*object_alignedICP, *object_alignedICP, T);

    // 4) Update result
    pose2 = T * pose2;
  }

  // Compute inliers and RMSE
  std::vector<std::vector<int>> idxICP;
  std::vector<std::vector<float>> distsqICP;
  treeICP.nearestKSearch(*object_alignedICP, std::vector<int>(), 1, idx, distsq);
  inliers = 0;
  rmse = 0;
  for (size_t i = 0; i < distsq.size(); ++i)
  {
    if (distsq[i][0] <= thressq2)
    {
      ++inliers, rmse += distsq[i][0];
    }
  }
  rmse = sqrtf(rmse / inliers);

  // Print pose
  std::cout << "Got the following pose:" << std::endl
            << pose2 << std::endl;
  std::cout << "Inliers: " << inliers << "/" << object_alignedICP->size() << std::endl;
  std::cout << "RMSE: " << rmse << std::endl;

  // Find centroid of square
  pcl::PointNormal centroidPoint;
  centroidPoint = findCentroid(object_alignedICP);
  // Find centroid of template
  pcl::PointNormal centroidPointTemplate;
  centroidPointTemplate = findCentroid(cloud_object);
  // find orientation of template and object
  Eigen::Matrix3f orientationTemplate, orientationObject;
  orientationTemplate = findOrientation(cloud_object, centroidPointTemplate);
  orientationObject = findOrientation(object_alignedICP, centroidPoint);

  // Find axis angle representation difference  Inspired by https://math.stackexchange.com/questions/2113634/comparing-two-rotation-matrices
  Eigen::Matrix3f rot = orientationTemplate.transpose() * orientationObject;

  double angle = std::acos((rot.trace() - 1) / 2) * 180 / 3.1415926536;
  std::cout << "ANGLE " << angle << std::endl;

  // Save data in file
  std::ofstream fileAng;
  fileAng.open("testNoiseAng", std::ios_base::app);
  fileAng << noise << ", " << angle << "\n";
  fileAng.close();

  // Load the workcell and device
  rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../../Project_WorkCell_Obstacle/Scene.wc.xml");
  rw::kinematics::Frame *cameraFrame25D = wc->findFrame("Scanner25D");

  // Find moveable grapsing object
  rw::kinematics::MovableFrame::Ptr movSquareFrame = wc->findFrame<rw::kinematics::MovableFrame>("Square");

  // get the default state
  rw::kinematics::State state = wc->getDefaultState();

  // set tranformation of square in scanner frame
  rw::math::Transform3D<> fTmf;
  fTmf.P()[0] = centroidPoint.x;
  fTmf.P()[1] = centroidPoint.y;
  fTmf.P()[2] = centroidPoint.z;
  fTmf.R() = rw::math::Rotation3D<>(1, 0, 0, 0, 1, 0, 0, 0, 1);

  // get scanner frame in world frame
  rw::math::Transform3D<> wTmf = cameraFrame25D->wTf(state);

  // calculate square in world frame
  rw::math::Transform3D<> wTf = wTmf * fTmf;

  std::cout << "Calculated position: " << wTf << std::endl
            << "Actual position: " << movSquareFrame->wTf(state) << std::endl;

  std::vector<double> diff = {std::abs(wTf.P()[0] - movSquareFrame->wTf(state).P()[0]), std::abs(wTf.P()[1] - movSquareFrame->wTf(state).P()[1]), std::abs(wTf.P()[2] - movSquareFrame->wTf(state).P()[2])};
  double diffMag = std::sqrt(std::pow(diff[0], 2) + std::pow(diff[1], 2) + std::pow(diff[2], 2));

  std::ofstream file;
  file.open("testNoisePos", std::ios_base::app);
  file << noise << ", " << diffMag << "\n";
  file.close();

  // visualize template on target square
  if (visualize)
  {
    pcl::visualization::PCLVisualizer v("Calculated position in pick location");
    v.addPointCloud<pcl::PointNormal>(object_alignedICP, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_alignedICP, 0, 255, 0), "object_aligned");
    v.addPointCloud<pcl::PointNormal>(cloud2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud2, 255, 0, 0), "scene");
    v.spin();
  }
  
}

// Inspired by preprocessing.cpp and exercises from lecture 6
int main(int argc, char **argv)
{
  bool visualize = true;
  executeSquare(visualize);

  return 0;
}