# include <pose_estimation3d/pose_estimation3d.h>

PoseEstimation3D::PoseEstimation3D()
{

}

// Inspired by lecture 6
inline float PoseEstimation3D::dist_sq(const pcl::Histogram<153> &query, const pcl::Histogram<153> &target)
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
void PoseEstimation3D::nearest_feature(const pcl::Histogram<153> &query, const pcl::PointCloud<pcl::Histogram<153>> &target, int &idx, float &distsq)
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
void PoseEstimation3D::spatialFilter(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &output_cloud, double min_lim, double max_lim, std::string fieldName)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointNormal> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName(fieldName);
  pass.setFilterLimits(min_lim, max_lim);
  pass.filter(*output_cloud);
}

// Get largest plane from image inspired by: https://answers.ros.org/question/67099/pcl-detecting-planes-in-the-map/
void PoseEstimation3D::findLargestPlane(pcl::PointCloud<pcl::PointNormal>::Ptr &input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &output_cloud)
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
  std::cout << "PointCloud representing the planar component: " << output_cloud->width * output_cloud->height << " data points remaining." << std::endl;
}

pcl::PointNormal PoseEstimation3D::findCentroid(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud)
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

Eigen::Matrix3f PoseEstimation3D::findOrientation(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud,  Eigen::Vector4f centroidPN)
{
  Eigen::Vector4f centroid;
  centroid = centroidPN;
  // centroid[0] = centroidPN.g;
  // centroid[1] = centroidPN.y;
  // centroid[2] = centroidPN.z;
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

void PoseEstimation3D::executePoseEstimation(bool visualize, std::string scene_path, std::string template_path)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_template(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_p(new pcl::PointCloud<pcl::PointNormal>);

  // Read in the cloud data
  pcl::PCDReader pcd_reader;
  pcl::PLYReader ply_reader;
  // pcd_reader.read("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/pose_estimation3d/screw2.pcd", *cloud);	 // Target
  // ply_reader.read("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/templates/screw.ply", *cloud_template); // Template
  pcd_reader.read(scene_path, *cloud);	 // Target
  ply_reader.read(template_path, *cloud_template); // Template

  for (size_t i = 0; i < cloud_template->size(); i++)
  {
    cloud_template->at(i).x = cloud_template->at(i).x/1000.0;
    cloud_template->at(i).y = cloud_template->at(i).y/1000.0;
    cloud_template->at(i).z = cloud_template->at(i).z/1000.0;
  }

   std::cout << "PointCloud before voxel grid: " << cloud->width * cloud->height
            << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointNormal> vox;
  float leaf_size = 0.002;
  vox.setInputCloud(cloud);
  vox.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox.filter(*cloud);
  leaf_size = leaf_size * 0.8f;
  vox.setInputCloud(cloud_template);
  vox.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox.filter(*cloud_template);

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  transform_1(1,1) = -1;
  transform_1(2,2) = -1; 
  pcl::transformPointCloud (*cloud, *cloud, transform_1);

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
  spatialFilter(cloud, cloud_filtered, -0.9, -0.589, "z");
  spatialFilter(cloud_filtered, cloud_filtered, -0.036, 0.25, "y");
  spatialFilter(cloud_filtered, cloud_filtered, -0.15, 0.2, "x");

  // make point cloud of only object
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_object(cloud_template);

  std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
            << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

  // // Find largest plane to remove table top
  findLargestPlane(cloud_filtered, cloud_p);

  std::cout << "PointCloud before StatisticalOutlierRemoval: " << cloud_filtered->width * cloud_filtered->height
            << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl; 
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(5);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  std::cout << "PointCloud after StatisticalOutlierRemoval: " << cloud_filtered->width * cloud_filtered->height
            << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl; 

  // // Compute surface normals
  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
  ne.setKSearch(10);
  // estimate normals for template
  ne.setInputCloud(cloud_template);
  ne.compute(*cloud_template);
  // estimate normals for filtered scene
  ne.setInputCloud(cloud_filtered);
  ne.compute(*cloud_filtered);

  // // Compute shape features
  pcl::PointCloud<pcl::Histogram<153>>::Ptr template_features(new pcl::PointCloud<pcl::Histogram<153>>);
  pcl::PointCloud<pcl::Histogram<153>>::Ptr scene_features(new pcl::PointCloud<pcl::Histogram<153>>);

  pcl::SpinImageEstimation<pcl::PointNormal, pcl::PointNormal, pcl::Histogram<153>> spin;
  spin.setRadiusSearch(0.05);

  spin.setInputCloud(cloud_template);
  spin.setInputNormals(cloud_template);
  spin.compute(*template_features);

  spin.setInputCloud(cloud_filtered);
  spin.setInputNormals(cloud_filtered);
  spin.compute(*scene_features);

  // // Find feature matches
  std::cout << "Found: " << template_features->size() << " template features" << std::endl;
  std::cout << "Found: " << scene_features->size() << " scene features" << std::endl;
  pcl::Correspondences corr(template_features->size());
  for (size_t i = 0; i < template_features->size(); ++i)
  {
    corr[i].index_query = i;
    nearest_feature(template_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
  }

  // // Create a k-d tree for pick location
  pcl::search::KdTree<pcl::PointNormal> tree;
  tree.setInputCloud(cloud_filtered);

  // // Set RANSAC parameters
  const size_t iter = 5000;
  const float thressq = 0.0015 * 0.0015;

  // // Start RANSAC
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
    est.estimateRigidTransformation(*cloud_template, idxobj, *cloud_filtered, idxscn, T);

    // Apply pose
    pcl::transformPointCloud(*cloud_template, *object_aligned, T);

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
    const float outlier_rate = 1.0f - float(inliers) / cloud_filtered->size();
    // const float penaltyi = rmse;
    const float penaltyi = outlier_rate;

    // Update result
    if (penaltyi < penalty)
    {
      std::cout << "\t--> Got a new model with " << inliers << " inliers!" << std::endl;
      penalty = penaltyi;
      pose = T;
      if (inliers >= scene_features->size()*0.9)
      {
        std::cout << inliers << " points out of " << scene_features->size() << " points for model are inliers. Ending RANSAC" << std::endl;
        break;
      }
    }
  }

  pcl::transformPointCloud(*cloud_template, *object_aligned, pose);

  // // Compute inliers and RMSE
  std::vector<std::vector<int>> idx;
  std::vector<std::vector<float>> distsq;
  tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
  size_t inliers = 0;
  float rmse = 0;
  for (size_t i = 0; i < distsq.size(); ++i)
  {
    // std::cout << "distsq[j][0] " << distsq[i][0] << std::endl;
    if (distsq[i][0] <= thressq)
    {
      ++inliers, rmse += distsq[i][0];
    }
  }
  rmse = sqrtf(rmse / inliers);

  // // Print pose
  std::cout << "Got the following pose:" << std::endl
            << pose << std::endl;
  std::cout << "Inliers: " << inliers << "/" << scene_features->size() << std::endl;
  std::cout << "RMSE: " << rmse << std::endl;

  // Inspired by lecture 6
  // Create a k-d tree for scene ICP
  pcl::search::KdTree<pcl::PointNormal> treeICP;
  treeICP.setInputCloud(cloud_filtered);

  // Set ICP parameters
  const size_t iter2 = 50;
  const float thressq2 = 0.002 * 0.002;

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
    est.estimateRigidTransformation(*object_alignedICP, idxobj, *cloud_filtered, idxscn, T);

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
  Eigen::Vector4f centroidPoint;
  pcl::compute3DCentroid(*object_alignedICP, centroidPoint);
  // Find centroid of template
  Eigen::Vector4f centroidPointTemplate;
  pcl::compute3DCentroid(*cloud_template, centroidPointTemplate);
  // // find orientation of template and object
  Eigen::Matrix3f orientationTemplate, orientationObject;
  orientationTemplate = findOrientation(cloud_template, centroidPointTemplate);
  orientationObject = findOrientation(object_alignedICP, centroidPoint);

  // Find axis angle representation difference  Inspired by https://math.stackexchange.com/questions/2113634/comparing-two-rotation-matrices
  Eigen::Matrix3f rot = orientationTemplate.transpose() * orientationObject;

  double angle = std::acos((rot.trace() - 1) / 2) * 180 / 3.1415926536;
  std::cout << "ANGLE " << angle << std::endl;

  // visualize template on target square
  if (visualize)
  {
    pcl::visualization::PCLVisualizer v("Calculated position in pick location");
    v.addPointCloud<pcl::PointNormal>(object_aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
    v.addPointCloud<pcl::PointNormal>(object_alignedICP, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_alignedICP, 0, 0, 255), "object_alignedICP");
    v.addPointCloud<pcl::PointNormal>(cloud_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_filtered, 255, 0, 0), "scene");
    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "object_aligned");
    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "object_alignedICP");
    v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "scene");
    
    v.spin();
  }
  
}
