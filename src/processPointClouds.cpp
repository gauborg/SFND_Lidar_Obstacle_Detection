#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds(){}


//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds(){}



template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::cout << cloud->points.size() << std::endl;
}


template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

  // Timing segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  std::vector<int> indices;

  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloudRegion;
}


// ------------------------------------ Gaurav Borgaonkar RANSAC Algorithm Implementation ------------------------------------- //

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  // For max iterations
  // Randomly sample subset and fit line
  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier
  // Return indicies of inliers from fitted line with most inliers

  while (maxIterations--)
  {
    // Randomly pick two points//

    std::unordered_set<int> inliers;
    while (inliers.size() < 3)
      inliers.insert(rand() % (cloud->points.size()));

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    auto itr = inliers.begin();

    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    z1 = cloud->points[*itr].z;
    itr++;

    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;
    z2 = cloud->points[*itr].z;
    itr++;

    x3 = cloud->points[*itr].x;
    y3 = cloud->points[*itr].y;
    z3 = cloud->points[*itr].z;

    float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

    // Calculate distance of the point //
    float D = -1 * (A * x1 + B * y1 + C * z1);

    for (int index = 0; index < cloud->points.size(); index++)
    {
      if (inliers.count(index) > 0)
        continue;
      
      else
      {
        float x4 = cloud->points[index].x;
        float y4 = cloud->points[index].y;
        float z4 = cloud->points[index].z;
        float d = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(A * A + B * B + C * C);

        if (d <= distanceTol)
          inliers.insert(index);
      }
    }

    if (inliers.size() > inliersResult.size())
      inliersResult = inliers;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "RANSAC algorithm took " << elapsedTime.count() << " milliseconds ..." << std::endl;

  return inliersResult;
}

// ---------------------------------------- End of Gaurav Borgaonkar RANSAC Algorithm ----------------------------------------- //

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

  for (int index : inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  /*
  pcl::PointIndices::Ptr inliers;

  // Original PCL Implemetation //

  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);         // maxIterations = 1000
  seg.setDistanceThreshold(distanceThreshold); // distanceThreshold = 0.01

  // Segment the largest planar component from the input cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Original PCL segmentation END //
  */

  // TODO:: Fill in this function to find inliers for the cloud //
  // Gaurav Borgaonkar segmentation //

  std::unordered_set<int> inliersSet = Ransac(cloud, maxIterations, distanceThreshold);
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int i : inliersSet)
    inliers->indices.push_back(i);


  if (inliers->indices.size() == 0)
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Plane segmentation took " << elapsedTime.count() << " milliseconds ..." << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
  return segResult;

}



template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  /*
  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

  // Original PCL Implementation //

  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusters_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters_indices);
  */

  // Gaurav Borgaonkar Clustering Algortihm //

  KdTree *tree = new KdTree;
  std::vector<std::vector<float>> points;

  for (int i = 0; i < cloud->points.size(); i++)
  {
    std::vector<float> point({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    points.push_back(point);
    tree->insert(points[i], i);
  }
  std::vector<std::vector<int>> clusters_indices = euclideanCluster(points, tree, clusterTolerance);

  // End of Gaurav Borgaonkar Clustering //

  // for (pcl::PointIndices getIndices : clusters_indices) // Original PCL implementation //
  for (std::vector<int> getIndices : clusters_indices) // My implementation //
  {
    typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

    // for (int index : getIndices.indices) //PCL implementation
    for (int index : getIndices) // My implementation //
      cloudCluster->points.push_back(cloud->points[index]);

    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;

    // For setting minimum and maximum cluster sizes //

    if (cloudCluster->width >= minSize && cloudCluster->width <= maxSize)
      clusters.push_back(cloudCluster);

  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}



template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}



template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}



template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}