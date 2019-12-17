/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i + scatter*rx;
  		point.y = i + scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int maxIterations, float distanceTol)
{
	
	std::unordered_set<int> inliersResult;
	
	srand(time(NULL)); // This makes use of computer's internal clock to control seed choice... Since time is continuosly changing,
						// seed also forever changing...//
						
	// This is for 3D pointcloud model ... //
	
	while (maxIterations--)
	{
		// randomly sample subset and select three points for plane fitting ... //
		std::unordered_set<int> inliers;
		// selects two inliers randomly from the cloud //
	
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->width));
		
		// initialized an iterator //
		// Get coordinates of three points //
			
		auto itr = inliers.begin();
		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
			
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
			
		// coordinates of point 2 //
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		
		// coordinates of point 3 //
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
			
		// Calculate coefficients A, B, C and D from the scalar product of vectors v1 and v2 for normal vector plane ... //
		
		float a = ((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1));
		float b = ((z2 - z1)*(x3 - x1)) - ((x2 - x1)*(z3 - z1));
		float c = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1));
		
		float d = (-1)*((a*x1) + (b*y1) + (c*z1));
			
		// Calculate distances for each of the points in the cloud iteratively//
			
		for (int i = 0; i < cloud->width; i++)
		{
			float x_coor, y_coor, z_coor;
			x_coor = cloud->points[i].x;
			y_coor = cloud->points[i].y;
			z_coor = cloud->points[i].z;
				
			// calculate the distance of the point ... //
			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier //
				
			float point_dist;
			point_dist = fabs((a*x_coor + b*y_coor + c*z_coor + d) / (sqrt(a*a + b*b + c*c)));
				
			// classification of point - whether the point is an inlier or a outlier ... //
				
			if (point_dist < distanceTol)
				inliers.insert(i);
		}
			
		// Create a list of inliers to form a fitted plane //
			
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	
	return inliersResult;
	
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	auto startTime = std::chrono::steady_clock::now();
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.25);
	
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac algorithm for plane segmentation took " << elapsedTime.count() << " milliseconds..." << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
