/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;

    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);  //initiated on heap //

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // Generate PCL point cloud
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    // ProcessPointClouds<pcl::PointXYZ> pointProcessor; // instatiating point processor on stack ... //

    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    float clusterTolerance = 0.4;
    int minClusterSize = 10;
    int maxClusterSize = 600;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
 
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)

// for single frame data file //
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS

    /*
    // for single frame data file //
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    */

    // Generate PCL point cloud
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // Experiment with the ? values and find what works best
    float filterResolution = 0.3;
    Eigen::Vector4f minPoint(-10, -6, -3, 1);
    Eigen::Vector4f maxPoint(30, 6, 4, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterResolution, minPoint, maxPoint);
    
    renderPointCloud(viewer, filterCloud, "filterCloud");

    int maxIterations = 100;
    float distanceTolerance = 0.2;

    // Segmenting the lidar points into obstacles and road points //
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIterations, distanceTolerance);
    renderPointCloud(viewer, segmentCloud.first, "obsCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    float clusterTolerance = 0.4;
    int minClusterSize = 15;
    int maxClusterSize = 600;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);

    // Render box around host vehicle //

    Box ego_car = {-1.5, -1.2, -1, 2.6, 1.2, -0.2};
    renderBox(viewer, ego_car, 0, Color(0.5, 0.5, 0.5), 0.8);

    int clusterId = 1;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "Cluster Size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}



void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    //  set camera position and angle
    viewer->initCameraParameters();
    //  distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    //  setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS};

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}


int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    //simpleHighway(viewer);
    //cityBlock(viewer);

    /*
    // for single frame data file
    while (!viewer->wasStopped())
    {
      viewer->spinOnce();
    }
    */
   
    // ----------------------------------- For CityBlock Simulation ---------------------------------- //
    // for continuous stream of images //    
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    // ---------------------------------- For CityBlock Simulation ----------------------------------- //

    while(!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }

    // --------------------------------- End of CityBlock Simulation -------------------------------- //

}
