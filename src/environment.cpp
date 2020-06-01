// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  Car egoCar(Vect3(0,  0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(  Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(  Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(  Vect3(-12,4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}


/// @deprecated
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;//true;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  /// Create Lidar sensor
  Lidar* lidarSensor = new Lidar(cars, 0.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidarSensor->scan();
  //renderRays(viewer, lidarSensor->position, inputCloud);
  //renderPointCloud(viewer, inputCloud, "inputCloud", Color(1.0, 1.0, 0.0));

  /// Create point processor
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;

  //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
  //            pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
    pointProcessor.RansacPlane(inputCloud, 100, 0.2);

  // Comment out renderPointCloud to render only clusters
  renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1.0, 0, 0));
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1.0, 0));

  /// Clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(
    segmentCloud.first, 1.0, 3, 30);

  int clusterId = 0;

  std::vector<Color> colors = {
    Color(1, 0, 0),
    Color(1, 1, 0),
    Color(0, 0, 1)
  };

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    // Render points of the cluster
    std::cout << "cluster size ";
    pointProcessor.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

    // Render bounding box around cluster
    Box box = pointProcessor.BoundingBox(cluster);
    renderBox(viewer, box, clusterId);

    ++clusterId;
  }
}


/// @brief Open 3D viewer and display City Block
void cityBlock(
  pcl::visualization::PCLVisualizer::Ptr&   viewer,
  ProcessPointClouds<pcl::PointXYZI>* const pointProcessorI,
  pcl::PointCloud<pcl::PointXYZI>::Ptr&     inputCloud)
{
  // Step 0: Load PCD-file
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud =
  //  pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  // Step 1: Apply voxel grid filter
  inputCloud = pointProcessorI->FilterCloud(
    inputCloud,
    0.3 ,
    Eigen::Vector4f (-20, -6, -3, 1),
    Eigen::Vector4f ( 30, 7, 2, 1));
  //renderPointCloud(viewer, inputCloud, "inputCloud");

  // Step 2: Segment the filtered cloud into two parts, road and obstacles
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
    pointProcessorI->RansacPlane(inputCloud, 100, 0.2);

  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
  renderPointCloud(viewer, segmentCloud.first,  "obstCloud", Color(1, 0, 0));

  // Step 3: Cluster the obstacle cloud
  KdTree* tree = new KdTree;
  for (int i = 0; i < segmentCloud.first->points.size(); ++i) {
    tree->insert(segmentCloud.first->points[i], i);
  }
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(
    segmentCloud.first, tree, 0.35, 15, 500);

  int clusterId = 0;

  std::vector<Color> colors = {
    Color(1, 0, 0),
    Color(1, 1, 0),
    Color(0, 0, 1)
  };

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    // Render points of the cluster
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

    // Step 4: Render bounding box around cluster
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);

    ++clusterId;
  }

  delete tree;
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(
  CameraAngle                             setAngle,
  pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  viewer->setBackgroundColor (0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch(setAngle)
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

  if (setAngle!=FPS) {
    viewer->addCoordinateSystem(1.0);
  }
}


int main (int argc, char** argv)
{
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");

  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped ()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load PCD and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    ++streamIterator;
    if (streamIterator == stream.end()) {
      streamIterator = stream.begin();
    }

    viewer->spinOnce();
  }

  delete pointProcessorI;

  return 0;
}
