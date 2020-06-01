// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <unordered_set>
#include <ctime>
#include <chrono>
#include "render/box.h"


/// @brief Structure to represent node of kd-tree
struct Node
{
  pcl::PointXYZI point;
  int            id;
  Node*          left;
  Node*          right;

  Node(
    pcl::PointXYZI arr,
    int            setId)
    : point (arr)
    , id    (setId)
    , left  (nullptr)
    , right (nullptr)
  {}
};


/// @brief @todo
struct KdTree
{
  Node* root;

  KdTree()
    : root(nullptr)
  {}


  void insertHelper(
    Node**         node,
    uint           depth,
    pcl::PointXYZI point,
    int            id)
  {
    // Tree is empty
    if (*node == nullptr) {
      *node = new Node(point, id);
    }
    else {
      // Calculate current dim (3D kd-tree)
      uint cd = depth % 3;

      if (cd == 0) {
        if (point.x < (*node)->point.x) {
          insertHelper(&(*node)->left, depth + 1, point, id);
        }
        else {
          insertHelper(&(*node)->right, depth + 1, point, id);
        }
      }
      else {
        if (point.y < (*node)->point.y) {
          insertHelper(&(*node)->left, depth + 1, point, id);
        }
        else {
          insertHelper(&(*node)->right, depth + 1, point, id);
        }
      }
    }
  }


  /// @brief Insert point to the tree
  /// @param[in] point - 2D point represented by a vector containing two floats
  /// @param[in] id - Unique identifier of the point
  void insert(
    pcl::PointXYZI point,
    int            id)
  {
    // This function inserts a new point into the tree.
    // The function should create a new node and place correctly with in the root
    insertHelper(&root, 0, point, id);
  }


  void searchHelper(
    pcl::PointXYZI    target,
    Node*             node,
    const int         depth,
    const float&      distanceTol,
    std::vector<int>& ids)
  {
    if (node != nullptr) {
      if ((node->point.x >= target.x - distanceTol &&  // Check for x-axis
           node->point.x <= target.x + distanceTol) &&
          (node->point.y >= target.y - distanceTol &&  // Check for y-axis
           node->point.y <= target.y + distanceTol)) {
        float distance = sqrt((node->point.x - target.x) * (node->point.x - target.x) +
                              (node->point.y - target.y) * (node->point.y - target.y));

        if (distance <= distanceTol) {
          ids.push_back(node->id);
        }
      }

      // 3D kd-tree
      if (depth % 3 == 0) {

        if ((target.x - distanceTol) < node->point.x) {
          searchHelper(target, node->left, depth + 1, distanceTol, ids);
        }

        if ((target.x + distanceTol) > node->point.x) {
          searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
      }
      else {

        if ((target.y - distanceTol) < node->point.y) {
          searchHelper(target, node->left, depth + 1, distanceTol, ids);
        }

        if ((target.y + distanceTol) > node->point.y) {
          searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
      }

    }
  }


  // Return a list of point ids in the tree that are within distance of target
  std::vector<int> search(
    pcl::PointXYZI target,
    const float&   distanceTol)
  {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);

    return ids;
  }
};


template<typename PointT>
class ProcessPointClouds {
public:

  // Constructor
  ProcessPointClouds();

  // Destructor
  ~ProcessPointClouds();

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  /// @brief Apply voxel grid filtering to the point cloud
  typename pcl::PointCloud<PointT>::Ptr FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float                                 filterRes,
    Eigen::Vector4f                       minPoint,
    Eigen::Vector4f                       maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(
    pcl::PointIndices::Ptr                inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud);

  /// @brief Deprecated
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int                                   maxIterations,
    float                                 distanceThreshold);

  /// @brief RANSAC for Plane (3D)
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
    typename pcl::PointCloud<PointT>::Ptr> RansacPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int                                   maxIterations,
    float                                 distanceThreshold);


  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float                                 clusterTolerance,
    int                                   minSize,
    int                                   maxSize);

  void clusterHelper(
    int                                   idx,
    typename pcl::PointCloud<PointT>::Ptr cloud,
    std::vector<int>&                     cluster,
    std::vector<bool>&                    processed,
    KdTree*                               tree,
    float                                 distanceTol);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    KdTree*                               tree,
    float                                 distanceTol,
    int                                   minSize,
    int                                   maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(
      typename pcl::PointCloud<PointT>::Ptr cloud,
      std::string                           file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};

#endif // PROCESSPOINTCLOUDS_H_
