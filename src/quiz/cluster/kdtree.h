// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd-tree
struct Node
{
  std::vector<float> point;
  int   id;
  Node* left;
  Node* right;

  Node(
    std::vector<float> arr,
    int                setId)
  : point (arr)
  , id    (setId)
  , left  (nullptr)
  , right (nullptr)
  {}
};


struct KdTree
{
  Node* root;

  KdTree()
  : root(nullptr)
  {}


  void insertHelper(
    Node**             node,
    uint               depth,
    std::vector<float> point,
    int                id)
  {
    // Tree is empty
    if (*node == nullptr) {
      *node = new Node(point, id);
    }
    else {
      // Calculate current dim
      uint cd = depth % 2;

      if (point[cd] < ((*node)->point[cd])) {
        insertHelper(&((*node)->left), depth + 1, point, id);
      }
      else {
        insertHelper(&((*node)->right), depth + 1, point, id);
      }
    }
  }


  /// @brief Insert point to the tree
  /// @param[in] point - 2D point represented by a vector containing two floats
  /// @param[in] id - Unique identifier of the point
  void insert(
    std::vector<float> point,
    int                id)
  {
    // This function inserts a new point into the tree.
    // The function should create a new node and place correctly with in the root
    insertHelper(&root, 0, point, id);
  }

  // Return a list of point ids in the tree that are within distance of target
  std::vector<int> search(
    std::vector<float> target,
    float              distanceTol)
  {
    std::vector<int> ids;
    return ids;
  }
};
