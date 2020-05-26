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


  void searchHelper(
    const std::vector<float>& target,
    Node*                     node,
    const int                 depth,
    const float&              distanceTol,
    std::vector<int>&         ids)
  {
    if (node != nullptr) {
      if ((node->point[0] >= target[0] - distanceTol &&  // Check for x-axis
           node->point[0] <= target[0] + distanceTol) &&
          (node->point[1] >= target[1] - distanceTol &&  // Check for y-axis
           node->point[1] <= target[1] + distanceTol)) {
        float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                              (node->point[1] - target[1]) * (node->point[1] - target[1]));

        if (distance <= distanceTol) {
          ids.push_back(node->id);
        }
      }

      // Check across boundary
      if (target[depth % 2] - distanceTol < node->point[depth % 2]) {
        searchHelper(target, node->left, depth + 1, distanceTol, ids);
      }
      if (target[depth % 2] + distanceTol > node->point[depth % 2]) {
        searchHelper(target, node->right, depth + 1, distanceTol, ids);
      }

    }
  }


  // Return a list of point ids in the tree that are within distance of target
  std::vector<int> search(
    std::vector<float> target,
    float              distanceTol)
  {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);

    return ids;
  }
};
