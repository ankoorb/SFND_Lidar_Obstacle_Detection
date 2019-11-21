/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
        int dim = point.size() >= 3 ? 3 : 2;
		insertHelper(root, point, id, dim);

	}

	void insertHelper(Node*& node, std::vector<float> point, int id, int dim=2, int depth=0){
		if (node == nullptr){
			node = getNode(point, id);
		} else {
		    int currDepth = depth % dim;
            if (point[currDepth] < node->point[currDepth]){
                insertHelper(node->left, point, id, dim, depth+1);
            } else {
                insertHelper(node->right, point, id, dim, depth+1);
            }
		}
	}

	Node* getNode(std::vector<float> point, int id){
		Node* node = new Node(point, id);
		return node;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}

};


//// First try is not DRY
//void insert(std::vector<float> point, int id)
//{
//    // TODO: Fill in this function to insert a new point into the tree
//    // the function should create a new node and place correctly with in the root
//    insertHelper(root, point, id);
//
//}
//
//void insertHelper(Node*& node, std::vector<float> point, int id, int depth=0){
//    if (node == nullptr){
//        node = getNode(point, id);
//    } else {
//        if (depth % 2 == 0){
//            if (point[0] < node->point[0]){
//                insertHelper(node->left, point, id, depth+1);
//            } else {
//                insertHelper(node->right, point, id, depth+1);
//            }
//        } else {
//            if (point[1] < node->point[1]){
//                insertHelper(node->left, point, id, depth+1);
//            } else {
//                insertHelper(node->right, point, id, depth+1);
//            }
//        }
//    }
//}
//
//Node* getNode(std::vector<float> point, int id){
//    Node* node = new Node(point, id);
//    return node;
//}


