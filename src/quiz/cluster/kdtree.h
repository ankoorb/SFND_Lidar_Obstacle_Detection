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

	// Search helper
	bool isInBox(std::vector<float> target, std::vector<float> point, float distanceTol){

        if (target.size() != point.size()) {
            std::cerr << "target dimensions should match point dimensions." << std::endl;
        }

	    bool status = true;
	    for (int i=0; i<target.size(); i++){
	        status *= (target[i] - distanceTol <= point[i]) && (target[i] + distanceTol >= point[i]);
	    }
	    return status;
	}

	float getDistance(std::vector<float> target, std::vector<float> point){

        if (target.size() != point.size()) {
            std::cerr << "target dimensions should match point dimensions." << std::endl;
        }
	    float distance = 0.0f;
        for (int i=0; i<target.size(); i++){
            float delta = target[i] - point[i];
            distance += delta * delta;
        }
        return sqrt(distance);
	}

	void searchHelper(Node*& node, std::vector<float> target, std::vector<int>* ids, float distanceTol, int dim=2, int depth=0){
	    // If nodes left/right child is not null
	    if (node != nullptr){

            // If point is in box and its distance is within threshold then add it
            if (isInBox(target, node->point, distanceTol)){
                if (getDistance(target, node->point) <= distanceTol){
                ids->emplace_back(node->id);
                }
            }

            // Visit left sub-tree or right sub-tree or both (box is intersecting boundary)
            int currDepth = depth % dim;

            if (target[currDepth] - distanceTol < node->point[currDepth]){
                searchHelper(node->left, target, ids, distanceTol, dim, depth+1);
            }

            if (target[currDepth] + distanceTol > node->point[currDepth]){
                searchHelper(node->right, target, ids, distanceTol, dim, depth+1);
            }
	    }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        int dim = target.size() >= 3 ? 3 : 2;
		searchHelper(root, target, &ids, distanceTol, dim);
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


//bool isInBox(std::vector<float> target, std::vector<float> point, float distanceTol){
//
//    if (target.size() != point.size()) {
//        std::cerr << "target dimensions should match point dimensions." << std::endl;
//    }
//
//    bool status = true;
//    for (int i=0; i<target.size(); i++){
//        status *= (target[i] - distanceTol <= point[i]) && (target[i] + distanceTol >= point[i]);
//    }
//    return status;
//}
//
//float getDistance(std::vector<float> target, std::vector<float> point){
//
//    if (target.size() != point.size()) {
//        std::cerr << "target dimensions should match point dimensions." << std::endl;
//    }
//    float distance = 0.0f;
//    for (int i=0; i<target.size(); i++){
//        float delta = target[i] - point[i];
//        distance += delta * delta;
//    }
//    return sqrt(distance);
//}
//
//void searchHelper(Node*& node, std::vector<float> target, std::vector<int>* ids, float distanceTol, int dim=2, int depth=0){
//    // Case: Node left/right child points to null
//    if (node == nullptr){
//        return;
//    }
//    // Case: Leaf node
//    if (node->left == nullptr && node->right == nullptr){
//        if (getDistance(target, node->point) <= distanceTol){
//            ids->emplace_back(node->id);
//        }
//    } else {
//        int currDepth = depth % dim;
//        if (isInBox(target, node->point, distanceTol)){
//            if (getDistance(target, node->point) <= distanceTol){
//                ids->emplace_back(node->id);
//            }
//        }
//
//        // Check if need to visit left sub-tree or right sub-tree or both
//        if (target[currDepth] - distanceTol <= node->point[currDepth] && target[currDepth] + distanceTol <= node->point[currDepth]){
//            searchHelper(node->left, target, ids, distanceTol, dim, depth+1);
//        } else if (target[currDepth] - distanceTol >= node->point[currDepth] && target[currDepth] + distanceTol >= node->point[currDepth]){
//            searchHelper(node->right, target, ids, distanceTol, dim, depth+1);
//        } else {
//            searchHelper(node->left, target, ids, distanceTol, dim, depth+1);
//            searchHelper(node->right, target, ids, distanceTol, dim, depth+1);
//        }
//    }
//}
//
//// return a list of point ids in the tree that are within distance of target
//std::vector<int> search(std::vector<float> target, float distanceTol)
//{
//    std::vector<int> ids;
//    int dim = target.size() >= 3 ? 3 : 2;
//    searchHelper(root, target, &ids, distanceTol, dim);
//    return ids;
//}


