//Adding Header Guard
#ifndef KNODE_H_
#define KNODE_H_

struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}


	void insertHelper(Node** rootNode, std::vector<float> point, int id, int depth){

		if (*rootNode == NULL)
		{
			*rootNode = new Node(point,id);
		}
		else 
		{
			int curr_depth;

			curr_depth = depth%3;
			
			if (point[curr_depth] < ((*rootNode)->point[curr_depth])) {

				insertHelper(&(*rootNode)->left,point,id,depth+1);	
			}
			else {
				
				insertHelper(&(*rootNode)->right,point,id,depth+1);
			
			}

		}

	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root,point,id,0);
	}

	float CalcDistToTarget(std::vector<float> node, std::vector<float> target)
	{
		float sumSquared = 0;
		for (size_t i = 0; i < node.size(); i++)
		{
			sumSquared += ((node[i] - target[i]) * (node[i] - target[i]));
		}
		double dist = sqrt(sumSquared);
		return dist;
	}

	std::vector<int> SearchHelper(Node *&node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids)
	{
		int coordIdx = depth % 3; // for 3D

		// Base 1 - if node is null
		if (node != NULL)
		{
			// Base 2 - node within bounding box
			if (node->point[0] <= (target[0] + distanceTol) && node->point[0] >= (target[0] - distanceTol) &&
				node->point[1] <= (target[1] + distanceTol) && node->point[1] >= (target[1] - distanceTol) &&
				node->point[2] <= (target[2] + distanceTol) && node->point[2] >= (target[2] - distanceTol))
			{
				float distToTarget = CalcDistToTarget(node->point, target);

				// Add node's ID only if node's distance is within distance tolerance
				if (distToTarget <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			// Recursive 1 - if node's cutting axis is intersecting bounding box
			if (node->point[coordIdx] <= (target[coordIdx] + distanceTol) && node->point[coordIdx] >= (target[coordIdx] - distanceTol))
			{
				SearchHelper(node->left, target, distanceTol, depth + 1, ids);
				SearchHelper(node->right, target, distanceTol, depth + 1, ids);
			}

			// Recursive 2a - if node's cutting axis is NOT intersecting bounding box - min bound
			else if (node->point[coordIdx] < (target[coordIdx] - distanceTol))
			{
				SearchHelper(node->right, target, distanceTol, depth + 1, ids);
			}

			// Recursive 2b - if node's cutting axis is NOT intersecting bounding box - max bound
			else
			{
				SearchHelper(node->left, target, distanceTol, depth + 1, ids);
			}
		}

		return ids;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		SearchHelper(root, target, distanceTol, 0, ids);

		return ids;
	}
	
};

#endif