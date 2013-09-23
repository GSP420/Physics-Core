/************************************************************************************************
*File: Octree.h
*Author:John Berg
*Date: 09/22/13
*Purpose: Octree creates a node structure that breaks down the game world into smaller
*boxes to check collisions in these boxes instead of checking for collision of everysingle object
*against each other.
************************************************************************************************/

#pragma once

#include "PhysicsCore.h"
#include <set>

class Octree
{
private:
	
	const int MAX_OCTREE_DEPTH = 6;			//keeps us from making to many nodes in one octree
	const int MIN_AABBS_PER_OCTREE = 2;		//if there is less than two then we no longer need that node
	const int MAX_AABBS_PER_OCTREE = 6;		//if there are more than 6 then we need to create another node

	D3DXVECTOR3 corner1;
	D3DXVECTOR3 corner2;
	D3DXVECTOR3 center;

	Octree *children[2][2][2];				//3 dimensional array for the nodes of the octree
	bool hasChildren;						//boolean indicating if there are children or not
	set<AABB*> boundingBoxes;				//the boundign boxes in the node if it doesn't have any children
	int depth;								//the depth of the tree
	int numBoundingBoxes;					//number of bounding boxes including in the children

	void handleBoundingBoxes(AABB* aabb, D3DXVECTOR3 pos, bool addAABB);
	void haveChildren();
	void collectBoundingBoxes(set<AABB*> &aabbSet);
	void destroyChildren();
	void remove(AABB* aabb, D3DXVECTOR3 pos);

public:

	Octree(D3DXVECTOR3 c1, D3DXVECTOR3 c2, int d);
	~Octree();
	void add(AABB* aabb);
	void remove(AABB* aabb);
	void boundingBoxMoved(AABB* aabb, D3DXVECTOR3 oldPos);
};
