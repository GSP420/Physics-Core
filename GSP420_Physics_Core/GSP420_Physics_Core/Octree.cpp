/********************************************************************************
*File: Octree.cpp
*Author: John Berg
*Date: 09/22/13
*Purpose: implents the code for our octree
********************************************************************************/
#include "Octree.h"

const float Octree::TIME_TO_NEXT_UPDATE = 0.01f;

//adds or removes an AABB from the children of the octree
void Octree::handleBoundingBoxes(AABB* aabb, D3DXVECTOR3 pos, bool addAABB)
{
	//Figure out in which child(ren) the AABB belongs
	for(int x = 0; x < 2; x++) 
	{
		if (x == 0) {
			if (pos[0] - aabb->extent().x > center[0]) 
			{
				continue;
			}
		}
		else if (pos[0] + aabb->extent().x < center[0]) 
		{
			continue;
		}
				
		for(int y = 0; y < 2; y++) 
		{
			if (y == 0) {
				if (pos[1] - aabb->extent().y > center[1]) 
				{
					continue;
				}
			}
			else if (pos[1] + aabb->extent().y < center[1]) 
			{
				continue;
			}
					
			for(int z = 0; z < 2; z++) {
				if (z == 0) {
					if (pos[2] - aabb->extent().z > center[2]) 
					{
						continue;
					}
				}
				else if (pos[2] + aabb->extent().z < center[2]) 
				{
					continue;
				}
						
				//Add or remove the ball
				if (addAABB) 
				{
					children[x][y][z]->add(aabb);
				}
				else 
				{
					children[x][y][z]->remove(aabb, pos);
				}
			}
		}
	}
}

//creates children of the octree and moves the AABB's in the children
void Octree::haveChildren()
{
	for(int x = 0; x < 2; x++) 
	{
		float minX;
		float maxX;
		if (x == 0) 
		{
			minX = corner1[0];
			maxX = center[0];
		}
		else 
		{
			minX = center[0];
			maxX = corner2[0];
		}
				
		for(int y = 0; y < 2; y++) 
		{
			float minY;
			float maxY;
			if (y == 0) 
			{
				minY = corner1[1];
				maxY = center[1];
			}
			else 
			{
				minY = center[1];
				maxY = corner2[1];
			}
					
			for(int z = 0; z < 2; z++) 
			{
				float minZ;
				float maxZ;
				if (z == 0) 
				{
					minZ = corner1[2];
					maxZ = center[2];
				}
				else 
				{
					minZ = center[2];
					maxZ = corner2[2];
				}
						
				children[x][y][z] = new Octree(D3DXVECTOR3(minX, minY, minZ),
											   D3DXVECTOR3(maxX, maxY, maxZ),
											   depth + 1);
			}
		}
	}

	//Remove all AABBs from "boundingBoxes" and add them to the new children
	for(set<AABB*>::iterator it = boundingBoxes.begin(); it != boundingBoxes.end(); it++) 
	{
		AABB* aabb = *it;
		handleBoundingBoxes(aabb, aabb->center(), true);
	}
	
	boundingBoxes.clear();
	hasChildren = true;
}

//adds all AABB's of the octree into a specified set 
void Octree::collectBoundingBoxes(set<AABB*> &aabbSet)
{
	if (hasChildren) 
	{
		for(int x = 0; x < 2; x++) 
		{
			for(int y = 0; y < 2; y++) 
			{
				for(int z = 0; z < 2; z++) 
				{
					children[x][y][z]->collectBoundingBoxes(aabbSet);
				}
			}
		}
	}
	else 
	{
		for(set<AABB*>::iterator it = boundingBoxes.begin(); it != boundingBoxes.end(); it++) 
		{
					AABB* aabb = *it;
					aabbSet.insert(aabb);
		}
	}
}

//Destroys the children of the octree, and moves all AABB's in its descendants
//to the "boundingBoxes" set
void Octree::destroyChildren()
{
	collectBoundingBoxes(boundingBoxes);
			
	for(int x = 0; x < 2; x++) 
	{
		for(int y = 0; y < 2; y++) 
		{
			for(int z = 0; z < 2; z++) 
			{
				delete children[x][y][z];
			}
		}
	}
			
	hasChildren = false;
}

//removes an AABB at a specified position
void Octree::remove(AABB* aabb, D3DXVECTOR3 pos)
{
	numBoundingBoxes--;
			
	if (hasChildren && numBoundingBoxes < MIN_AABBS_PER_OCTREE) 
	{
		destroyChildren();
	}
			
	if (hasChildren) 
	{
		handleBoundingBoxes(aabb, pos, false);
	}
	else 
	{
		boundingBoxes.erase(aabb);
	}
}

//constructs a new octree
Octree::Octree(D3DXVECTOR3 c1, D3DXVECTOR3 c2, int d)
{
	corner1 = c1;
	corner2 = c2;
	center = (c1 + c2) / 2;
	depth = d;
	numBoundingBoxes = 0;
	hasChildren = false;
}

//destructs the octrees along with all their children
Octree::~Octree()
{
		if (hasChildren) 
	{
		destroyChildren();
	}
}

void Octree::add(AABB* aabb)
{
	numBoundingBoxes++;
	if (!hasChildren && depth < MAX_OCTREE_DEPTH && numBoundingBoxes > MAX_AABBS_PER_OCTREE) 
	{
		haveChildren();
	}
			
	if (hasChildren) 
	{
		handleBoundingBoxes(aabb, aabb->center(), true);
	}
	else 
	{
		boundingBoxes.insert(aabb);
	}
}

void Octree::remove(AABB* aabb)
{
	remove(aabb, aabb->center());
}

void Octree::boundingBoxMoved(AABB* aabb, D3DXVECTOR3 oldPos)
{
	remove(aabb, oldPos);
	add(aabb);
}

//Adds potential AABB-AABB collisions to the specified set
void Octree::potentialBoxBoxCollision(vector<AABBPair> &collisions)
{
	if (hasChildren) 
	{
		for(int x = 0; x < 2; x++) 
		{
			for(int y = 0; y < 2; y++) 
			{
				for(int z = 0; z < 2; z++) 
				{
					children[x][y][z]->potentialBoxBoxCollision(collisions);
				}
			}
		}
	}
	else 
	{
		//Add all pairs (aabb1, aabb2) from boundingBoxes
		for(set<AABB*>::iterator it = boundingBoxes.begin(); it != boundingBoxes.end(); it++) 
		{
			AABB* aabb1 = *it;
			for(set<AABB*>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); it2++) 
			{
				AABB* aabb2 = *it2;
				//This test makes sure that we only add each pair once
				if (aabb1 < aabb2) 
				{
					AABBPair bp;
					bp.aabb1 = aabb1;
					bp.aabb2 = aabb2;
					collisions.push_back(bp);
				}
			}
		}
	}
}

//Puts potential Box-Box collisions in potentialCollisions
//this function must return all actual collisions but it doesn't to return only actual collisions
void Octree::potentialBoxBoxCollision(vector<AABBPair> &potentialCollisions, vector<AABB*> &boxes, Octree* octree)
{
	octree->potentialBoxBoxCollision(potentialCollisions);
}

void Octree::moveBoxes(vector<AABB*> &boxes, Octree* octree, float dt)
{
	for(int i = 0; i << boxes.size(); i++)
	{
		AABB* aabb = boxes[i];
		D3DXVECTOR3 oldPos = aabb->extent();
		aabb->extent() += core.velocity * dt;
		octree->boundingBoxMoved(aabb, oldPos);
	}
}

void Octree::advance(vector<AABB*> &boxes, Octree* octree, float t, float &timeUntilUpdate)
{
	while(t > 0)
	{
		if(timeUntilUpdate <= t)
		{
			moveBoxes(boxes, octree, timeUntilUpdate);
			t -= timeUntilUpdate;
			timeUntilUpdate = TIME_TO_NEXT_UPDATE;
		}
		else
		{
			moveBoxes(boxes, octree, t);
			timeUntilUpdate -= t;
			t = 0;
		}
	}
}