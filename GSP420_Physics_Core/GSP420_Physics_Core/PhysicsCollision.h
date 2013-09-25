#pragma once
#include "PhysicsCore.h"
#include "Octree.h"

class PhysicsCollision
{
	Octree* _octree;

public:
	PhysicsCollision(void);
	~PhysicsCollision(void);

	void ContinuousCollisionDetection();
	vector<AABBPair> CollisionDetection(vector<AABB*> &boxes, Octree* octree, bool test_z_axis);
};

