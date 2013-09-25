#pragma once
#include "PhysicsCore.h"
#include "Octree.h"

class PhysicsCollision
{
public:
	PhysicsCollision(void);
	~PhysicsCollision(void);

	void ContinuousCollisionDetection();
	bool CollisionDetection(AABB shapeOne, AABB shapeTwo, Octree* octree, bool test_z_axis);
};

