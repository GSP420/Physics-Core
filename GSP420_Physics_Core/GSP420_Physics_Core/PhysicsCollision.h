#pragma once
#include "PhysicsCore.h"

class PhysicsCollision
{
public:
	PhysicsCollision(void);
	~PhysicsCollision(void);

	void ContinuousCollisionDetection();
	bool CollisionDetection(AABB shapeOne, AABB shapeTwo, bool test_z_axis);
};

