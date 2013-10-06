#pragma once
#include "PhysicsCore.h"

class PhysicsCollision
{
public:
	PhysicsCore core;

	PhysicsCollision(void);
	~PhysicsCollision(void);

	D3DXVECTOR3 ObjectDistance(D3DXVECTOR3, D3DXVECTOR3);
	bool PhysicsCollision::CCD(D3DXVECTOR2, D3DXVECTOR2, D3DXVECTOR2,
							D3DXVECTOR2, D3DXVECTOR2, D3DXVECTOR2,
							float, float);
	bool PhysicsCollision::CCD(D3DXVECTOR2 Obj1_centerPoint_current, D3DXVECTOR2 Obj1_centerPoint_future, D3DXVECTOR2 Obj1_extent, D3DXVECTOR2 Obj1_velocity,
													D3DXVECTOR2 Obj2_centerPoint_current, D3DXVECTOR2 Obj2_centerPoint_future, D3DXVECTOR2 Obj2_extent,
													D3DXVECTOR2 Obj2_velocity, float deltaTime, float &timeOfImpact);
	bool PhysicsCollision::sweptCCD(vector<AABB*> boxes, Octree* octree, float &timeOfImpact);
	bool CollisionDetection(vector<AABB*> &boxes, Octree* octree, bool test_z_axis);
};

