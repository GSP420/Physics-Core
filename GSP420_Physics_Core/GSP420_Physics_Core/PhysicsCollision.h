#pragma once
#include "PhysicsCore.h"
#include "Octree.h"

class PhysicsCollision
{
public:
	Octree* _octree;

	PhysicsCollision(void);
	~PhysicsCollision(void);

	D3DXVECTOR3 ObjectDistance(D3DXVECTOR3, D3DXVECTOR3);
	bool PhysicsCollision::CCD(D3DXVECTOR2, D3DXVECTOR2, D3DXVECTOR2,
							D3DXVECTOR2, D3DXVECTOR2, D3DXVECTOR2,
							float, float);
	bool PhysicsCollision::CCD(D3DXVECTOR2 Obj1_centerPoint_current, D3DXVECTOR2 Obj1_centerPoint_future, D3DXVECTOR2 Obj1_extent, D3DXVECTOR2 Obj1_velocity,
													D3DXVECTOR2 Obj2_centerPoint_current, D3DXVECTOR2 Obj2_centerPoint_future, D3DXVECTOR2 Obj2_extent,
													D3DXVECTOR2 Obj2_velocity, float deltaTime, float &timeOfImpact);
	bool PhysicsCollision::sweptCCD(D3DXVECTOR2 boxA_centerPoint_previous, D3DXVECTOR2 boxA_centerPoint_current, D3DXVECTOR2 boxA_extent,
							D3DXVECTOR2 boxB_centerPoint_previous, D3DXVECTOR2 boxB_centerPoint_current, D3DXVECTOR2 boxB_extent,
							float &timeOfImpact)
	bool CollisionDetection(vector<AABB*> &boxes, Octree* octree, bool test_z_axis);
};

