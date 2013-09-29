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
	bool ContinuousCollisionDetection(D3DXVECTOR3 Obj1_centerPoint_previous, D3DXVECTOR3 Obj1_centerPoint_current, D3DXVECTOR3 Obj1_extent, 
													D3DXVECTOR3 Obj2_centerPoint_previous, D3DXVECTOR3 Obj2_centerPoint_current, D3DXVECTOR3 Obj2_extent,
													float deltaTime, float &timeToImpact);
	bool CollisionDetection(vector<AABB*> &boxes, Octree* octree, bool test_z_axis);
};

