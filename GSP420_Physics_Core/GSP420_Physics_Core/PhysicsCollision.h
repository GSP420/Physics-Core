#pragma once
#include "PhysicsCore.h"
#include "Octree.h"

class PhysicsCollision
{
	Octree* _octree;

public:
	PhysicsCollision(void);
	~PhysicsCollision(void);

	D3DXVECTOR3 ObjectDistance(D3DXVECTOR3, D3DXVECTOR3);
	void ContinuousCollisionDetection(D3DXVECTOR3, D3DXVECTOR3);
	vector<AABBPair> CollisionDetection(vector<AABB*> &boxes, Octree* octree, bool test_z_axis);
};

