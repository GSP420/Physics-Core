#pragma once

#include "PhysicsCore.h"
#include "PhysicsCollision.h"

class PhysicsInterface
{
public:
	virtual bool RayCast(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput) = 0;
	virtual bool RayCast(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput) = 0;
	virtual bool collision(vector<AABB*> &boxes, Octree* octree, bool test_z_axis) = 0;
	virtual bool collision(D3DXVECTOR3 Obj1, D3DXVECTOR3 Obj2);
	virtual void setAccel(D3DXVECTOR3 acceleration) = 0;
	virtual D3DXVECTOR3 getVel() = 0;
};