#pragma once

#include "PhysicsCore.h"
#include "PhysicsCollision.h"

class PhysicsInterface
{
public:
	virtual bool RayCast3D(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput) = 0;
	virtual bool RayCast2D(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput) = 0;
	virtual bool detectCollision(vector<AABB*> &boxes, Octree* octree, bool test_z_axis) = 0;
	virtual bool continuousCollision(D3DXVECTOR3 Obj1, D3DXVECTOR3 Obj2);
	virtual void setAccel(D3DXVECTOR3 acceleration) = 0;
	virtual void setAABB(D3DXVECTOR3 min, D3DXVECTOR3 max);
	virtual void setAABB(D3DXVECTOR3 min, D3DXVECTOR3 max, bool useContinuousCollisionDetection);
	virtual void setAABB(D3DXVECTOR3 min, D3DXVECTOR3 max, string ID);
	virtual void setAABB(D3DXVECTOR3 min, D3DXVECTOR3 max, string ID, bool useContinuousCollisionDetection);
	virtual D3DXVECTOR3 getAccel() = 0;
	virtual void setVel(D3DXVECTOR3 velocity) = 0;
	virtual D3DXVECTOR3 getVel() = 0;
};