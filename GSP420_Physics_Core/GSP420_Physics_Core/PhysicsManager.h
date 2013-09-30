#pragma once

#include "PhysicsInterface.h"

class PhysicsManager : public PhysicsInterface
{
public:
	vector<AABB*> boxes;

	PhysicsManager(void);
	~PhysicsManager(void);
	
	virtual bool RayCast3D(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput);
	virtual bool RayCast2D(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput);
	virtual bool detectCollision(vector<AABB*> &boxes, Octree* octree, bool test_z_axis);
	virtual bool continuousCollision(D3DXVECTOR3 Obj1, D3DXVECTOR3 Obj2);
	virtual void setAccel( D3DXVECTOR3 acceleration);
	virtual D3DXVECTOR3 getAccel();
	virtual D3DXVECTOR3 getVel();
	virtual void setVel(D3DXVECTOR3 velocity);
	
	void StartUp();
	void Shutdown();
	void Update(float dt);

private:
	PhysicsCore core;
	PhysicsCollision collide;
	Octree* octree;
	float timeUntilUpdate;
};

