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
	virtual bool CCD(D3DXVECTOR2 Obj1_centerPoint_current, D3DXVECTOR2 Obj1_centerPoint_future, D3DXVECTOR2 Obj1_extent, D3DXVECTOR2 Obj1_velocity,
							D3DXVECTOR2 Obj2_centerPoint_current, D3DXVECTOR2 Obj2_centerPoint_future, D3DXVECTOR2 Obj2_extent,	D3DXVECTOR2 Obj2_velocity,
							float deltaTime, float &timeOfImpact);
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

