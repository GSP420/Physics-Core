#pragma once

#include "PhysicsInterface.h"

class PhysicsManager : public PhysicsInterface
{
public:
	vector<AABB*> boxes;

	PhysicsManager(void);
	~PhysicsManager(void);
	
	virtual bool RayCast(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput) override;
	virtual bool RayCast(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput) override;
	virtual bool collision(vector<AABB*> &boxes, Octree* octree, bool test_z_axis) override;
	virtual bool collision(D3DXVECTOR3 Obj1, D3DXVECTOR3 Obj2) override;
	virtual void setAccel( D3DXVECTOR3 acceleration) override;
	virtual D3DXVECTOR3 getVel() override;
	void StartUp();
	void Shutdown();
	void Update(float dt);

private:
	PhysicsCore core;
	PhysicsCollision collide;
	Octree* octree;
	float timeUntilUpdate;
};

