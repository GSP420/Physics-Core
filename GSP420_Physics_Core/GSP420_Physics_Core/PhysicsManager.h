#pragma once

#include "PhysicsInterface.h"

class PhysicsManager : public PhysicsInterface
{
public:
	PhysicsManager(void);
	~PhysicsManager(void);
	
	virtual void useRaycast(bool RayCast) override;
	virtual void useContinuous(bool Continuous) override;
	virtual bool collision() override;
	virtual void setAccel( D3DXVECTOR3 acceleration) override;
	virtual D3DXVECTOR3 getVel() override;
	void StartUp();
	void Shutdown();
	void Update(float dt);

private:
	bool rayCast;
	bool contiuous;
	PhysicsCore core;
	PhysicsCollision collide;
};

