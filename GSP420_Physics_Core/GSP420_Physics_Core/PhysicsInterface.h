#pragma once

#include "PhysicsCore.h"
#include "PhysicsCollision.h"

class PhysicsInterface
{
public:
	virtual void useRaycast(bool Raycast) = 0;
	virtual void useContinuous(bool Contiuous) = 0;
	virtual bool collision() = 0;
	virtual void setAccel(D3DXVECTOR3 acceleration) = 0;
	virtual D3DXVECTOR3 getVel() = 0;
};