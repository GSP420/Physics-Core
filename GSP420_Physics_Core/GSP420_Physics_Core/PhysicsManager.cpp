#include "PhysicsManager.h"


PhysicsManager::PhysicsManager(void)
{
}


PhysicsManager::~PhysicsManager(void)
{
}


void PhysicsManager::useRaycast(bool RayCast)
{
	rayCast = RayCast;
}


void PhysicsManager::useContinuous(bool Continuous)
{
	contiuous = Continuous;
}

//Not sure about this one yet
bool PhysicsManager::collision()
{
	if(contiuous)
	{
		//collide.ContinuousCollisionDetection(
	}
	else
	{
		//collide.CollisionDetection(
	}
}


void PhysicsManager::setAccel(D3DXVECTOR3 acceleration)
{
	core.SetAcceleration(acceleration);
}


D3DXVECTOR3 PhysicsManager::getVel()
{
	return core.velocity;
}


void PhysicsManager::StartUp()
{
	core.velocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);
	core.acceleration = D3DXVECTOR3(0.0f, float(core.GRAVITY), 0.0f);
}


void PhysicsManager::Shutdown()
{

}


void PhysicsManager::Update(float dt)
{
	core.Accelerate(dt);
}
