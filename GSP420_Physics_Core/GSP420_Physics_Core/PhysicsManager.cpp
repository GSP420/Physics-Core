#include "PhysicsManager.h"


PhysicsManager::PhysicsManager(void)
{
}


PhysicsManager::~PhysicsManager(void)
{
}


bool PhysicsManager::RayCast3D(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput)
{
	return core.RayCast(startPoint, directionVector, collidables, maxTestLimit, contactOutput);
}


bool PhysicsManager::RayCast2D(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput)
{
	return core.RayCast(startPoint, directionVector, collidables, maxTestLimit, contactOutput);
}


bool PhysicsManager::detectCollision(vector<AABB*> &boxes, Octree* octree, bool test_z_axis)
{
	 return collide.CollisionDetection(boxes, octree, test_z_axis);
}


bool PhysicsManager::continuousCollision(D3DXVECTOR3 Obj1, D3DXVECTOR3 Obj2)
{
	return collide.ContinuousCollisionDetection(Obj1, Obj2);
}


void PhysicsManager::setAccel(D3DXVECTOR3 acceleration)
{
	core.SetAcceleration(acceleration);
}


D3DXVECTOR3 PhysicsManager::getAccel()
{
	return core.GetAcceleration();
}


void PhysicsManager::setVel(D3DXVECTOR3 velocity)
{
	core.SetVelocity(velocity);
}


D3DXVECTOR3 PhysicsManager::getVel()
{
	return core.velocity;
}


void PhysicsManager::StartUp()
{
	core.velocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);
	core.acceleration = D3DXVECTOR3(0.0f, float(core.GRAVITY), 0.0f);
	timeUntilUpdate = 0.0f;
}


void PhysicsManager::Shutdown()
{
	for(unsigned int i = 0; i < boxes.size(); i++)
		delete boxes[i];

	 delete octree;
}


void PhysicsManager::Update(float dt)
{
	core.Accelerate(dt);
	octree->advance(boxes, octree, dt, timeUntilUpdate); 
}
