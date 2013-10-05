#pragma once

#include "PhysicsCore.h"
#include "PhysicsCollision.h"
#include "Octree.h"

class PhysicsInterface
{
public:
	vector<AABB*> boxes;

	void StartUp(int SCENE_SIZE);
	void Update(float deltaTime);
	void Shutdown();

	bool RayCast3D(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput);
	bool RayCast2D(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput);
	void setAccel(D3DXVECTOR3 acceleration);
	void setAABB(D3DXVECTOR3 min, D3DXVECTOR3 max);
	void setAABB(D3DXVECTOR3 min, D3DXVECTOR3 max, bool useContinuousCollisionDetection);
	void setAABB(D3DXVECTOR3 min, D3DXVECTOR3 max, string ID);
	void setAABB(D3DXVECTOR3 min, D3DXVECTOR3 max, string ID, bool useContinuousCollisionDetection);
	D3DXVECTOR3 getAccel();
	void setVel(D3DXVECTOR3 velocity);
	D3DXVECTOR3 getVel();

private:
	PhysicsCore core;
	PhysicsCollision collide;
	Octree* octree;
	float timeUntilUpdate;
};

void PhysicsInterface::StartUp(int SCENE_SIZE)
{
	core.velocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);
	core.acceleration = D3DXVECTOR3(0.0f, float(core.GRAVITY), 0.0f);
	timeUntilUpdate = 0.0f;

	octree = new Octree(D3DXVECTOR3(-SCENE_SIZE / 2, -SCENE_SIZE / 2, -SCENE_SIZE / 2),
						D3DXVECTOR3(SCENE_SIZE / 2, SCENE_SIZE / 2, SCENE_SIZE / 2),
						0.0f);
}

void PhysicsInterface::Update(float dt)
{
	core.Accelerate(dt);
	octree->advance(boxes, octree, dt, timeUntilUpdate);

	//Loop through the boxes for broad phase
	for(int i = 0; i < boxes.size(); i++)
	{
		//Broad phase collision detection
		//TODO:Call CollisionDetection() function to check for collisions
	}

	//Loop through the boxes a second time for narrow phase
	for(int i = 0; i < boxes.size(); i++)
	{
		//Narrow phase collision detection
		if(boxes[i]->useContinuousDetection)
		{
			//box has continuous detection toggled on, so determine if we've already found a collision for it in broad phase
			//TODO:Call sweptCCD() function to check for collision if a collision wasn't already detected, otherwise do nothing
		}
	}
}

void PhysicsInterface::Shutdown()
{
	for(unsigned int i = 0; i < boxes.size(); i++)
		delete boxes[i];

	 delete octree;
}

bool PhysicsInterface::RayCast3D(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput)
{
	return core.RayCast(startPoint, directionVector, collidables, maxTestLimit, contactOutput);
}

bool PhysicsInterface::RayCast2D(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput)
{
	return core.RayCast(startPoint, directionVector, collidables, maxTestLimit, contactOutput);
}

void PhysicsInterface::setAccel(D3DXVECTOR3 acceleration)
{
	core.SetAcceleration(acceleration);
}


D3DXVECTOR3 PhysicsInterface::getAccel()
{
	return core.GetAcceleration();
}


void PhysicsInterface::setVel(D3DXVECTOR3 velocity)
{
	core.SetVelocity(velocity);
}


D3DXVECTOR3 PhysicsInterface::getVel()
{
	return core.velocity;
}
