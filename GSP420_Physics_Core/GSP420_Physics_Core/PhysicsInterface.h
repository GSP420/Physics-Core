/****************************************************************************************
Example Pseudo Code
Main()
{
	PhysicsInterface physics;
	
	Startup()
	{
		physics.Startup();
	}

	Update()
	{
		physics.Update();
		{
			for(int i=0;i<physics.core._collisions->collisionList.size();i++)
			{
				//Pop the next collision off the stack
				physics.core._collisions->getNext();

				//Resolve collisions by accessing the following properties
				physics.core._collisions->currentCollision.boxA_ID;
				physics.core._collisions->currentCollision.boxB_ID;
				physics.core._collisions->currentCollision.boxA_movementVector;
				physics.core._collisions->currentCollision.boxB_movementVector;
				physics.core._collisions->currentCollision.impactPoint.x;
				physics.core._collisions->currentCollision.impactPoint.y;
				physics.core._collisions->currentCollision.impactPoint.z;
				physics.core._collisions->currentCollision.timeOfImpact;
			}
		}

		//Raycast example
		physics.RayCast3D(entity.position, entity.movementVector, physics.core.boxes, 15, 				physics.core._rayCastContact);

		//Returns true if a collision is detected along the ray
		//Assigns information about the collision to physics.core._rayCastContact struct
	}

	Shutdown()
	{
		physics.Shutdown();
	}
}
*******************************************************************************************/
#pragma once
#include "PhysicsCore.h"
#include "PhysicsCollision.h"
#include "ICore.h"

class PhysicsInterface : public ICore
{
public:

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
	float timeUntilUpdate;
};

void PhysicsInterface::StartUp(int SCENE_SIZE)
{
	core.velocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);
	core.acceleration = D3DXVECTOR3(0.0f, float(core.GRAVITY), 0.0f);
	timeUntilUpdate = 0.0f;

	collide._octree = new Octree(D3DXVECTOR3(-SCENE_SIZE / 2, -SCENE_SIZE / 2, -SCENE_SIZE / 2),
						D3DXVECTOR3(SCENE_SIZE / 2, SCENE_SIZE / 2, SCENE_SIZE / 2),
						0.0f);
}

void PhysicsInterface::Update(float dt)
{
	float TOI;
	core.Accelerate(dt);
	collide._octree->advance(core.boxes, collide._octree, dt, timeUntilUpdate);

	//Loop through the boxes for broad phase
	for(int i = 0; i < core.boxes.size(); i++)
	{
		//Broad phase collision detection
		if(collide.CollisionDetection(core.boxes, collide._octree, true))
		{
			//this box collided with another box
		}
		else
		{
			//this box didnt collide with anything
		}
	}

	//Loop through the boxes a second time for narrow phase
	for(int i = 0; i < core.boxes.size(); i++)
	{
		//Narrow phase collision detection
		if(core.boxes[i]->useContinuousDetection)
		{
			//box has continuous detection toggled on, so determine if we've already found a collision for it in broad phase
			if(collide.sweptCCD(core.boxes, collide._octree, TOI))
			{
				//this box collided with another box
			}
			else
			{
				//this box didnt collide with anything
			}
		}
	}

	//After calling update, core._collisions.collisionList will contain information about all the collisions that occurred and can be used
	//by the designer to resolve the collisions.
}

void PhysicsInterface::Shutdown()
{
	for(unsigned int i = 0; i < core.boxes.size(); i++)
		delete core.boxes[i];

	 delete collide._octree;
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


void PhysicsInterface::setAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint)
{
	core.SetAABB(minPoint, maxPoint);	
}


void PhysicsInterface::setAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, bool useCCD)
{
	core.SetAABB(minPoint, maxPoint, useCCD);	
}


void PhysicsInterface::setAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, string ID)
{
	core.SetAABB(minPoint, maxPoint, ID);	
}


void PhysicsInterface::setAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, string ID, bool useCCD)
{
	core.SetAABB(minPoint, maxPoint, useCCD, ID);	
}