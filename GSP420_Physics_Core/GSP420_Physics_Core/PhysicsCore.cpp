#include "PhysicsCore.h"


PhysicsCore::PhysicsCore(void)
{
}


PhysicsCore::~PhysicsCore(void)
{
}


void PhysicsCore::Accelerate()
{
}


void PhysicsCore::SpatialPartitioning()
{
}


bool PhysicsCore::RayCast(D3DXVECTOR2 startPoint, D3DXVECTOR2 endPoint, list<AABB> collidables, int maxTestLimit)
{
	/******************************************************
	*	Function Name:		RayCast()
	*	Programmer:			Josh Archer
	*
	*	Determines if an AABB in the collidables list
	*	lies along the path of the line between
	*	startPoint and endPoint.
	*
	*	Returns TRUE if a collision has occured with one
	*	of the collidables,
	*	Returns FALSE if we have reached the test limit
	*	or if no collisions were detected
	******************************************************/

	//Create and initialize an iterator for the collidables list.
	list<AABB>::iterator i = collidables.begin();

	//Iterate through all collidables in the list and test for collisions along the ray
	while(i != collidables.end())
	{
		if(i->collidables.maxPoint.x < startPoint.x || endPoint.x < i->collidables.minPoint.x)
		{
			//The projections along the x-axis are disjoint, so there is no collision with the current AABB
		}
		else
		{
			if(i->collidables.maxPoint.y < startPoint.y || endPoint.y < i->collidables.minPoint.y)
			{
				//The projections along the y-axis are disjoint, so there is no collision with the current AABB
			}
			else
			{
				//The shapes' projection along both axes are intersecting, so there is a collision with the current AABB
				return TRUE;
			}
		}
		
		//Move to the next collidable in the list
		i++;

		//Check if we have hit the limit of collidables to test, and break the while loop if we have
		if(i > maxTestLimit)
		{
			break;
		}
	}

	//If we get to this point in the function, none of the AABB's in the collidables list were colliding with the ray, or we have reached our test limit
	return FALSE;
}


void PhysicsCore::CollisionMaskLayers()
{
}


double PhysicsCore::GetVelocity()
{
  return velocity;
}


void PhysicsCore::SetVelocity(double Vel)
{
  velocity = Vel;
}


double PhysicsCore::GetAcceleration()
{
  return acceleration;
}


void PhysicsCore::SetAcceleration(double Accel)
{
  acceleration = Accel;
}
