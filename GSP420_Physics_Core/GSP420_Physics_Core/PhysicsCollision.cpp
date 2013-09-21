#include "PhysicsCollision.h"


PhysicsCollision::PhysicsCollision(void)
{
}


PhysicsCollision::~PhysicsCollision(void)
{
}


void PhysicsCollision::ContinuousCollisionDetection()
{
}


bool PhysicsCollision::CollisionDetection(AABB shapeOne, AABB shapeTwo, bool test_z_axis)
{
	/******************************************************
	*	Function Name:		CollisionDetection()
	*	Programmer:			Josh Archer
	*
	*	Determines if a collision has taken place between
	*	two AABB's, and returns the appropriate boolean
	*	value.
	*
	*	Uses the Separating Axis Theorem for detection.
	*
	******************************************************/

	//Test shape projections along each axes for overlap. Function can exit as soon as a disjoint (non intersecting) projection is found
	if(shapeOne.maxPoint.x < shapeTwo.minPoint.x || shapeTwo.maxPoint.x < shapeOne.minPoint.x)
	{
		//The shapes' projections along the x-axis are disjoint, so the shapes are not colliding
		return False;
	}
	else
	{
		if(shapeOne.maxPoint.y < shapeTwo.minPoint.y || shapeTwo.maxPoint.y < shapeOne.minPoint.y)
		{
			//The shapes' projection along the y-axis are disjoint, so the shapes are not colliding
			return False;
		}
		else
		{
			if(test_z_axis)
			{
				//Collision detection along z axis is desired, so test the third axis for intersection:
				if(shapeOne.maxPoint.z < shapeTwo.minPoint.z || shapeTwo.maxPoint.z < shapeOne.minPoint.z)
				{
					//The shapes' projection along the z-axis are disjoint, so the shapes are not colliding
					return False;
				}
				else
				{
					//The shapes' projection along all three axes are intersecting, so the shapes ARE colliding
					return True;
				}
			}
			else
			{
				//Collision detection along z axis is NOT desired, and the shapes' projections along both the x and y axes are intersecting, so the shapes ARE colliding
				return True;
			}
		}
	}	
}