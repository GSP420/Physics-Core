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


bool PhysicsCore::RayCast(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, bool test_z_axis, RayCastContact &contactOutput)
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

	float tx_min;
	float ty_min;
	float tz_min;

	float tx_max;
	float ty_max;
	float tz_max;

	float t1;
	float t2;

	//Convert the three vector components of the direction vector into normalized form
	float dxNormal = 1.0/directionVector.x;
	float dyNormal = 1.0/directionVector.y;
	float dzNormal = 1.0/directionVector.z;

	//Iterate through all collidables in the list and test for collisions along the ray
	while(i != collidables.end())
	{
		//Reset all of our t values for the next collidable in the list
		tx_min = -FLT_MAX;
		ty_min = -FLT_MAX;
		tz_min = -FLT_MAX;

		tx_max = FLT_MAX;
		ty_max = FLT_MAX;
		tz_max = FLT_MAX;

		//Reset the intersection detected toggle to TRUE
		bool intersectionHasOccured = TRUE;

		for(int j = 0;j < 3; j++)
		{
			if(j == 0)
			{
				//Perform x-axis calculations


				if(Abs(directionVector.x) < EPSILON)
				{
					//Ray is parallel to the x-axis slab
					if(startPoint.x < i->collidables.minPoint.x || startPoint.x > i->collidables.maxPoint.x)
					{
						//Ray is parallel to the x-axis slab and starting x-coordinate lies outside the current collidable. Therefore we aren't intersecting
						intersectionHasOccured = FALSE;
						break;
					}
				}
				else
				{
					//Ray is not parallel to the x-axis slab, so calculate t intersect values
					t1 = (i->collidables.minPoint.x - startPoint.x) * dxNormal;
					t2 = (i->collidables.maxPoint.x - startPoint.x) * dxNormal;

					//Swap values if necessary to ensure that t1 is our near intersect and t2 is our far intersect
					if(t1 > t2)
					{
						float temp = t1;
						t1 = t2;
						t2 = temp;
					}

					if(t1 > tx_min)
					{
						tx_min = t1;
					}
					if(t2 < tx_max)
					{
						tx_max = t2;
					}

					if(tx_min > tx_max || tx_max < 0)
					{
						//intersect interval is empty, so we aren't intersecting
						intersectionHasOccured = FALSE;
						break;
					}
				}
			}

			if(j == 1)
			{
				//Perform y-axis calculations


				if(Abs(directionVector.y) < EPSILON)
				{
					//Ray is parallel to the y-axis slab
					if(startPoint.y < i->collidables.minPoint.y || startPoint.y > i->collidables.maxPoint.y)
					{
						//Ray is parallel to the y-axis slab and starting y-coordinate lies outside the current collidable. Therefore we aren't intersecting
						intersectionHasOccured = FALSE;
						break;
					}
				}
				else
				{
					//Ray is not parallel to the y-axis slab, so calculate t intersect values
					t1 = (i->collidables.minPoint.y - startPoint.y) * dyNormal;
					t2 = (i->collidables.maxPoint.y - startPoint.y) * dyNormal;

					//Swap values if necessary to ensure that t1 is our near intersect and t2 is our far intersect
					if(t1 > t2)
					{
						float temp = t1;
						t1 = t2;
						t2 = temp;
					}

					if(t1 > ty_min)
					{
						ty_min = t1;
					}
					if(t2 < ty_max)
					{
						ty_max = t2;
					}

					if(ty_min > ty_max || ty_max < 0)
					{
						//intersect interval is empty, so we aren't intersecting
						intersectionHasOccured = FALSE;
						break;
					}
				}
			}

			if(j == 2 && test_z_axis == TRUE)
			{
				//Perform z-axis calculations


				if(Abs(directionVector.z) < EPSILON)
				{
					//Ray is parallel to the z-axis slab
					if(startPoint.z < i->collidables.minPoint.z || startPoint.z > i->collidables.maxPoint.z)
					{
						//Ray is parallel to the z-axis slab and starting z-coordinate lies outside the current collidable. Therefore we aren't intersecting
						intersectionHasOccured = FALSE;
						break;
					}
				}
				else
				{
					//Ray is not parallel to the z-axis slab, so calculate t intersect values
					t1 = (i->collidables.minPoint.z - startPoint.z) * dzNormal;
					t2 = (i->collidables.maxPoint.z - startPoint.z) * dzNormal;

					//Swap values if necessary to ensure that t1 is our near intersect and t2 is our far intersect
					if(t1 > t2)
					{
						float temp = t1;
						t1 = t2;
						t2 = temp;
					}

					if(t1 > tz_min)
					{
						tz_min = t1;
					}
					if(t2 < tz_max)
					{
						tz_max = t2;
					}

					if(tz_min > tz_max || tz_max < 0)
					{
						//intersect interval is empty, so we aren't intersecting
						intersectionHasOccured = FALSE;
						break;
					}
				}
			}
		}

		if(intersectionHasOccured == TRUE)
		{
			//We have intersected with one of the collidables, so move the appropriate values into the output structure and return TRUE
			contactOutput.t_min.x = tx_min;
			contactOutput.t_min.y = ty_min;
			contactOutput.t_min.z = tz_min;
			contactOutput.collidable_ID = i->collidables.ID;
			return TRUE;
		}
		
		if(i > maxTestLimit)
		{
			break;
		}
		else
		{
			i++;
		}
	}

	//If we get to this point in the function, we've either reached the limit for number of tests, or we've iterated through all collidables in the list and found no intersections
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
