#include "PhysicsCore.h"

const float PhysicsCore::GRAVITY = -9.8f;
const float PhysicsCore::FRICTION = 0.5f;

PhysicsCore::PhysicsCore(void)
{
}


PhysicsCore::~PhysicsCore(void)
{
}


void PhysicsCore::Accelerate(float delta_Time)
{
	/******************************************************
	*	Function Name		Accelerate()
	*	Programmer			Alexander Hunsiker
	*
	*	Determines the new velocity by adding the current
	*	velocity to current acceleration*delta time
	******************************************************/
	D3DXVECTOR3 max_Velocity = D3DXVECTOR3(5.0f, 5.0f, 5.0f);
	D3DXVECTOR3 max_Acceleration = D3DXVECTOR3(-GRAVITY, -GRAVITY, -GRAVITY);
	D3DXVECTOR3 cur_Acceleration;
	D3DXVECTOR3 cur_Velocity;
	D3DXVECTOR3 new_Velocity;
	
	cur_Acceleration = GetAcceleration();
	cur_Velocity = GetVelocity();
	new_Velocity += cur_Acceleration * delta_Time;
	new_Velocity.x *= float(FRICTION);

	if(new_Velocity > max_Velocity)
		new_Velocity = max_Velocity;
	if(new_Velocity < -max_Velocity)
		new_Velocity = -max_Velocity;
	if(cur_Acceleration > -max_Acceleration)
		cur_Acceleration = max_Acceleration;
	if(cur_Acceleration < max_Acceleration)
		cur_Acceleration = -max_Acceleration;

	SetVelocity(new_Velocity);
}


bool PhysicsCore::RayCast(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput)
{
	/******************************************************
	*	Function Name:		RayCast()
	*	Programmer:			Josh Archer
	*
	*	Determines if an AABB in the collidables list
	*	intersects the ray with origin startPoint and
	*	direction vector directionVector
	*
	*	Returns TRUE if a collision has occurred with one
	*	of the collidables,
	*	Returns FALSE if we have reached the test limit
	*	or if no collisions were detected
	*
	*	Also assigns hit information to contactOutput
	*	if a collision has occurred.
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

	
	const float EPSILON = 0.00001f;

	//Keep track of how many times we've iterated through the collidables list
	int numIterations = 0;

	//Convert the three vector components of the direction vector into normalized form
	float dxNormal = 1.0f/directionVector.x;
	float dyNormal = 1.0f/directionVector.y;
	float dzNormal = 1.0f/directionVector.z;

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
		bool intersectionHasOccured = true;

		for(int j = 0;j < 3; j++)
		{
			if(j == 0)
			{
				//Perform x-axis calculations


				if(abs(directionVector.x) < EPSILON)
				{
					//Ray is parallel to the x-axis slab
					if(startPoint.x < i->minPoint.x || startPoint.x > i->maxPoint.x)
					{
						//Ray is parallel to the x-axis slab and starting x-coordinate lies outside the current collidable. Therefore we aren't intersecting
						intersectionHasOccured = false;
						break;
					}
				}
				else
				{
					//Ray is not parallel to the x-axis slab, so calculate t intersect values
					t1 = (i->minPoint.x - startPoint.x) * dxNormal;
					t2 = (i->maxPoint.x - startPoint.x) * dxNormal;

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
						intersectionHasOccured = false;
						break;
					}
				}
			}

			if(j == 1)
			{
				//Perform y-axis calculations


				if(abs(directionVector.y) < EPSILON)
				{
					//Ray is parallel to the y-axis slab
					if(startPoint.y < i->minPoint.y || startPoint.y > i->maxPoint.y)
					{
						//Ray is parallel to the y-axis slab and starting y-coordinate lies outside the current collidable. Therefore we aren't intersecting
						intersectionHasOccured = false;
						break;
					}
				}
				else
				{
					//Ray is not parallel to the y-axis slab, so calculate t intersect values
					t1 = (i->minPoint.y - startPoint.y) * dyNormal;
					t2 = (i->maxPoint.y - startPoint.y) * dyNormal;

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
						intersectionHasOccured = false;
						break;
					}
				}
			}

			if(j == 2)
			{
				//Perform z-axis calculations


				if(abs(directionVector.z) < EPSILON)
				{
					//Ray is parallel to the z-axis slab
					if(startPoint.z < i->minPoint.z || startPoint.z > i->maxPoint.z)
					{
						//Ray is parallel to the z-axis slab and starting z-coordinate lies outside the current collidable. Therefore we aren't intersecting
						intersectionHasOccured = false;
						break;
					}
				}
				else
				{
					//Ray is not parallel to the z-axis slab, so calculate t intersect values
					t1 = (i->minPoint.z - startPoint.z) * dzNormal;
					t2 = (i->maxPoint.z - startPoint.z) * dzNormal;

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
						intersectionHasOccured = false;
						break;
					}
				}
			}
		}

		if(intersectionHasOccured == true)
		{
			//We have intersected with one of the collidables, so move the appropriate values into the output structure and return TRUE
			contactOutput.t_min.x = tx_min;
			contactOutput.t_min.y = ty_min;
			contactOutput.t_min.z = tz_min;
			contactOutput.collidable_ID = i->ID;
			return true;
		}
		
		if(numIterations > maxTestLimit)
		{
			break;
		}
		else
		{
			numIterations++;
		}
	}

	//If we get to this point in the function, we've either reached the limit for number of tests, or we've iterated through all collidables in the list and found no intersections
	return false;
}


bool PhysicsCore::RayCast(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput)
{
	/******************************************************
	*	Function Name:		RayCast()
	*	Programmer:			Josh Archer
	*
	*	Determines if an AABB in the collidables list
	*	intersects the ray with origin startPoint and
	*	direction vector directionVector
	*
	*	Returns TRUE if a collision has occurred with one
	*	of the collidables,
	*	Returns FALSE if we have reached the test limit
	*	or if no collisions were detected
	*
	*	Also assigns hit information to contactOutput
	*	if a collision has occurred.
	******************************************************/

	//Create and initialize an iterator for the collidables list.
	list<AABB>::iterator i = collidables.begin();

	float tx_min;
	float ty_min;

	float tx_max;
	float ty_max;

	float t1;
	float t2;

	
	const float EPSILON = 0.00001f;

	//Keep track of how many times we've iterated through the collidables list
	int numIterations = 0;

	//Convert the three vector components of the direction vector into normalized form
	float dxNormal = 1.0f/directionVector.x;
	float dyNormal = 1.0f/directionVector.y;

	//Iterate through all collidables in the list and test for collisions along the ray
	while(i != collidables.end())
	{
		//Reset all of our t values for the next collidable in the list
		tx_min = -FLT_MAX;
		ty_min = -FLT_MAX;

		tx_max = FLT_MAX;
		ty_max = FLT_MAX;

		//Reset the intersection detected toggle to TRUE
		bool intersectionHasOccured = true;

		for(int j = 0;j < 2; j++)
		{
			if(j == 0)
			{
				//Perform x-axis calculations


				if(abs(directionVector.x) < EPSILON)
				{
					//Ray is parallel to the x-axis slab
					if(startPoint.x < i->minPoint.x || startPoint.x > i->maxPoint.x)
					{
						//Ray is parallel to the x-axis slab and starting x-coordinate lies outside the current collidable. Therefore we aren't intersecting
						intersectionHasOccured = false;
						break;
					}
				}
				else
				{
					//Ray is not parallel to the x-axis slab, so calculate t intersect values
					t1 = (i->minPoint.x - startPoint.x) * dxNormal;
					t2 = (i->maxPoint.x - startPoint.x) * dxNormal;

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
						intersectionHasOccured = false;
						break;
					}
				}
			}

			if(j == 1)
			{
				//Perform y-axis calculations


				if(abs(directionVector.y) < EPSILON)
				{
					//Ray is parallel to the y-axis slab
					if(startPoint.y < i->minPoint.y || startPoint.y > i->maxPoint.y)
					{
						//Ray is parallel to the y-axis slab and starting y-coordinate lies outside the current collidable. Therefore we aren't intersecting
						intersectionHasOccured = false;
						break;
					}
				}
				else
				{
					//Ray is not parallel to the y-axis slab, so calculate t intersect values
					t1 = (i->minPoint.y - startPoint.y) * dyNormal;
					t2 = (i->maxPoint.y - startPoint.y) * dyNormal;

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
						intersectionHasOccured = false;
						break;
					}
				}
			}
		}

		if(intersectionHasOccured == true)
		{
			//We have intersected with one of the collidables, so move the appropriate values into the output structure and return TRUE
			contactOutput.t_min.x = tx_min;
			contactOutput.t_min.y = ty_min;
			contactOutput.collidable_ID = i->ID;
			return true;
		}
		
		if(numIterations > maxTestLimit)
		{
			break;
		}
		else
		{
			numIterations++;
		}
	}

	//If we get to this point in the function, we've either reached the limit for number of tests, or we've iterated through all collidables in the list and found no intersections
	return false;
}


D3DXVECTOR3 PhysicsCore::GetVelocity()
{
  return velocity;
}


void PhysicsCore::SetVelocity(D3DXVECTOR3 Vel)
{
  velocity = Vel;
}


D3DXVECTOR3 PhysicsCore::GetAcceleration()
{
  return acceleration;
}


void PhysicsCore::SetAcceleration(D3DXVECTOR3 Accel)
{
  acceleration = Accel;
}


void PhysicsCore::SetAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint)
{
	AABB aabb;

	aabb.minPoint = minPoint;
	aabb.maxPoint = maxPoint;
}


void PhysicsCore::SetAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, bool useCCD)
{
	AABB aabb;

	aabb.minPoint = minPoint;
	aabb.maxPoint = maxPoint;
	aabb.useContinuousDetection = useCCD;
}


void PhysicsCore::SetAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, string ID)
{
	AABB aabb;

	aabb.minPoint = minPoint;
	aabb.maxPoint = maxPoint;
	aabb.ID = ID;
}


void PhysicsCore::SetAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, bool useCCD, string ID)
{
	AABB aabb;

	aabb.minPoint = minPoint;
	aabb.maxPoint = maxPoint;
	aabb.useContinuousDetection = useCCD;
	aabb.ID = ID;
}