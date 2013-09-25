#include "PhysicsCollision.h"


PhysicsCollision::PhysicsCollision(void)
{
}


PhysicsCollision::~PhysicsCollision(void)
{
}


D3DXVECTOR3 PhysicsCollision::ObjectDistance(D3DXVECTOR3 Obj1, D3DXVECTOR3 Obj2)
{
	/******************************************************
	*	Function Name:		ObjectDistance()
	*	Programmer:			Nathanael Blanchard
	*
	*	Finds distance between two objects
	******************************************************/
	D3DXVECTOR3 dist;
	dist = Obj2 - Obj1;
	return(dist);
}


void PhysicsCollision::ContinuousCollisionDetection(D3DXVECTOR3 Obj1, D3DXVECTOR3 Obj2)
{
	/******************************************************
	*	Function Name:		ContinousCollisionDetection()
	*	Programmer:			Nathanael Blanchard
	*
	*
	******************************************************/
	D3DXVECTOR3 dist = ObjectDistance(Obj1, Obj2);
	/*
	dist2 = ObjectDistance(Obj1.nextmove, Obj2.nextmove)

	if((dist - dist2) < 0)
		adjust vel so object stops at collide point
	else
		do nothing?
	*/
}


vector<AABBPair> PhysicsCollision::CollisionDetection(vector<AABB*> &boxes, Octree* octree, bool test_z_axis)
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

	vector<AABBPair> bps;
	vector<AABBPair> are_colliding;
	bool continueChecking;

	_octree->potentialBoxBoxCollision(bps, boxes, octree);
	for(unsigned int i = 0; i < bps.size(); i++) 
	{
		continueChecking = true;
		while(continueChecking)
		{
			AABBPair bp = bps[i];
		
			AABB* shapeOne = bp.aabb1;
			AABB* shapeTwo = bp.aabb2;

			//Test shape projections along each axes for overlap. Function can exit as soon as a disjoint (non intersecting) projection is found
			if(shapeOne->maxPoint.x < shapeTwo->minPoint.x || shapeTwo->maxPoint.x < shapeOne->minPoint.x)
			{
				//The shapes' projections along the x-axis are disjoint, so the shapes are not colliding
				continueChecking = false;
			}
			else
			{
				if(shapeOne->maxPoint.y < shapeTwo->minPoint.y || shapeTwo->maxPoint.y < shapeOne->minPoint.y)
				{
					//The shapes' projection along the y-axis are disjoint, so the shapes are not colliding
					continueChecking = false;
				}
				else
				{
					if(test_z_axis)
					{
						//Collision detection along z axis is desired, so test the third axis for intersection:
						if(shapeOne->maxPoint.z < shapeTwo->minPoint.z || shapeTwo->maxPoint.z < shapeOne->minPoint.z)
						{
							//The shapes' projection along the z-axis are disjoint, so the shapes are not colliding
							continueChecking = false;
						}
						else
						{
							//The shapes' projection along all three axes are intersecting, so the shapes ARE colliding
							are_colliding.push_back(bp);
							continueChecking = false;
						}
					}
					else
					{
						//Collision detection along z axis is NOT desired, and the shapes' projections along both the x and y axes are intersecting, so the shapes ARE colliding
						are_colliding.push_back(bp);
						continueChecking = false;
					}
				}
			}
		}//End while loop
	}//End for loop
	return are_colliding;
}