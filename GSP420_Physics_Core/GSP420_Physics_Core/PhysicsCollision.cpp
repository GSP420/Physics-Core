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


bool PhysicsCollision::ContinuousCollisionDetection(D3DXVECTOR3 Obj1_centerPoint_previous, D3DXVECTOR3 Obj1_centerPoint_current, D3DXVECTOR3 Obj1_extent, 
													D3DXVECTOR3 Obj2_centerPoint_previous, D3DXVECTOR3 Obj2_centerPoint_current, D3DXVECTOR3 Obj2_extent,
													float deltaTime, float &timeToImpact)
{
	/******************************************************
	*	Function Name:		ContinousCollisionDetection()
	*	Programmer:			Nathanael Blanchard
	*
	*
	******************************************************/
	
	
	/*
	dist2 = ObjectDistance(Obj1.nextmove, Obj2.nextmove)

	if((dist - dist2) < 0)
		adjust vel so object stops at collide point
	else
		do nothing?
	*/

	D3DXVECTOR3 previousDistance = ObjectDistance(Obj1_centerPoint_previous, Obj2_centerPoint_previous);
	D3DXVECTOR3 currentDistance = ObjectDistance(Obj1_centerPoint_current, Obj2_centerPoint_current);


	//Calculate min and max points of object 1 for previous and current positions
	D3DXVECTOR3 Obj1_minPoint_previous = Obj1_centerPoint_previous - Obj1_extent;
	D3DXVECTOR3 Obj1_maxPoint_previous = Obj1_centerPoint_previous + Obj1_extent;

	D3DXVECTOR3 Obj1_minPoint_current = Obj1_centerPoint_current - Obj1_extent;
	D3DXVECTOR3 Obj1_maxPoint_current = Obj1_centerPoint_current + Obj1_extent;

	//Calculate the change in min and max positions between previous and current time steps
	D3DXVECTOR3 Obj1_minPoint_movement = Obj1_minPoint_current - Obj1_minPoint_previous;
	D3DXVECTOR3 Obj1_maxPoint_movement = Obj1_maxPoint_current - Obj1_maxPoint_previous;


	//Calculate min and max points of object 2 for previous and current positions
	D3DXVECTOR3 Obj2_minPoint_previous = Obj2_centerPoint_previous - Obj2_extent;
	D3DXVECTOR3 Obj2_maxPoint_previous = Obj2_centerPoint_previous + Obj2_extent;

	D3DXVECTOR3 Obj2_minPoint_current = Obj2_centerPoint_current - Obj2_extent;
	D3DXVECTOR3 Obj2_maxPoint_current = Obj2_centerPoint_current + Obj2_extent;

	//Calculate the change in min and max point positions between previous and current time steps
	D3DXVECTOR3 Obj2_minPoint_movement = Obj2_minPoint_current - Obj2_minPoint_previous;
	D3DXVECTOR3 Obj2_maxPoint_movement = Obj2_maxPoint_current - Obj2_maxPoint_previous;


	//Calculate the change in distance between objects along each axis
	D3DXVECTOR3 delta_distance;
	delta_distance.x = currentDistance.x - previousDistance.x;
	delta_distance.y = currentDistance.y - previousDistance.y;
	delta_distance.z = currentDistance.z - previousDistance.z;

	if(currentDistance.x < previousDistance.x || currentDistance.y < previousDistance.y || currentDistance.z < previousDistance.z)
	{
		//The two objects have moved closer together

		D3DXVECTOR3 Obj1_movement = ObjectDistance(Obj1_centerPoint_previous, Obj1_centerPoint_current);	//How far object 1 moved during the last time interval
		D3DXVECTOR3 Obj2_movement = ObjectDistance(Obj2_centerPoint_previous, Obj2_centerPoint_current);	//How far object 2 moved during the last time interval

		if(abs(delta_distance.x) >= abs(delta_distance.y) && abs(delta_distance.x) >= abs(delta_distance.z))
		{
			//Greater change along x axis
			//Left & right faces are the fastest approaching faces
		}
		else
		{
			if(abs(delta_distance.y) >= abs(delta_distance.x) && abs(delta_distance.y) >= abs(delta_distance.z))
			{
				//Greater change along y axis
				//Top & bottom faces are the fastest approaching faces
			}
			else
			{
				//Greater change along z axis
				//Front & back faces are the fastest approaching faces
			}
		}
	}
	else
	{
		//The two objects are either at the same distance as the previous coordinate, or further apart, so no testing is needed
		return false;
	}
}


bool PhysicsCollision::CollisionDetection(vector<AABB*> &boxes, Octree* octree, bool test_z_axis)
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

	_octree->potentialBoxBoxCollision(bps, boxes, octree);
	for(unsigned int i = 0; i < bps.size(); i++) 
	{
		AABBPair bp = bps[i];
		
		AABB* shapeOne = bp.aabb1;
		AABB* shapeTwo = bp.aabb2;

		//Test shape projections along each axes for overlap. Function can exit as soon as a disjoint (non intersecting) projection is found
		if(shapeOne->maxPoint.x < shapeTwo->minPoint.x || shapeTwo->maxPoint.x < shapeOne->minPoint.x)
		{
			//The shapes' projections along the x-axis are disjoint, so the shapes are not colliding
			return false;
		}
		else
		{
			if(shapeOne->maxPoint.y < shapeTwo->minPoint.y || shapeTwo->maxPoint.y < shapeOne->minPoint.y)
			{
				//The shapes' projection along the y-axis are disjoint, so the shapes are not colliding
				return false;
			}
			else
			{
				if(test_z_axis)
				{
					//Collision detection along z axis is desired, so test the third axis for intersection:
					if(shapeOne->maxPoint.z < shapeTwo->minPoint.z || shapeTwo->maxPoint.z < shapeOne->minPoint.z)
					{
						//The shapes' projection along the z-axis are disjoint, so the shapes are not colliding
						return false;
					}
					else
					{
						//The shapes' projection along all three axes are intersecting, so the shapes ARE colliding
						return true;
					}
				}
				else
				{
					//Collision detection along z axis is NOT desired, and the shapes' projections along both the x and y axes are intersecting, so the shapes ARE colliding
					return false;
				}
			}
		}
	}//End for loop
	return true;
}