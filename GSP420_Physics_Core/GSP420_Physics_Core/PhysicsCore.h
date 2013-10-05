/************************************************************
*File: PhysicsCore.h
*Programmer: GSP 420 Group G: Physics
*Purpose: contains data that will be the foundation for the 
*rest of the Physics core for the game engine
*Date:9/17/13
************************************************************/

#pragma once

#include <d3d9.h>
#include <d3dx9.h>
#include <sstream>
#include <list>
#include <string>
#include <cmath>
#include "Octree.h"

using namespace std;

/*************************************************************
*struct: AABB 
*Programmer: John Berg
*Purpose: to create an invisible box around obejcts to detect
*collisions
*************************************************************/
const float INFINITY = FLT_MAX; //variable to make an infinitely small AABB

struct AABB
{
	//initialize to an infinitely small AABB
	AABB():	minPoint(INFINITY, INFINITY, INFINITY),
			maxPoint(-INFINITY, -INFINITY, -INFINITY){useContinuousDetection = false;}

	//function that defines the center of the AABB
	D3DXVECTOR3 center()const
	{
		return (minPoint + maxPoint) / 2.0f;
	}

	//function to define the extent(outer bounds) of the AABB
	D3DXVECTOR3 extent()const
	{
		return (maxPoint - minPoint) / 2.0f;
	}

	//function that transforms the two vector3's into matrices and back
	void transform(const D3DXMATRIX& Mat, AABB& out)
	{
		D3DXVECTOR3 midPoint = center();
		D3DXVECTOR3 outerPoint = extent();

		//tranform center to matrix Mat
		D3DXVec3TransformCoord(&midPoint, &midPoint, &Mat);

		//now transform extent into another matrix
		D3DXMATRIX absMat;
		D3DXMatrixIdentity(&absMat);
			
		//set each position of absMat to the absolute value of the corresponding
		//position in Mat because extent can't have negative values
		for(int i = 0; i < 2; i++)
		{
			for(int j = 0; j < 2; j++)
			{
				absMat(i, j) = fabsf(Mat(i, j));
			}
		}
		D3DXVec3TransformNormal(&outerPoint, &outerPoint, &absMat);

		//convert back to two vectors
		out.minPoint = midPoint - outerPoint;
		out.maxPoint = midPoint + outerPoint;
	}

	void moveTo(D3DXVECTOR3 newMinPoint, D3DXVECTOR3 newMaxPoint)
	{
		centerPointPrevious = this->center();
		this->minPoint = newMinPoint;
		this->maxPoint = newMaxPoint;
	}

	D3DXVECTOR3 minPoint;
	D3DXVECTOR3 maxPoint;

	D3DXVECTOR3 centerPointPrevious;

	string ID;

	bool useContinuousDetection;
};

/*************************************************************
*struct: collisionInfo 
*Programmer: Josh Archer
*Purpose: to contain contact information about a collision
*between two AABB's
*************************************************************/
struct collisionInfo
{
	collisionInfo(AABB boxA, AABB boxB, D3DXVECTOR3 pointOfImpact, float TOI)
	{
		//Get the ID's of our colliding boxes
		boxA_ID = boxA.ID;
		boxB_ID = boxB.ID;

		//Get the time and points of impact
		impactPoint = pointOfImpact;
		timeOfImpact = TOI;

		//Calculate movement vectors for each box
		boxA_movementVector = boxA.center() - boxA.centerPointPrevious;
		boxB_movementVector = boxB.center() - boxB.centerPointPrevious;
	}

	collisionInfo(AABB boxA, AABB boxB, D3DXVECTOR3 pointOfImpact)
	{
		//overloaded version of constructor without a time of impact
		collisionInfo(boxA, boxB, pointOfImpact, -FLT_MAX);
	}

	D3DXVECTOR3 impactPoint;
	float timeOfImpact;

	string boxA_ID;
	string boxB_ID;

	D3DXVECTOR3 boxA_movementVector;
	D3DXVECTOR3 boxB_movementVector;
};

/*************************************************************
*struct: collisions
*Programmer: Josh Archer
*Purpose: Contains a list of detected collisions and functions
*which allow the user to move through each collisionInfo
*member of the list, retrieving information about each
*collision
*************************************************************/
struct collisions
{
	list<collisionInfo> collisionList;
	collisionInfo currentCollision;

	//Returns the length of the collision list
	int length()
	{
		if(!collisionList.empty())
		{
			return collisionList.size();
		}
		else
		{
			return 0;
		}
	}

	//Gets the first member in the list, assigns the values of that member of the list to the currentCollision object, and pops that member off of the list
	void getNext()
	{
		//Create an iterator and assign it to the first member of the list
		list<collisionInfo>::iterator it = collisionList.begin();

		//Transfer all the values to the corresponding values in the struct
		currentCollision.impactPoint = it->impactPoint;
		currentCollision.timeOfImpact = it->timeOfImpact;
		currentCollision.boxA_ID = it->boxA_ID;
		currentCollision.boxB_ID = it->boxB_ID;
		currentCollision.boxA_movementVector = it->boxA_movementVector;
		currentCollision.boxB_movementVector = it->boxB_movementVector;

		//Now pop the first member off of the list
		collisionList.pop_front();
	}

	//Adds the current collision to the end of the list
	void addToList()
	{
		collisionList.push_back(currentCollision);
	}

};

/*************************************************************
*struct: RayCastContact 
*Programmer: Josh Archer
*Purpose: to contain contact information about a collision
*between a ray and an AABB
*************************************************************/
struct RayCastContact
{
	D3DXVECTOR3 t_min;
	D3DXVECTOR3 t_max;

	string collidable_ID;
};


class PhysicsCore
{
public:
	static const float GRAVITY;
	static const float FRICTION;	//friction coefficient that may need changed after testing	

	D3DXVECTOR3 velocity;
	D3DXVECTOR3 acceleration;
	Octree* _octree;
	collisionInfo* _collisionInfo;
	collisions* _collisions;

	PhysicsCore(void);
	~PhysicsCore(void);

	void Accelerate(float delta_Time);
	bool RayCast(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput);
	bool RayCast(D3DXVECTOR2 startPoint, D3DXVECTOR2 directionVector, list<AABB> collidables, int maxTestLimit, RayCastContact &contactOutput);
	void CollisionMaskLayers();
	D3DXVECTOR3 GetVelocity();
	void SetVelocity(D3DXVECTOR3 vel);
	D3DXVECTOR3 GetAcceleration();
	void SetAcceleration(D3DXVECTOR3 accel);
	void SetAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint);
	void SetAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, bool useCCD);
	void SetAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, string ID);
	void SetAABB(D3DXVECTOR3 minPoint, D3DXVECTOR3 maxPoint, bool useCCD, string ID);
};

	

