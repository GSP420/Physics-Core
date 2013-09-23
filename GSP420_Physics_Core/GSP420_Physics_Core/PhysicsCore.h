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
#include <functional>
#include <algorithm>

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
			maxPoint(-INFINITY, -INFINITY, -INFINITY){}

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

	D3DXVECTOR3 minPoint;
	D3DXVECTOR3 maxPoint;

	string ID;
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
	double gravity;
	double friction;
	double max_Velocity;
	double max_Acceleration;
	double velocity;
	double acceleration;


	PhysicsCore(void);
	~PhysicsCore(void);

	void Accelerate();
	void SpatialPartitioning();
	bool RayCast(D3DXVECTOR3 startPoint, D3DXVECTOR3 directionVector, list<AABB> collidables, int maxTestLimit, bool test_z_axis, RayCastContact &contactOutput);
	void CollisionMaskLayers();
	double GetVelocity();
	void SetVelocity(double vel);
	double GetAcceleration();
	void SetAcceleration(double accel);
};

