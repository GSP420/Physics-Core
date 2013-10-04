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

bool PhysicsCollision::sweptCCD(D3DXVECTOR2 boxA_centerPoint_previous, D3DXVECTOR2 boxA_centerPoint_current, D3DXVECTOR2 boxA_extent,
							D3DXVECTOR2 boxB_centerPoint_previous, D3DXVECTOR2 boxB_centerPoint_current, D3DXVECTOR2 boxB_extent,
							float &timeOfImpact)
{
	//Check if box A and box B were already overlapping in their previous positions:
	D3DXVECTOR2 AB_separation = boxA_centerPoint_previous - boxB_centerPoint_previous;

	if(		(fabs(AB_separation.x) <= fabs(boxA_extent.x + boxB_extent.x))
		&&	(fabs(AB_separation.y) <= fabs(boxA_extent.y + boxB_extent.y))		)
	{
		//The boxes were already overlapping at their previous positions
		timeOfImpact = 0;
		return true;
	}

	//We know they weren't overlapping at thier previous positions, so let's set up for detecting potential times of impact 
	
	
	D3DXVECTOR2 displacementA = boxA_centerPoint_current - boxA_centerPoint_previous;		//Displacement of box A between previous and current positions
	D3DXVECTOR2 displacementB = boxB_centerPoint_current - boxB_centerPoint_previous;		//Displacement of box B between previous and current positions

	D3DXVECTOR2 relativeVelocity = displacementB - displacementA;		//Relative velocity between box A and box B

	D3DXVECTOR2 aMin = boxA_centerPoint_previous - boxA_extent;		//Min point of box A
	D3DXVECTOR2 aMax = boxA_centerPoint_previous + boxA_extent;		//Max point of box A

	D3DXVECTOR2 bMin = boxB_centerPoint_previous - boxB_extent;		//Min point of box B
	D3DXVECTOR2 bMax = boxB_centerPoint_previous + boxB_extent;		//Max point of box B

	//First time of overlap along x and y axes
	D3DXVECTOR2 t0;
	t0.x, t0.y = 0, 0;
	//Last time of overlap along x and y axes
	D3DXVECTOR2 t1;
	t1.x, t1.y = 1, 1;

	for(int i = 0; i < 2; i++)
	{
		if(i = 0)
		{
			//Test x axis

			//Test for earliest time of impact:
			if(		(aMax.x < bMin.x)	&&	(relativeVelocity.x < 0)	)
			{
				t0.x = (aMax.x - bMin.x) / relativeVelocity.x;		//Potential time of impact, normalized
			}
			else if(		(bMax.x < aMin.x)	&&	(relativeVelocity.x > 0)	)
			{
				t0.x = (aMin.x - bMax.x) / relativeVelocity.x;		//Potential time of impact, normalized
			}

			//Test for last time of impact:
			if(		(bMax.x > aMin.x)	&&	(relativeVelocity.x < 0)	)
			{
				t1.x = (aMin.x - bMax.x) / relativeVelocity.x;		//Potential time of impact, normalized
			}
			else if(		(aMax.x > bMin.x)	&&	(relativeVelocity.x > 0)		)
			{
				t1.x = (aMax.x - bMin.x) / relativeVelocity.x;
			}
		}
		else
		{
			//Test y axis

			//Test for earliest time of impact:
			if(		(aMax.y < bMin.y)	&&	(relativeVelocity.y < 0)	)
			{
				t0.y = (aMax.y - bMin.y) / relativeVelocity.y;		//Potential time of impact, normalized
			}
			else if(		(bMax.y < aMin.y)	&&	(relativeVelocity.y > 0)	)
			{
				t0.y = (aMin.y - bMax.y) / relativeVelocity.y;		//Potential time of impact, normalized
			}

			//Test for last time of impact:
			if(		(bMax.y > aMin.y)	&&	(relativeVelocity.y < 0)	)
			{
				t1.y = (aMin.y - bMax.y) / relativeVelocity.y;		//Potential time of impact, normalized
			}
			else if(		(aMax.y > bMin.y)	&&	(relativeVelocity.y > 0)		)
			{
				t1.y = (aMax.y - bMin.y) / relativeVelocity.y;
			}
		}
	}

	float tMin, tMax;
	//Potential time of first impact
	tMin = max(t0.x, t0.y);
	
	//Potential time of last impact
	tMax = min(t1.x, t1.y);

	//An impact has only occurred if the time of first impact is less than or equal to the time of last impact
	if(tMin <= tMax)
	{
		timeOfImpact = tMin;
		return true;
	}
	else
	{
		timeOfImpact = -FLT_MAX;
		return false;
	}
}

bool PhysicsCollision::CCD(D3DXVECTOR2 Obj1_centerPoint_current, D3DXVECTOR2 Obj1_centerPoint_future, D3DXVECTOR2 Obj1_extent, D3DXVECTOR2 Obj1_velocity,
							D3DXVECTOR2 Obj2_centerPoint_current, D3DXVECTOR2 Obj2_centerPoint_future, D3DXVECTOR2 Obj2_extent,	D3DXVECTOR2 Obj2_velocity,
							float deltaTime, float &timeOfImpact)
{
	struct CCD_line
	{
		//Start and ending points of the line
		D3DXVECTOR2 currentPoint;
		D3DXVECTOR2 futurePoint;

		//point-slope formula:		m = (y2 - y1) / (x2 - x1)
		float slope;

		float calculateSlope()
		{
			return (futurePoint.y - currentPoint.y) / (futurePoint.x - currentPoint.x);
		}

		int lineID;

		/*********************************
		*					Object 1
		*	 ___________________________
		*	|___________________________|
		*	|LINE #		|	ID #		|
		*	|___________|_______________|
		*	|bottomLeft	|	0			|
		*	|bottomRight|	1			|
		*	|topRight	|	2			|
		*	|topLeft	|	3			|
		*	|___________|_______________|
		*********************************/

		/*********************************
		*					Object 2
		*	 ___________________________
		*	|___________________________|
		*	|LINE #		|	ID #		|
		*	|___________|_______________|
		*	|bottomLeft	|	4			|
		*	|bottomRight|	5			|
		*	|topRight	|	6			|
		*	|topLeft	|	7			|
		*	|___________|_______________|
		*********************************/
	};

	struct CCD_intersection
	{
		D3DXVECTOR2 intersectionPoint;

		int lineOneID;
		int lineTwoID;

		/*********************************
		*					Object 1
		*	 ___________________________
		*	|___________________________|
		*	|LINE #		|	ID #		|
		*	|___________|_______________|
		*	|bottomLeft	|	0			|
		*	|bottomRight|	1			|
		*	|topRight	|	2			|
		*	|topLeft	|	3			|
		*	|___________|_______________|
		*********************************/

		/*********************************
		*					Object 2
		*	 ___________________________
		*	|___________________________|
		*	|LINE #		|	ID #		|
		*	|___________|_______________|
		*	|bottomLeft	|	4			|
		*	|bottomRight|	5			|
		*	|topRight	|	6			|
		*	|topLeft	|	7			|
		*	|___________|_______________|
		*********************************/
	};

	bool willCollide = false;

	//Object 1 current minPoint and maxPoint:
	D3DXVECTOR2 Obj1_minPoint_current = Obj1_centerPoint_current - Obj1_extent;
	D3DXVECTOR2 Obj1_maxPoint_current = Obj1_centerPoint_current + Obj1_extent;

	//Object 1 future minPoint and maxPoint:
	D3DXVECTOR2 Obj1_minPoint_future = Obj1_centerPoint_future - Obj1_extent;
	D3DXVECTOR2 Obj1_maxPoint_future = Obj1_centerPoint_future + Obj1_extent;

	//Object 2 current minPoint and maxPoint:
	D3DXVECTOR2 Obj2_minPoint_current = Obj2_centerPoint_current - Obj2_extent;
	D3DXVECTOR2 Obj2_maxPoint_current = Obj2_centerPoint_current + Obj2_extent;

	//Object 2 future minPoint and maxPoint:
	D3DXVECTOR2 Obj2_minPoint_future = Obj2_centerPoint_future - Obj2_extent;
	D3DXVECTOR2 Obj2_maxPoint_future = Obj2_centerPoint_future + Obj2_extent;

	/*
		(minPoint.x, maxPoint.y)	*-------*	(maxPoint.x, maxPoint.y)
									|		|
									|		|
		(minPoint.x, minPoint.y)	*-------*	(maxPoint.x, minPoint.y)
	*/

	/*********************************
	******		OBJECT 2		******
	*********************************/
	//Object 1 corners current position:
	D3DXVECTOR2 Obj1_bottomLeft_current, Obj1_bottomRight_current, Obj1_topRight_current, Obj1_topLeft_current;

	Obj1_bottomLeft_current.x, Obj1_bottomLeft_current.y			=	Obj1_minPoint_current.x, Obj1_minPoint_current.y;
	Obj1_bottomRight_current.x, Obj1_bottomRight_current.y			=	Obj1_maxPoint_current.x, Obj1_minPoint_current.y;
	Obj1_topRight_current.x, Obj1_topRight_current.y				=	Obj1_maxPoint_current.x, Obj1_maxPoint_current.y;
	Obj1_topLeft_current.x, Obj1_topLeft_current.y					=	Obj1_minPoint_current.x, Obj1_maxPoint_current.y;


	//Object 1 corners future position:
	D3DXVECTOR2 Obj1_bottomLeft_future, Obj1_bottomRight_future, Obj1_topRight_future, Obj1_topLeft_future;

	Obj1_bottomLeft_future.x, Obj1_bottomLeft_future.y				=	Obj1_minPoint_future.x, Obj1_minPoint_future.y;
	Obj1_bottomRight_future.x, Obj1_bottomRight_future.y			=	Obj1_maxPoint_future.x, Obj1_minPoint_future.y;
	Obj1_topRight_future.x, Obj1_topRight_future.y					=	Obj1_maxPoint_future.x, Obj1_maxPoint_future.y;
	Obj1_topLeft_future.x, Obj1_topLeft_future.y					=	Obj1_minPoint_future.x, Obj1_maxPoint_future.y;

	//Generate our lines for object 1's path
	CCD_line Obj1_bottomLeft, Obj1_bottomRight, Obj1_topRight, Obj1_topLeft;

	Obj1_bottomLeft.currentPoint, Obj1_bottomLeft.futurePoint		=	Obj1_bottomLeft_current, Obj1_bottomLeft_future;
	Obj1_bottomRight.currentPoint, Obj1_bottomRight.futurePoint		=	Obj1_bottomRight_current, Obj1_bottomRight_future;
	Obj1_topRight.currentPoint, Obj1_topRight.futurePoint			=	Obj1_topRight_current, Obj1_topRight_future;
	Obj1_topLeft.currentPoint, Obj1_topLeft.futurePoint				=	Obj1_topLeft_current, Obj1_topLeft_future;

	Obj1_bottomLeft.lineID	= 0;
	Obj1_bottomRight.lineID = 1;
	Obj1_topRight.lineID	= 2;
	Obj1_topLeft.lineID		= 3;

	//Calculate the slopes of the lines
	Obj1_bottomLeft.slope = Obj1_bottomLeft.calculateSlope();
	Obj1_bottomRight.slope = Obj1_bottomRight.calculateSlope();
	Obj1_topRight.slope = Obj1_topRight.calculateSlope();
	Obj1_topLeft.slope = Obj1_topLeft.calculateSlope();

	/*********************************
	******		OBJECT 2		******
	*********************************/
	//Object 2 corners current position:
	D3DXVECTOR2 Obj2_bottomLeft_current, Obj2_bottomRight_current, Obj2_topRight_current, Obj2_topLeft_current;

	Obj2_bottomLeft_current.x, Obj2_bottomLeft_current.y			=	Obj2_minPoint_current.x, Obj2_minPoint_current.y;
	Obj2_bottomRight_current.x, Obj2_bottomRight_current.y			=	Obj2_maxPoint_current.x, Obj2_minPoint_current.y;
	Obj2_topRight_current.x, Obj2_topRight_current.y				=	Obj2_maxPoint_current.x, Obj2_maxPoint_current.y;
	Obj2_topLeft_current.x, Obj2_topLeft_current.y					=	Obj2_minPoint_current.x, Obj2_maxPoint_current.y;


	//Object 2 corners future position:
	D3DXVECTOR2 Obj2_bottomLeft_future, Obj2_bottomRight_future, Obj2_topRight_future, Obj2_topLeft_future;

	Obj2_bottomLeft_future.x, Obj2_bottomLeft_future.y				=	Obj2_minPoint_future.x, Obj2_minPoint_future.y;
	Obj2_bottomRight_future.x, Obj2_bottomRight_future.y			=	Obj2_maxPoint_future.x, Obj2_minPoint_future.y;
	Obj2_topRight_future.x, Obj2_topRight_future.y					=	Obj2_maxPoint_future.x, Obj2_maxPoint_future.y;
	Obj2_topLeft_future.x, Obj2_topLeft_future.y					=	Obj2_minPoint_future.x, Obj2_maxPoint_future.y;

	//Generate our lines for object 2's path
	CCD_line Obj2_bottomLeft, Obj2_bottomRight, Obj2_topRight, Obj2_topLeft;

	Obj2_bottomLeft.currentPoint, Obj2_bottomLeft.futurePoint		=	Obj2_bottomLeft_current, Obj2_bottomLeft_future;
	Obj2_bottomRight.currentPoint, Obj2_bottomRight.futurePoint		=	Obj2_bottomRight_current, Obj2_bottomRight_future;
	Obj2_topRight.currentPoint, Obj2_topRight.futurePoint			=	Obj2_topRight_current, Obj2_topRight_future;
	Obj2_topLeft.currentPoint, Obj2_topLeft.futurePoint				=	Obj2_topLeft_current, Obj2_topLeft_future;

	Obj2_bottomLeft.lineID	= 4;
	Obj2_bottomRight.lineID = 5;
	Obj2_topRight.lineID	= 6;
	Obj2_topLeft.lineID		= 7;

	//Calculate the slopes of the lines
	Obj1_bottomLeft.slope = Obj1_bottomLeft.calculateSlope();
	Obj1_bottomRight.slope = Obj1_bottomRight.calculateSlope();
	Obj1_topRight.slope = Obj1_topRight.calculateSlope();
	Obj1_topLeft.slope = Obj1_topLeft.calculateSlope();



	//Now group all of the lines from object 1 into a list
	list<CCD_line> Obj1_lines;

	Obj1_lines.push_back(Obj1_bottomLeft);
	Obj1_lines.push_back(Obj1_bottomRight);
	Obj1_lines.push_back(Obj1_topRight);
	Obj1_lines.push_back(Obj1_topLeft);

	list<CCD_line>::iterator i = Obj1_lines.begin();


	//And group all of the lines from object 2 into a separate list
	list<CCD_line> Obj2_lines;

	Obj1_lines.push_back(Obj1_bottomLeft);
	Obj1_lines.push_back(Obj1_bottomRight);
	Obj1_lines.push_back(Obj1_topRight);
	Obj1_lines.push_back(Obj1_topLeft);

	list<CCD_line>::iterator j = Obj2_lines.begin();



	//list to hold the resulting intersection point information
	list<CCD_intersection> intersections;

	//Loop through the lines in object 1
	while(i != Obj1_lines.end())
	{
		//Loop through the lines in object 2 for each line in object 1
		while(j != Obj2_lines.end())
		{
			if(i->slope == j->slope)
			{
				//Lines are parallel, do nothing
			}
			else
			{
				//slope intercept form equation of a line:		y = mx + b

				//line one intercept:	b = y - mx
				float b1 = i->currentPoint.y - (i->slope * i->currentPoint.x);

				//line two intercept:	b = y - mx
				float b2 = j->currentPoint.y - (j->slope * j->currentPoint.x);
				
				//x intercept coordinate formula:	x = b2-b1 / m1+m2
				float intercept_x = (b2 - b1) / (i->slope + j->slope);

				//y intercept coordinate formula:	y = mx + b
				float intercept_y = (i->slope * i->currentPoint.x) + b1;

				//point of intersection:	(intercept_x, intercept_y)


				if((intercept_x < i->currentPoint.x && intercept_y < i->currentPoint.y) || (intercept_x > i->futurePoint.x && intercept_y > i->futurePoint.y))
				{
					//intercept point is outside the end points of the line segment, so this intersection isn't important
				}
				else
				{
					//We found an intersection point within the end points of one of our line segments, so there will be a collision 
					willCollide = true;

					//Temp intersection object to hold our calculated information
					CCD_intersection thisIntersection;

					//Store the intersection point coordinates
					thisIntersection.intersectionPoint.x, thisIntersection.intersectionPoint.y = intercept_x, intercept_y;

					//Transfer each line's ID to the intersection object
					thisIntersection.lineOneID = i->lineID;
					thisIntersection.lineTwoID = j->lineID;

					//Add the intersection object to the intersections list
					intersections.push_back(thisIntersection);
				}
			}
			j++;	//increment Object 2 line list
		}


		j = Obj2_lines.begin();	//Reset the second line list to the beginning
		i++;	//increment Object 1 line list
	}


	//Now determine which intersection point yields the intersection with the shortest time
	list<CCD_intersection>::iterator it = intersections.begin();

	//Set intersection time to the highest possible value
	float intersectionTime_x = FLT_MAX;
	float intersectionTime_y = FLT_MAX;
	
	float Obj1_x_speed = Obj1_velocity.x / deltaTime;
	float Obj1_y_speed = Obj1_velocity.y / deltaTime;

	float Obj2_x_speed = Obj2_velocity.x / deltaTime;
	float Obj2_y_speed = Obj2_velocity.y / deltaTime;

	float x_distance, y_distance;
	
	if(!intersections.empty())
	{
		while(it != intersections.end())
		{
			if(it->lineOneID == 0)
			{

				//bottomLeft line
				if(it->lineTwoID == 4)
				{
					//bottomLeft to bottomLeft intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_bottomLeft.currentPoint.x), abs(it->intersectionPoint.x - Obj2_bottomLeft.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_bottomLeft.currentPoint.y), abs(it->intersectionPoint.y - Obj2_bottomLeft.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 5)
				{
					//bottomLeft to bottomRight intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_bottomLeft.currentPoint.x), abs(it->intersectionPoint.x - Obj2_bottomRight.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_bottomLeft.currentPoint.y), abs(it->intersectionPoint.y - Obj2_bottomRight.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 6)
				{
					//bottomLeft to topRight intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_bottomLeft.currentPoint.x), abs(it->intersectionPoint.x - Obj2_topRight.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_bottomLeft.currentPoint.y), abs(it->intersectionPoint.y - Obj2_topRight.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 7)
				{
					//bottomLeft to topLeft intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_bottomLeft.currentPoint.x), abs(it->intersectionPoint.x - Obj2_topLeft.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_bottomLeft.currentPoint.y), abs(it->intersectionPoint.y - Obj2_topLeft.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}


			}
			else if(it->lineOneID == 1)
			{

				//bottomRight line
				if(it->lineTwoID == 4)
				{
					//bottomRight to bottomLeft intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_bottomRight.currentPoint.x), abs(it->intersectionPoint.x - Obj2_bottomLeft.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_bottomRight.currentPoint.y), abs(it->intersectionPoint.y - Obj2_bottomLeft.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 5)
				{
					//bottomRight to bottomRight intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_bottomRight.currentPoint.x), abs(it->intersectionPoint.x - Obj2_bottomRight.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_bottomRight.currentPoint.y), abs(it->intersectionPoint.y - Obj2_bottomRight.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 6)
				{
					//bottomRight to topRight intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_bottomRight.currentPoint.x), abs(it->intersectionPoint.x - Obj2_topRight.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_bottomRight.currentPoint.y), abs(it->intersectionPoint.y - Obj2_topRight.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 7)
				{
					//bottomRight to topLeft intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_bottomRight.currentPoint.x), abs(it->intersectionPoint.x - Obj2_topLeft.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_bottomRight.currentPoint.y), abs(it->intersectionPoint.y - Obj2_topLeft.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}

			}
			else if(it->lineOneID == 2)
			{

				//topRight line
				if(it->lineTwoID == 4)
				{
					//topRight to bottomLeft intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_topRight.currentPoint.x), abs(it->intersectionPoint.x - Obj2_bottomLeft.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_topRight.currentPoint.y), abs(it->intersectionPoint.y - Obj2_bottomLeft.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 5)
				{
					//topRight to bottomRight intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_topRight.currentPoint.x), abs(it->intersectionPoint.x - Obj2_bottomRight.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_topRight.currentPoint.y), abs(it->intersectionPoint.y - Obj2_bottomRight.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 6)
				{
					//topRight to topRight intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_topRight.currentPoint.x), abs(it->intersectionPoint.x - Obj2_topRight.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_topRight.currentPoint.y), abs(it->intersectionPoint.y - Obj2_topRight.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 7)
				{
					//topRight to topLeft intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_topRight.currentPoint.x), abs(it->intersectionPoint.x - Obj2_topLeft.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_topRight.currentPoint.y), abs(it->intersectionPoint.y - Obj2_topLeft.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}

			}
			else if(it->lineOneID == 3)
			{

				//topLeft line
				if(it->lineTwoID == 4)
				{
					//topLeft to bottomLeft intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_topLeft.currentPoint.x), abs(it->intersectionPoint.x - Obj2_bottomLeft.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_topLeft.currentPoint.y), abs(it->intersectionPoint.y - Obj2_bottomLeft.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 5)
				{
					//topLeft to bottomRight intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_topLeft.currentPoint.x), abs(it->intersectionPoint.x - Obj2_bottomRight.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_topLeft.currentPoint.y), abs(it->intersectionPoint.y - Obj2_bottomRight.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 6)
				{
					//topLeft to topRight intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_topLeft.currentPoint.x), abs(it->intersectionPoint.x - Obj2_topRight.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_topLeft.currentPoint.y), abs(it->intersectionPoint.y - Obj2_topRight.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}
				else if(it->lineTwoID == 7)
				{
					//topLeft to topLeft intersection
					x_distance = min(abs(it->intersectionPoint.x - Obj1_topLeft.currentPoint.x), abs(it->intersectionPoint.x - Obj2_topLeft.currentPoint.x));
					y_distance = min(abs(it->intersectionPoint.y - Obj1_topLeft.currentPoint.y), abs(it->intersectionPoint.y - Obj2_topLeft.currentPoint.y));

					float x_traversal = x_distance / Obj1_x_speed;
					float y_traversal = y_distance / Obj1_y_speed;

					if(intersectionTime_x > x_traversal)
					{
						//New fastest intersection time
						intersectionTime_x = x_traversal;
					}
					if(intersectionTime_y > y_traversal)
					{
						//New fastest intersection time
						intersectionTime_y = y_traversal;
					}
				}

			}

			it++;
		}
	}

	//Assign shortest intersection time to timeOfImpact
	timeOfImpact = min(intersectionTime_x, intersectionTime_y);

	if(willCollide)
	{
		return true;
	}
	else
	{
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