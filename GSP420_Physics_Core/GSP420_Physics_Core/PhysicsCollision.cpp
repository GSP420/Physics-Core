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


bool PhysicsCollision::CollisionDetection(shapeOne, shapeTwo)
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


	//Create a line between the minimum x and the maximum x of each shape
	//example:
	//lineA = (shapeOne.X_min, 0) to (shapeOne.X_max, 0)
	//lineB = (shapeTwo.X_min, 0) to (shapeTwo.X_max, 0)

	//Create a line between the minimum y and the maximum y of each shape
	//example:
	//lineC = (0, shapeOne.Y_min) to (0, shapeOne.Y_max)
	//lineD = (0, shapeTwo.Y_min) to (0, shapeTwo.Y_max)

	//Check for overlap (if overlap between lines on both axes is found, then the shapes are colliding)
}