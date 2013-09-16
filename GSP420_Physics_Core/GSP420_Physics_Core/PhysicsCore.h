#pragma once
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

	void CreateAABB();
	void Accelerate();
	void SpatialPartitioning();
	void RayCast();
	void CollisionMaskLayers();
	void GetVelocity();
	void SetVelocity();
	void GetAcceleration();
	void SetAcceleration();
};

