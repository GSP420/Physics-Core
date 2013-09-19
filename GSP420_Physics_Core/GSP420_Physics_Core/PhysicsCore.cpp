#include "PhysicsCore.h"


PhysicsCore::PhysicsCore(void)
{
}


PhysicsCore::~PhysicsCore(void)
{
}


void PhysicsCore::CreateAABB()
{
}


void PhysicsCore::Accelerate()
{
}


void PhysicsCore::SpatialPartitioning()
{
}


void PhysicsCore::RayCast()
{
}


void PhysicsCore::CollisionMaskLayers()
{
}


void PhysicsCore::GetVelocity()
{
  return Velocity;
}


void PhysicsCore::SetVelocity(double Vel)
{
  Velocity = Vel;
}


void PhysicsCore::GetAcceleration()
{
  return Acceleration;
}


void PhysicsCore::SetAcceleration(double Accel)
{
  Acceleration = Accel;
}
