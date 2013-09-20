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


void PhysicsCore::RayCast()
{
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
