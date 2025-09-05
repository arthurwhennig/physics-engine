//
//  RigidBody.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#include <iostream>
#include <cmath>

#include "Vector2D.hpp"
#include "RigidBody.hpp"

// constructors
RigidBody::RigidBody()
    : position(0.0f, 0.0f), rotation(0.0f), velocity(0.0f, 0.0f), acceleration(0.0f, 0.0f),
      angularVelocity(0.0f), angularAcceleration(0.0f), mass(1.0f), inverseMass(1.0f),
      momentOfInertia(1.0f), inverseInertia(1.0f), restitution(0.5f), friction(0.3f),
      drag(0.01f), forceAccumulator(0.0f, 0.0f), torqueAccumulator(0.0f),
      bodyType(BodyType::DYNAMIC), awake(true), radius(1.0f)
{
}

RigidBody::RigidBody(const Vector2D &pos, const float mass, const float radius)
    : position(pos), rotation(0.0f), velocity(0.0f, 0.0f), acceleration(0.0f, 0.0f),
      angularVelocity(0.0f), angularAcceleration(0.0f), mass(mass), inverseMass(1.0f),
      momentOfInertia(1.0f), inverseInertia(1.0f), restitution(0.5f),
      friction(0.3f), drag(0.01f), forceAccumulator(0.0f, 0.0f), torqueAccumulator(0.0f), bodyType(BodyType::DYNAMIC),
      awake(true), radius(radius)
{
    setMass(mass);
}

// destructor
RigidBody::~RigidBody() = default;

// mass and inertia
void RigidBody::setMass(float const m)
{
    if (m <= 0.0f)
    {
        // infinite mass (static body)
        mass = 0.0f;
        inverseMass = 0.0f;
        bodyType = BodyType::STATIC;
    }
    else
    {
        mass = m;
        inverseMass = 1.0f / mass;
        if (bodyType == BodyType::STATIC)
        {
            bodyType = BodyType::DYNAMIC;
        }
    }

    // calculate moment of inertia for a circle: I = 0.5 * m * r^2
    if (mass > 0.0f)
    {
        momentOfInertia = 0.5f * mass * radius * radius;
        inverseInertia = 1.0f / momentOfInertia;
    }
    else
    {
        momentOfInertia = 0.0f;
        inverseInertia = 0.0f;
    }
}

void RigidBody::setMomentOfInertia(const float inertia)
{
    if (inertia <= 0.0f)
    {
        momentOfInertia = 0.0f;
        inverseInertia = 0.0f;
    }
    else
    {
        momentOfInertia = inertia;
        inverseInertia = 1.0f / inertia;
    }
}

// body type
void RigidBody::setBodyType(const BodyType type)
{
    bodyType = type;
    if (type == BodyType::STATIC)
    {
        inverseMass = 0.0f;
        inverseInertia = 0.0f;
        velocity.setZero();
        angularVelocity = 0.0f;
        acceleration.setZero();
        angularAcceleration = 0.0f;
    }
    else if (mass > 0.0f)
    {
        inverseMass = 1.0f / mass;
        if (momentOfInertia > 0.0f)
        {
            inverseInertia = 1.0f / momentOfInertia;
        }
    }
}

// force and impulse application
void RigidBody::addForce(const Vector2D &force)
{
    if (bodyType == BodyType::STATIC)
        return;

    forceAccumulator += force;
    awake = true;
}

void RigidBody::addForceAtPoint(const Vector2D &force, const Vector2D &point)
{
    if (bodyType == BodyType::STATIC)
        return;

    forceAccumulator += force;

    // calculate torque: τ = r × F
    const Vector2D r = point - position;
    const float torque = r.cross(force);
    torqueAccumulator += torque;

    awake = true;
}

void RigidBody::addTorque(const float torque)
{
    if (bodyType == BodyType::STATIC)
        return;

    torqueAccumulator += torque;
    awake = true;
}

void RigidBody::addImpulse(const Vector2D &impulse)
{
    if (bodyType == BodyType::STATIC)
        return;

    velocity += impulse * inverseMass;
    awake = true;
}

void RigidBody::addImpulseAtPoint(const Vector2D &impulse, const Vector2D &point)
{
    if (bodyType == BodyType::STATIC)
        return;

    velocity += impulse * inverseMass;

    // calculate angular impulse
    const Vector2D r = point - position;
    const float angularImpulse = r.cross(impulse);
    angularVelocity += angularImpulse * inverseInertia;

    awake = true;
}

// integration
void RigidBody::integrate(const float deltaTime)
{
    if (bodyType == BodyType::STATIC || !awake)
        return;

    // calculate acceleration from forces
    acceleration = forceAccumulator * inverseMass;
    angularAcceleration = torqueAccumulator * inverseInertia;

    // apply drag
    const Vector2D dragForce = velocity * (-drag);
    acceleration += dragForce * inverseMass;

    // integrate velocity: v = v0 + a*dt
    velocity += acceleration * deltaTime;
    angularVelocity += angularAcceleration * deltaTime;

    // apply damping
    velocity *= std::pow(0.999f, deltaTime); // Simple velocity damping
    angularVelocity *= std::pow(0.999f, deltaTime);

    // integrate position: p = p0 + v*dt
    position += velocity * deltaTime;
    rotation += angularVelocity * deltaTime;

    // normalize rotation to [0, 2π]
    while (rotation < 0.0f)
        rotation += 2.0f * M_PI;
    while (rotation >= 2.0f * M_PI)
        rotation -= 2.0f * M_PI;
}

void RigidBody::clearAccumulators()
{
    forceAccumulator.setZero();
    torqueAccumulator = 0.0f;
}

// utility methods
Vector2D RigidBody::getPointVelocity(const Vector2D &point) const
{
    const Vector2D r = point - position;
    // v = v_center + ω × r
    const Vector2D rotationalVelocity(-r.y * angularVelocity, r.x * angularVelocity);
    return velocity + rotationalVelocity;
}

// debug information
void RigidBody::printDebugInfo() const
{
    std::cout << "RigidBody Debug Info:\n";
    std::cout << "  Position: " << position << "\n";
    std::cout << "  Velocity: " << velocity << "\n";
    std::cout << "  Mass: " << mass << " (inverse: " << inverseMass << ")\n";
    std::cout << "  Radius: " << radius << "\n";
    std::cout << "  Type: " << (bodyType == BodyType::STATIC ? "Static" : "Dynamic") << "\n";
    std::cout << "  Awake: " << (awake ? "Yes" : "No") << "\n";
    std::cout << "  Restitution: " << restitution << "\n";
    std::cout << "  Friction: " << friction << "\n";
}
