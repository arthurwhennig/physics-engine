//
// Created by Arthur Hennig on 04.09.2025.
//


#include "CollisionDetector.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

CollisionDetector::CollisionDetector() = default;

CollisionDetector::~CollisionDetector() = default;

// main collision detection methods
void CollisionDetector::detectCollisions(const std::vector<std::shared_ptr<RigidBody>> &bodies)
{
    collisions.clear();

    // simple N^2 collision detection (can be optimized with spatial partitioning)
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        for (size_t j = i + 1; j < bodies.size(); ++j)
        {
            CollisionInfo collision;

            if (testCircleCircle(*bodies[i], *bodies[j], collision))
            {
                collision.bodyA = bodies[i];
                collision.bodyB = bodies[j];
                collision.separatingVelocity = calculateSeparatingVelocity(collision);

                // only add collision if objects are moving towards each other
                if (collision.separatingVelocity < 0.0f)
                {
                    collisions.push_back(collision);
                }
            }
        }
    }
}

// collision resolution
void CollisionDetector::resolveCollisions() const {
    for (auto &collision : collisions)
    {
        resolveCollision(collision);
    }
}

void CollisionDetector::resolveCollision(const CollisionInfo &collision)
{
    // Resolve velocity (bounce)
    resolveVelocity(collision);

    // Resolve position (separate overlapping objects)
    resolvePosition(collision);
}

// Primitive collision tests
bool CollisionDetector::testCircleCircle(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision)
{
    const Vector2D& centerA = bodyA.getPosition();
    const Vector2D& centerB = bodyB.getPosition();
    const float radiusA = bodyA.getRadius();
    const float radiusB = bodyB.getRadius();

    const Vector2D direction = centerB - centerA;
    const float distance = direction.magnitude();

    if (const float combinedRadius = radiusA + radiusB; distance < combinedRadius && distance > 0.0f)
    {
        // Collision detected
        collision.contactNormal = direction.normalized();
        collision.penetrationDepth = combinedRadius - distance;
        collision.contactPoint = centerA + collision.contactNormal * radiusA;
        return true;
    }

    return false;
}

// Utility methods
Vector2D CollisionDetector::getContactPoint(const RigidBody &bodyA, const RigidBody &bodyB)
{
    const Vector2D direction = bodyB.getPosition() - bodyA.getPosition();
    if (const float distance = direction.magnitude(); distance > 0.0f)
    {
        const Vector2D normal = direction / distance;
        return bodyA.getPosition() + normal * bodyA.getRadius();
    }

    return bodyA.getPosition(); // Fallback for zero distance
}

float CollisionDetector::calculateSeparatingVelocity(const CollisionInfo &collision)
{
    const Vector2D relativeVelocity = collision.bodyB->getVelocity() - collision.bodyA->getVelocity();
    return relativeVelocity.dot(collision.contactNormal);
}

// Broad phase collision detection
std::vector<std::pair<size_t, size_t>> CollisionDetector::broadPhase(const std::vector<std::shared_ptr<RigidBody>> &bodies)
{
    // Simple implementation - just return all pairs
    // In a real engine, this would use spatial partitioning (quad-tree, grid, etc.)
    std::vector<std::pair<size_t, size_t>> pairs;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        for (size_t j = i + 1; j < bodies.size(); ++j)
        {
            pairs.emplace_back(i, j);
        }
    }

    return pairs;
}

// Debug information
void CollisionDetector::printCollisionInfo() const
{
    std::cout << "Active Collisions: " << collisions.size() << "\n";

    for (size_t i = 0; i < collisions.size(); ++i)
    {
        const auto &collision = collisions[i];
        std::cout << "Collision " << i << ":\n";
        std::cout << "  Contact Point: " << collision.contactPoint << "\n";
        std::cout << "  Contact Normal: " << collision.contactNormal << "\n";
        std::cout << "  Penetration: " << collision.penetrationDepth << "\n";
        std::cout << "  Separating Velocity: " << collision.separatingVelocity << "\n";
    }
}

// Private helper methods for collision resolution
void CollisionDetector::resolveVelocity(const CollisionInfo &collision) {
    const float separatingVelocity = collision.separatingVelocity;

    // Don't resolve if separating velocity is positive (moving apart)
    if (separatingVelocity > 0.0f)
        return;

    // Calculate restitution
    const float restitution = calculateRestitution(collision);

    // Calculate new separating velocity
    const float newSepVelocity = -separatingVelocity * restitution;

    // Calculate change in velocity
    const float deltaVelocity = newSepVelocity - separatingVelocity;

    // Calculate total inverse mass
    const float totalInverseMass = collision.bodyA->getInverseMass() + collision.bodyB->getInverseMass();

    // If both objects are static/infinite mass, no resolution needed
    if (totalInverseMass <= 0.0f)
        return;

    // Calculate impulse per unit mass
    const float impulse = deltaVelocity / totalInverseMass;

    // Calculate impulse vector
    const Vector2D impulseVector = collision.contactNormal * impulse;

    // Apply impulse to both bodies
    if (collision.bodyA->hasFiniteMass())
    {
        collision.bodyA->addImpulse(-impulseVector);
    }

    if (collision.bodyB->hasFiniteMass())
    {
        collision.bodyB->addImpulse(impulseVector);
    }
}

void CollisionDetector::resolvePosition(const CollisionInfo &collision)
{
    // Only resolve position if there's penetration
    if (collision.penetrationDepth <= 0.0f)
        return;

    const float totalInverseMass = collision.bodyA->getInverseMass() + collision.bodyB->getInverseMass();

    // If both objects are static/infinite mass, no resolution needed
    if (totalInverseMass <= 0.0f)
        return;

    // Calculate movement per unit of inverse mass
    const Vector2D movePerIMass = collision.contactNormal * (collision.penetrationDepth / totalInverseMass);

    // Calculate position adjustments
    const Vector2D bodyAMovement = movePerIMass * collision.bodyA->getInverseMass();
    const Vector2D bodyBMovement = movePerIMass * collision.bodyB->getInverseMass();

    // Apply position corrections
    if (collision.bodyA->hasFiniteMass())
    {
        collision.bodyA->setPosition(collision.bodyA->getPosition() - bodyAMovement);
    }

    if (collision.bodyB->hasFiniteMass())
    {
        collision.bodyB->setPosition(collision.bodyB->getPosition() + bodyBMovement);
    }
}

float CollisionDetector::calculateRestitution(const CollisionInfo &collision) {
    // Use the minimum restitution of the two bodies
    return std::min(collision.bodyA->getRestitution(), collision.bodyB->getRestitution());
}

float CollisionDetector::calculateFriction(const CollisionInfo &collision)
{
    // Use the geometric mean of the friction coefficients
    return std::sqrt(collision.bodyA->getFriction() * collision.bodyB->getFriction());
}
