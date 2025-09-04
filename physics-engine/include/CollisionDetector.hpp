//
//  CollisionDetector.hpp
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#pragma once
#include <vector>
#include <memory>

#include "RigidBody.hpp"
#include "Vector2D.hpp"

#ifndef CollisionDetector_h
#define CollisionDetector_h
#endif /* CollisionDetector_h */

/**
 * @brief structure to hold collision information
 */
struct CollisionInfo
{
    std::shared_ptr<RigidBody> bodyA;
    std::shared_ptr<RigidBody> bodyB;
    Vector2D contactPoint;    // point of contact in world coordinates
    Vector2D contactNormal;   // normal from A to B
    float penetrationDepth;   // how deep the collision is
    float separatingVelocity; // velocity in direction of contact normal

    CollisionInfo() : penetrationDepth(0.0f), separatingVelocity(0.0f) {}
};

/**
 * @brief handles collision detection between rigid bodies
 *
 * currently supports circle-circle collisions. can be extended
 * for other primitive shapes.
 */
class CollisionDetector
{
private:
    std::vector<CollisionInfo> collisions;

public:
    CollisionDetector();
    ~CollisionDetector();

    // main collision detection methods
    void detectCollisions(const std::vector<std::shared_ptr<RigidBody>> &bodies);
    const std::vector<CollisionInfo> &getCollisions() const { return collisions; }
    void clearCollisions() { collisions.clear(); }

    // collision resolution
    void resolveCollisions() const;
    static void resolveCollision(const CollisionInfo &collision);

    // primitive collision tests
    static bool testCircleCircle(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision);

    // utility methods
    static Vector2D getContactPoint(const RigidBody &bodyA, const RigidBody &bodyB);
    static float calculateSeparatingVelocity(const CollisionInfo &collision);

    // broad phase collision detection (spatial partitioning could be added here)
    static std::vector<std::pair<size_t, size_t>> broadPhase(const std::vector<std::shared_ptr<RigidBody>> &bodies);

    // debug information
    void printCollisionInfo() const;
    size_t getCollisionCount() const { return collisions.size(); }

private:
    // helper methods for collision resolution
    static void resolveVelocity(const CollisionInfo &collision) ;
    static void resolvePosition(const CollisionInfo &collision);

    static float calculateRestitution(const CollisionInfo &collision);
    static float calculateFriction(const CollisionInfo &collision) ;
};
