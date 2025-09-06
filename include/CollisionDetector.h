//
//  CollisionDetector.h
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_COLLISIONDETECTOR_H
#define PHYSICSENGINE_COLLISIONDETECTOR_H
#endif //PHYSICSENGINE_COLLISIONDETECTOR_H

#include <vector>
#include <memory>

#include <RigidBody.h>
#include <Vector2D.h>
#include <Quadtree.h>

/**
 * @brief structure to hold collision information
 */
struct CollisionInfo
{
    std::shared_ptr<RigidBody> bodyA;
    std::shared_ptr<RigidBody> bodyB;
    
    // contact manifold (polygon collisions can have multiple contact points)
    static constexpr int MAX_CONTACT_POINTS = 2;
    Vector2D contactPoints[MAX_CONTACT_POINTS];  // points of contact in world coordinates
    int contactCount;                            // number of active contact points
    
    Vector2D contactNormal;     // normal from A to B
    Vector2D penetration;       // smallest vector of translation needed to separate bodyA and bodyB
    float separatingVelocity;   // velocity in direction of contact normal

    CollisionInfo() : contactCount(0), separatingVelocity(0.0f) {}
    
    // helper method to get primary contact point (for backward compatibility)
    [[nodiscard]] Vector2D getContactPoint() const { return contactCount > 0 ? contactPoints[0] : Vector2D::zero(); }
    [[nodiscard]] Vector2D getContactNormal() const { return contactNormal; }
    [[nodiscard]] Vector2D getPenetration() const { return penetration; }
    [[nodiscard]] float getPenetrationDepth() const { return penetration.magnitude(); };
    
    // add a contact point to the manifold
    void addContactPoint(const Vector2D& point) {
        if (contactCount < MAX_CONTACT_POINTS) {
            contactPoints[contactCount++] = point;
        }
    }
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
    std::unique_ptr<Quadtree> quadtree;
    bool useQuadtree;
    AABB worldBounds;

public:
    explicit CollisionDetector(bool enableQuadtree = true);
    explicit CollisionDetector(const AABB& worldBounds, bool enableQuadtree = true);
    ~CollisionDetector();

    // main collision detection methods
    void detectCollisions(const std::vector<std::shared_ptr<RigidBody>> &bodies);
    [[nodiscard]] const std::vector<CollisionInfo> &getCollisions() const { return collisions; }
    void clearCollisions() { collisions.clear(); }

    // collision resolution
    void resolveCollisions() const;
    static void resolveCollision(const CollisionInfo &collision);

    // primitive collision tests
    static bool testCircleCircle(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision);
    static bool testPolygonPolygon(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision);
    static bool testCirclePolygon(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision);
    
    static bool testCollision(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision);

    // utility methods
    static Vector2D getContactPoint(const RigidBody &bodyA, const RigidBody &bodyB);
    static float calculateSeparatingVelocity(const CollisionInfo &collision);

    // broad phase collision detection (spatial partitioning could be added here)
    static std::vector<std::pair<size_t, size_t>> broadPhase(const std::vector<std::shared_ptr<RigidBody>> &bodies);

    // quadtree control
    void enableQuadtree(bool enable) { useQuadtree = enable; }
    void setWorldBounds(const AABB& bounds);
    [[nodiscard]] std::vector<AABB> getQuadtreeVisualization() const;
    
    // debug information
    void printCollisionInfo() const;
    void printQuadtreeStats() const;
    [[nodiscard]] int getCollisionCount() const { return static_cast<int>(collisions.size()); }

private:
    // helper methods for collision resolution
    static void resolveVelocity(const CollisionInfo &collision) ;
    static void resolvePosition(const CollisionInfo &collision);

    static float calculateRestitution(const CollisionInfo &collision);
    static float calculateFriction(const CollisionInfo &collision);

    // polygon collision helper methods
    struct SATResult {
        bool collision;
        Vector2D mtv;           // minimum translation vector
        float penetration;
        size_t referenceEdge;   // which edge of reference polygon;
    };

    static SATResult performSAT(const Polygon& polyA, const Polygon& polyB);
    static void findContactPoints(const RigidBody& bodyA, const RigidBody& bodyB, 
                                const SATResult& satResult, CollisionInfo& collision);
    static std::vector<Vector2D> clipPolygonToLine(const std::vector<Vector2D>& polygon, 
                                                  const Vector2D& linePoint, const Vector2D& lineNormal);
    static float projectVertexOnAxis(const Vector2D& vertex, const Vector2D& axis);
};
