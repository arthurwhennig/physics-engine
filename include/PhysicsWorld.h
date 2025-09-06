//
//  PhysicsWorld.h
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_PHYSICSWORLD_H
#define PHYSICSENGINE_PHYSICSWORLD_H
#endif //PHYSICSENGINE_PHYSICSWORLD_H

#include <vector>
#include <memory>

#include <CollisionDetector.h>
#include <ForceGenerator.h>
#include <RigidBody.h>
#include <Vector2D.h>

/**
 * @brief manages the physics simulation world
 *
 * the PhysicsWorld contains all rigid bodies, handles integration,
 * collision detection, and force generation.
 */
class PhysicsWorld
{
private:
    std::vector<std::shared_ptr<RigidBody>> bodies;
    std::vector<std::shared_ptr<ForceGenerator>> forceGenerators;
    std::unique_ptr<CollisionDetector> collisionDetector;

    // world properties
    Vector2D dimensions;
    Vector2D gravity;
    float damping;

    // simulation parameters
    int velocityIterations;
    int positionIterations;
    float sleepEpsilon; // Threshold for putting bodies to sleep

public:
    // constructor and destructor
    explicit PhysicsWorld(const Vector2D &gravity = Vector2D(0.0f, -9.81f));
    ~PhysicsWorld();

    // body management
    std::shared_ptr<RigidBody> createBody(const Vector2D &position,
                                          float mass = 1.0f);
    bool addBody(const std::shared_ptr<RigidBody>& body);
    bool removeBody(const std::shared_ptr<RigidBody>& body);
    void clearBodies();

    [[nodiscard]] const std::vector<std::shared_ptr<RigidBody>> &getBodies() const { return bodies; }
    [[nodiscard]] int getBodyCount() const { return static_cast<int>(bodies.size()); }

    // force generator management
    bool addForceGenerator(const std::shared_ptr<ForceGenerator>& generator);
    bool removeForceGenerator(const std::shared_ptr<ForceGenerator>& generator);
    void clearForceGenerators();

    // world properties
    [[nodiscard]] const Vector2D &getGravity() const { return gravity; }
    void setGravity(const Vector2D &g) { gravity = g; }
    void setDimensions(const Vector2D &dim) { dimensions = dim; }
    [[nodiscard]] const Vector2D &getDimensions() const { return dimensions; }

    // damping
    [[nodiscard]] float getDamping() const { return damping; }
    void setDamping(const float d) { damping = std::max(0.0f, std::min(1.0f, d)); }

    // simulation parameters
    [[nodiscard]] int getVelocityIterations() const { return velocityIterations; }
    void setVelocityIterations(const int iterations) { velocityIterations = std::max(1, iterations); }

    [[nodiscard]] int getPositionIterations() const { return positionIterations; }
    void setPositionIterations(const int iterations) { positionIterations = std::max(1, iterations); }

    // simulation step
    void step(float timeDelta) const;

    // individual simulation phases
    void applyForces(float timeDelta) const;
    void integrate(float timeDelta) const;
    void detectCollisions() const;
    void resolveCollisions() const;
    void updateSleepState() const;

    // utility methods
    [[nodiscard]] std::shared_ptr<RigidBody> getBodyAt(const Vector2D &point) const;
    [[nodiscard]] std::vector<std::shared_ptr<RigidBody>> getBodiesInRadius(const Vector2D &center, float radius) const;

    void setAllBodiesAwake() const;
    void applyExplosion(const Vector2D &center, float force, float radius) const;

    // debug and statistics
    void printDebugInfo() const;
    void printBodiesInfo() const;
    [[nodiscard]] int getAwakeBodyCount() const;
    [[nodiscard]] int getSleepingBodyCount() const;

private:
    // internal helper methods
    void applyGravity() const;
    void applyDamping() const;

    static void removeBodiesMarkedForDeletion();
};
