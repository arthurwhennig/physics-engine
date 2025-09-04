//
//  PhysicsWorld.hpp
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#include <vector>
#include <memory>

#include "RigidBody.hpp"
#include "Vector2D.hpp"
#include "ForceGenerator.hpp"
#include "CollisionDetector.hpp"

#ifndef PhysicsWorld_h
#define PhysicsWorld_h
#endif /* PhysicsWorld_h */

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
    PhysicsWorld(const Vector2D &gravity = Vector2D(0.0f, -9.81f));
    ~PhysicsWorld();

    // body management
    std::shared_ptr<RigidBody> createBody(const Vector2D &position,
                                          float mass = 1.0f,
                                          float radius = 1.0f);
    bool addBody(std::shared_ptr<RigidBody> body);
    bool removeBody(std::shared_ptr<RigidBody> body);
    void clearBodies();

    const std::vector<std::shared_ptr<RigidBody>> &getBodies() const { return bodies; }
    size_t getBodyCount() const { return bodies.size(); }

    // force generator management
    bool addForceGenerator(std::shared_ptr<ForceGenerator> generator);
    bool removeForceGenerator(std::shared_ptr<ForceGenerator> generator);
    void clearForceGenerators();

    // world properties
    const Vector2D &getGravity() const { return gravity; }
    void setGravity(const Vector2D &g) { gravity = g; }
    void setDimensions(Vector2D &dim) { dimensions = dim; }
    const Vector2D &getDimensions() const { return dimensions; }

    // damping
    float getDamping() const { return damping; }
    void setDamping(float d) { damping = std::max(0.0f, std::min(1.0f, d)); }

    // simulation parameters
    int getVelocityIterations() const { return velocityIterations; }
    void setVelocityIterations(int iterations) { velocityIterations = std::max(1, iterations); }

    int getPositionIterations() const { return positionIterations; }
    void setPositionIterations(int iterations) { positionIterations = std::max(1, iterations); }

    // simulation step
    void step(float deltaTime);

    // individual simulation phases
    void applyForces(float deltaTime);
    void integrate(float deltaTime);
    void detectCollisions();
    void resolveCollisions();
    void updateSleepState();

    // utility methods
    std::shared_ptr<RigidBody> getBodyAt(const Vector2D &point) const;
    std::vector<std::shared_ptr<RigidBody>> getBodiesInRadius(const Vector2D &center, float radius) const;

    void setAllBodiesAwake();
    void applyExplosion(const Vector2D &center, float force, float radius);

    // debug and statistics
    void printDebugInfo() const;
    int getAwakeBodyCount() const;
    int getSleepingBodyCount() const;

private:
    // internal helper methods
    void applyGravity();
    void applyDamping();
    void removeBodiesMarkedForDeletion();
};
