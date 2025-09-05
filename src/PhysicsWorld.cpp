//
// Created by Arthur Hennig on 04.09.2025.
//

#include "PhysicsWorld.hpp"
#include "CollisionDetector.hpp"
#include "ForceGenerator.hpp"
#include <iostream>
#include <algorithm>

// constructor and destructor
PhysicsWorld::PhysicsWorld(const Vector2D &gravity)
    : gravity(gravity), damping(0.999f), velocityIterations(8), positionIterations(3), sleepEpsilon(0.1f)
{
    collisionDetector = std::make_unique<CollisionDetector>();
}

PhysicsWorld::~PhysicsWorld()
{
    clearBodies();
    clearForceGenerators();
}

// Body management
std::shared_ptr<RigidBody> PhysicsWorld::createBody(const Vector2D &position, float mass, float radius)
{
    auto body = std::make_shared<RigidBody>(position, mass, radius);
    bodies.push_back(body);
    return body;
}

bool PhysicsWorld::addBody(const std::shared_ptr<RigidBody>& body)
{
    if (body && std::find(bodies.begin(), bodies.end(), body) == bodies.end())
    {
        bodies.push_back(body);
        return true;
    };
    return false;
}

bool PhysicsWorld::removeBody(const std::shared_ptr<RigidBody>& body)
{
    if (const auto it = std::find(bodies.begin(), bodies.end(), body); it != bodies.end())
    {
        bodies.erase(it);
        return true;
    };
    return false;
}

void PhysicsWorld::clearBodies()
{
    bodies.clear();
}

// force generator management
bool PhysicsWorld::addForceGenerator(const std::shared_ptr<ForceGenerator>& generator)
{
    if (generator && std::find(forceGenerators.begin(), forceGenerators.end(), generator) == forceGenerators.end())
    {
        forceGenerators.push_back(generator);
        return true;
    }
    return false;
}

bool PhysicsWorld::removeForceGenerator(const std::shared_ptr<ForceGenerator>& generator)
{
    if (const auto it = std::find(forceGenerators.begin(), forceGenerators.end(), generator); it != forceGenerators.end())
    {
        forceGenerators.erase(it);
        return true;
    }
    return false;
}

void PhysicsWorld::clearForceGenerators()
{
    forceGenerators.clear();
}

// a single simulation step
void PhysicsWorld::step(float timeDelta) const {
    // clamp timeDelta to prevent instability
    timeDelta = std::min(timeDelta, 0.016f); // max 60fps step

    // apply forces
    applyForces(timeDelta);

    // integrate all physical bodies
    integrate(timeDelta);

    // detect and resolve collisions
    detectCollisions();
    resolveCollisions();

    // update sleep state
    updateSleepState();

    // clear force accumulators for next frame
    for (auto &body : bodies)
    {
        body->clearAccumulators();
    }
}

// individual simulation phases
void PhysicsWorld::applyForces(const float timeDelta) const {
    // apply gravity to all dynamic bodies
    applyGravity();

    // apply custom force generators
    for (auto &generator : forceGenerators)
    {
        generator->applyForce();
        generator->updateTime(timeDelta);
    }

    // apply global damping
    applyDamping();
}

// update each bodies position based on the time delta
void PhysicsWorld::integrate(const float timeDelta) const {
    for (auto &body : bodies)
    {
        body->integrate(timeDelta);
    }
}

void PhysicsWorld::detectCollisions() const {
    // check for collisions with world border
    for (auto &body : bodies)
    {
        if (body->isStatic())
            continue;
        const Vector2D position = body->getPosition();
        const float radius = body->getRadius();
        const float upper = position.y - radius;
        const float lower = position.y + radius;
        const float left = position.x - radius;
        const float right = position.x + radius;

        if (upper <= 0 || lower >= dimensions.y || left <= 0 || right >= dimensions.x) { // collision

        }
    }
    collisionDetector->clearCollisions();
    collisionDetector->detectCollisions(bodies);
}

void PhysicsWorld::resolveCollisions() const {
    collisionDetector->resolveCollisions();
}

void PhysicsWorld::updateSleepState() const {
    for (auto &body : bodies)
    {
        if (body->isDynamic() && body->isAwake())
        {
            // check if body should go to sleep based on low velocity
            // check if body should go to sleep
            // constexpr float sleepEpsilon = 0.1f;
            // if (body->getAcceleration().magnitudeSquared() < sleepEpsilon * sleepEpsilon &&
            //     body->getRotation() < sleepEpsilon)
            // {
            //     awake = false;
            //     velocity.setZero();
            //     angularVelocity = 0.0f;
            // }
        }
    }
}

// utility methods
std::shared_ptr<RigidBody> PhysicsWorld::getBodyAt(const Vector2D &point) const
{
    for (const auto &body : bodies)
    {
        if (const float distanceSq = point.distanceSquaredTo(body->getPosition()); distanceSq <= body->getRadius() * body->getRadius())
        {
            return body;
        }
    }
    return nullptr;
}

std::vector<std::shared_ptr<RigidBody>> PhysicsWorld::getBodiesInRadius(const Vector2D &center, const float radius) const
{
    std::vector<std::shared_ptr<RigidBody>> result;
    const float radiusSquared = radius * radius;

    for (const auto &body : bodies)
    {
        const float distanceSq = center.distanceSquaredTo(body->getPosition());
        if (distanceSq <= radiusSquared)
        {
            result.push_back(body);
        }
    }

    return result;
}

void PhysicsWorld::setAllBodiesAwake() const {
    for (auto &body : bodies)
    {
        body->setAwake(true);
    }
}

void PhysicsWorld::applyExplosion(const Vector2D &center, const float force, const float radius) const {
    ExplosionGenerator explosion(center, force, radius, 1.0f);
    for (auto &body : bodies)
    {
        explosion.affect(body);
    }
    explosion.applyForce();
}

// debug and statistics
void PhysicsWorld::printDebugInfo() const
{
    std::cout << "PhysicsWorld Debug Info:\n";
    std::cout << "  Bodies: " << bodies.size() << "\n";
    std::cout << "  Force Generators: " << forceGenerators.size() << "\n";
    std::cout << "  Gravity: " << gravity << "\n";
    std::cout << "  Awake Bodies: " << getAwakeBodyCount() << "\n";
    std::cout << "  Sleeping Bodies: " << getSleepingBodyCount() << "\n";
    std::cout << "  Active Collisions: " << collisionDetector->getCollisionCount() << "\n";
}

// print the most important body data to the standard output
void PhysicsWorld::printBodiesInfo() const {
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        const auto body = bodies[i];
        std::cout << "Body " << i+1 << ": "
                  << "Pos(" << body->getPosition().x << ", " << body->getPosition().y << ") "
                  << "Vel(" << body->getVelocity().x << ", " << body->getVelocity().y << ") "
                  << "Mass(" << body->getMass() << ") "
                  << (body->isDynamic() ? "Dynamic" : "Static")
                  << "\n";
    }
}

int PhysicsWorld::getAwakeBodyCount() const
{
    int count = 0;
    for (const auto &body : bodies)
    {
        if (body->isAwake())
            count++;
    }
    return count;
}

int PhysicsWorld::getSleepingBodyCount() const
{
    int count = 0;
    for (const auto &body : bodies)
    {
        if (!body->isAwake())
            count++;
    }
    return count;
}

// Private helper methods
void PhysicsWorld::applyGravity() const {
    for (auto &body : bodies)
    {
        if (body->isDynamic() && body->isAwake())
        {
            Vector2D gravityForce = gravity * body->getMass();
            body->addForce(gravityForce);
        }
    }
}

void PhysicsWorld::applyDamping() const {
    for (auto &body : bodies)
    {
        if (body->isDynamic() && body->isAwake())
        {
            Vector2D velocity = body->getVelocity();
            body->setVelocity(velocity * damping);
        }
    }
}

void PhysicsWorld::removeBodiesMarkedForDeletion()
{
    // This could be used if we implement body deletion marking
    // For now, bodies are removed explicitly via removeBody()
}
