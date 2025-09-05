//
// Created by Arthur Hennig on 04.09.2025.
//

#include "ForceGenerator.hpp"
#include <algorithm>

// GravityGenerator implementation

// apply gravity to all affected bodies
void GravityGenerator::applyForce()
{
    for (const auto &affected : affectedBodies)
    {
        if (!affected->isAwake())
            continue;
        Vector2D gravityForce = gravity * affected->getMass();
        affected->addForce(gravityForce);
    }
}

// return true if the given body is affected by this generator
bool GravityGenerator::isAffected(const std::shared_ptr<RigidBody> body)
{
    return std::find(affectedBodies.begin(), affectedBodies.end(), body) != affectedBodies.end();
}

// add the body to the affected bodies and return true if it is an affected body afterwards
bool GravityGenerator::affect(const std::shared_ptr<RigidBody> body)
{
    if (!body->isDynamic())
        return false;
    if (std::find(affectedBodies.begin(), affectedBodies.end(), body) == affectedBodies.end())
        affectedBodies.push_back(body);
    return true;
}

// remove the body from the affected bodies and return true if it got removed
bool GravityGenerator::release(const std::shared_ptr<RigidBody> body)
{
    const auto it = std::find(affectedBodies.begin(), affectedBodies.end(), body);
    if (it != affectedBodies.end())
    {
        affectedBodies.erase(it);
        return true;
    }
    return false;
}

// DragGenerator implementation
void DragGenerator::applyForce()
{
    for (const auto &affected : affectedBodies)
    {
        if (!affected->isAwake())
            continue;
        Vector2D velocity = affected->getVelocity();
        const float speed = velocity.magnitude();

        if (speed <= 0.0f)
            continue;

        Vector2D dragDirection = velocity.normalized();
        // Calculate drag force: F_drag = -k1*v - k2*v²*v̂
        const float linearDragMagnitude = k1 * speed;
        const float quadraticDragMagnitude = k2 * speed * speed;
        Vector2D dragForce = dragDirection * -(linearDragMagnitude + quadraticDragMagnitude);

        affected->addForce(dragForce);
    }
}

bool DragGenerator::isAffected(std::shared_ptr<RigidBody> body)
{
    return find(affectedBodies.begin(), affectedBodies.end(), body) != affectedBodies.end();
}

// tries to add the body to the affected bodies and returns true if it is an affected body afterwards
bool DragGenerator::affect(const std::shared_ptr<RigidBody> body)
{
    if (!body->isDynamic())
        return false;
    if (const auto it = std::find(affectedBodies.begin(), affectedBodies.end(), body); it == affectedBodies.end())
        affectedBodies.push_back(body);
    return true;
}

// tries to remove the body from the affected bodies and returns true if it got removed
bool DragGenerator::release(const std::shared_ptr<RigidBody> body)
{
    if (const auto it = std::find(affectedBodies.begin(), affectedBodies.end(), body); it != affectedBodies.end())
    {
        affectedBodies.erase(it);
        return true;
    }
    return false;
}

// SpringGenerator implementation
SpringGenerator::SpringGenerator(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b,
                                 const Vector2D &localA, const Vector2D &localB,
                                 const float k, const float restLen, const float damp)
    : localPointA(localA), localPointB(localB),
      springConstant(k), restLength(restLen), damping(damp)
{
    affectedBodies.push_back(a);
    affectedBodies.push_back(b);
}

SpringGenerator::SpringGenerator(const std::shared_ptr<RigidBody>& body, const Vector2D &worldPoint,
                                 const Vector2D &localPoint, float k, float restLen, float damp)
    : fixedPoint(worldPoint), localPointA(localPoint),
      springConstant(k), restLength(restLen), damping(damp)
{
    affectedBodies.push_back(body);
}

void SpringGenerator::applyForce()
{
    if (affectedBodies.empty())
        return;

    if (affectedBodies.size() == 1)
    { // only bodyA, use fixedPoint
        std::shared_ptr<RigidBody> body = affectedBodies[0];
        if (!body->isAwake())
            return;
        const Vector2D posA = getWorldPoint(body, localPointA);
        const Vector2D posB = fixedPoint;

        const Vector2D displacement = posB - posA;
        const float currentLength = displacement.magnitude();

        if (currentLength <= 0.0f)
            return;

        const Vector2D springDirection = displacement.normalized();

        // Calculate spring force: F = k * (current_length - rest_length)
        const float springForceMagnitude = springConstant * (currentLength - restLength);
        Vector2D springForce = springDirection * springForceMagnitude;

        if (damping > 0.0f)
        {
            const Vector2D relativeVelocity = -body->getPointVelocity(posA);
            const float dampingForceMagnitude = damping * relativeVelocity.dot(springDirection);
            const Vector2D dampingForce = springDirection * dampingForceMagnitude;
            springForce += dampingForce;
        }
        body->addForceAtPoint(springForce, posA);
    }
    else if (affectedBodies.size() == 2)
    { // apply to each of the other bodies after the first body (bodyA)
        std::shared_ptr<RigidBody> bodyA = affectedBodies[0];
        std::shared_ptr<RigidBody> bodyB = affectedBodies[1];
        if (!bodyA->isAwake() || !bodyB->isAwake())
            return;

        const Vector2D posA = getWorldPoint(bodyA, localPointA);
        const Vector2D posB = getWorldPoint(bodyB, localPointB);

        const Vector2D displacement = posB - posA;
        const float currentLength = displacement.magnitude();

        if (currentLength <= 0.0f)
            return;

        const Vector2D springDirection = displacement.normalized();

        // calculate spring force: F = k * (currentLength - restLength)
        const float springForceMagnitude = springConstant * (currentLength - restLength);
        Vector2D springForce = springDirection * springForceMagnitude;

        // calculate the damping force if damping coefficient is set
        if (damping > 0.0f)
        {
            const Vector2D relativeVelocity = bodyB->getPointVelocity(posB) - bodyA->getPointVelocity(posA);
            const float dampingForceMagnitude = damping * relativeVelocity.dot(springDirection);
            const Vector2D dampingForce = springDirection * dampingForceMagnitude;
            springForce += dampingForce;
        }
        bodyB->addForceAtPoint(springForce, posB);
    }
}

bool SpringGenerator::isAffected(std::shared_ptr<RigidBody> body)
{
    return std::find(affectedBodies.begin(), affectedBodies.end(), body) != affectedBodies.end();
}

bool SpringGenerator::affect(const std::shared_ptr<RigidBody> body)
{
    if (!body->isDynamic())
        return false;
    if (affectedBodies.size() == 1)
    {
        affectedBodies.push_back(body);
        return true;
    }
    return false;
}

bool SpringGenerator::release(const std::shared_ptr<RigidBody> body)
{
    if (affectedBodies.size() != 2)
        return false;
    affectedBodies.pop_back();
    return true;
}

Vector2D SpringGenerator::getWorldPoint(const std::shared_ptr<RigidBody>& body, const Vector2D &localPoint)
{
    // for now assume local point is just offset from center (no rotation)
    // in a full implementation, this would apply rotation transformation
    return body->getPosition() + localPoint;
}

// ConstantForceGenerator implementation
void ConstantForceGenerator::applyForce()
{
    for (const auto &affected : affectedBodies)
    {
        if (!affected->isAwake())
            continue;
        affected->addForce(force);
    }
}

bool ConstantForceGenerator::isAffected(const std::shared_ptr<RigidBody> body)
{
    return std::find(affectedBodies.begin(), affectedBodies.end(), body) != affectedBodies.end();
}

bool ConstantForceGenerator::affect(const std::shared_ptr<RigidBody> body)
{
    if (!body->isDynamic())
        return false;
    if (const auto it = std::find(affectedBodies.begin(), affectedBodies.end(), body); it == affectedBodies.end())
    {
        affectedBodies.push_back(body);
        return true;
    }
    return false;
}

bool ConstantForceGenerator::release(const std::shared_ptr<RigidBody> body)
{
    if (const auto it = std::find(affectedBodies.begin(), affectedBodies.end(), body); it != affectedBodies.end())
    {
        affectedBodies.erase(it);
        return true;
    }
    return false;
}

// ExplosionGenerator implementation
ExplosionGenerator::ExplosionGenerator(const Vector2D &center, const float force, const float radius, const float duration)
    : center(center), maxForce(force), radius(radius), duration(duration), currentTime(0.0f)
{
}

void ExplosionGenerator::applyForce()
{
    if (affectedBodies.empty() || hasExpired())
        return;

    for (const auto &affected : affectedBodies)
    {
        if (!affected->isAwake())
            continue;

        Vector2D direction = affected->getPosition() - center;
        const float distance = direction.magnitude();

        if (distance > radius || distance <= 0.0f)
        {
            continue;
        }

        // calculate force falloff with distance and time
        const float distanceFalloff = 1.0f - (distance / radius);
        const float timeFalloff = 1.0f - (currentTime / duration);
        const float forceMagnitude = maxForce * distanceFalloff * timeFalloff;

        Vector2D explosionForce = direction.normalized() * forceMagnitude;
        affected->addForce(explosionForce);
    }
}

bool ExplosionGenerator::isAffected(const std::shared_ptr<RigidBody> body)
{
    return find(affectedBodies.begin(), affectedBodies.end(), body) != affectedBodies.end();
}

bool ExplosionGenerator::affect(const std::shared_ptr<RigidBody> body)
{
    if (!body->isDynamic() || hasExpired())
        return false;
    affectedBodies.push_back(body);
    return true;
}

bool ExplosionGenerator::release(const std::shared_ptr<RigidBody> body)
{
    const auto it = std::find(affectedBodies.begin(), affectedBodies.end(), body);
    if (it == affectedBodies.end())
        return false;
    affectedBodies.erase(it);
    return true;
}

void ExplosionGenerator::updateTime(const float timeDelta)
{
    currentTime += timeDelta;
}

bool ExplosionGenerator::hasExpired() const
{
    return currentTime >= duration;
}