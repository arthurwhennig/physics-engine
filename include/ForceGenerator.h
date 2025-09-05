//
//  ForceGenerator.h
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#pragma once

#include <memory>
#include <vector>

#include "RigidBody.h"
#include "Vector2D.h"

#ifndef PHYSICSENGINE_FORCEGENERATOR_H
#define PHYSICSENGINE_FORCEGENERATOR_H
#endif //PHYSICSENGINE_FORCEGENERATOR_H

/**
 * @brief base class for force generators
 *
 * force generators apply forces to rigid bodies. This can include
 * gravity, springs, drag, and custom forces.
 */
class ForceGenerator
{
public:
    virtual ~ForceGenerator() = default;

    // apply force to the affected rigid bodies
    virtual void applyForce() = 0;

    // add the given body to the affected bodies
    virtual bool affect(std::shared_ptr<RigidBody> body) = 0;

    // remove the given body from the affected bodies
    virtual bool release(std::shared_ptr<RigidBody> body) = 0;

    // check if this generator affects the given body
    virtual bool isAffected(std::shared_ptr<RigidBody> body) = 0;

    // update the current time by the given time delta (only overridden by timed generators)
    virtual void updateTime(float timeDelta) = 0;

    // check if this generator has expired (only overridden by timed generators)
    [[nodiscard]] virtual bool hasExpired() const { return false; };

    // clear all affected bodies
    virtual void clearAffected() = 0;
};

/**
 * @brief gravity force generator
 *
 * applies a constant gravitational force to all dynamic bodies.
 */
class GravityGenerator final : public ForceGenerator
{
private:
    std::vector<std::shared_ptr<RigidBody>> affectedBodies;
    Vector2D gravity;

public:
    explicit GravityGenerator(const Vector2D &g) : gravity(g) {}

    void applyForce() override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    void clearAffected() override { affectedBodies.clear(); };
    void updateTime(float timeDelta) override {};

    [[nodiscard]] const Vector2D &
    getGravity() const
    {
        return gravity;
    }
    void setGravity(const Vector2D &g) { gravity = g; }
};

/**
 * @brief drag force generator
 *
 * applies drag force proportional to velocity (linear drag) and
 * velocity squared (quadratic drag).
 */
class DragGenerator final : public ForceGenerator
{
private:
    std::vector<std::shared_ptr<RigidBody>> affectedBodies;
    float k1; // linear drag coefficient
    float k2; // quadratic drag coefficient

public:
    DragGenerator(const float linear, const float quadratic) : k1(linear), k2(quadratic) {}

    void applyForce() override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    void clearAffected() override { affectedBodies.clear(); };
    void updateTime(float timeDelta) override {};

    [[nodiscard]] float getLinearDrag() const { return k1; }
    void setLinearDrag(const float drag) { k1 = drag; }

    [[nodiscard]] float getQuadraticDrag() const { return k2; }
    void setQuadraticDrag(const float drag) { k2 = drag; }
};

/**
 * @brief spring force generator
 *
 * creates a spring connection between two rigid bodies or
 * between a rigid body and a fixed point in world space.
 */
class SpringGenerator final : public ForceGenerator
{
private:
    std::vector<std::shared_ptr<RigidBody>> affectedBodies;
    Vector2D fixedPoint;  // used when affectedBodies only contains one element
    Vector2D localPointA; // attachment point on bodyA (local coords)
    Vector2D localPointB; // attachment point on bodyB (local coords)

    float springConstant; // spring stiffness
    float restLength;     // natural length of spring
    float damping;        // damping coefficient
    
public:
    // spring between two bodies
    SpringGenerator(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b,
                    const Vector2D &localA, const Vector2D &localB,
                    float k, float restLen, float damp = 0.0f);

    // spring to fixed point
    SpringGenerator(const std::shared_ptr<RigidBody>& body, const Vector2D &worldPoint,
                    const Vector2D &localPoint, float k, float restLen, float damp = 0.0f);

    void applyForce() override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    void clearAffected() override { affectedBodies.clear(); };
    void updateTime(float timeDelta) override {};

    // getters and setters
    [[nodiscard]] float getSpringConstant() const { return springConstant; }
    void setSpringConstant(const float k) { springConstant = k; }

    [[nodiscard]] float getRestLength() const { return restLength; }
    void setRestLength(const float length) { restLength = length; }

    [[nodiscard]] float getDamping() const { return damping; }
    void setDamping(const float damp) { damping = damp; }

private:
    static Vector2D getWorldPoint(const std::shared_ptr<RigidBody>& body, const Vector2D &localPoint) ;
};

/**
 * @brief constant force generator
 *
 * applies a constant force to specific bodies.
 */
class ConstantForceGenerator final : public ForceGenerator
{
private:
    std::vector<std::shared_ptr<RigidBody>> affectedBodies;
    Vector2D force;

public:
    explicit ConstantForceGenerator(const Vector2D &f) : force(f) {}

    void applyForce() override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    void clearAffected() override { affectedBodies.clear(); };
    void updateTime(float timeDelta) override {};

    [[nodiscard]] const Vector2D &getForce() const { return force; }
    void setForce(const Vector2D &f) { force = f; }
};

/**
 * @brief explosion force generator
 *
 * applies an explosion force that decreases with distance from a center point.
 * this is typically used as a one-time effect.
 */
class ExplosionGenerator final : public ForceGenerator
{
private:
    std::vector<std::shared_ptr<RigidBody>> affectedBodies;
    Vector2D center;
    float maxForce;
    float radius;
    float duration;
    float currentTime;

public:
    ExplosionGenerator(const Vector2D &center, float force, float radius, float duration = 0.1f);

    void applyForce() override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    void updateTime(float timeDelta) override;
    [[nodiscard]] bool hasExpired() const override;

    void clearAffected() override { affectedBodies.clear(); };
};
