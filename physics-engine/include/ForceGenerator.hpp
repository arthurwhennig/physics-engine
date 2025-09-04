//
//  ForceGenerator.hpp
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#include <memory>
#include <vector>

#include "RigidBody.hpp"
#include "Vector2D.hpp"

#ifndef ForceGenerator_h
#define ForceGenerator_h
#endif /* ForceGenerator_h */

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

    // update the current time by the given time delta (only overriden by timed generators)
    virtual void updateTime(float timeDelta) = 0;

    // check if this generator has expired (only overriden by timed generators)
    virtual bool hasExpired() const { return false; };

    // clear all affected bodies
    virtual void clearAffected() = 0;
};

/**
 * @brief gravity force generator
 *
 * applies a constant gravitational force to all dynamic bodies.
 */
class GravityGenerator : public ForceGenerator
{
private:
    std::vector<std::shared_ptr<RigidBody>> affectedBodies;
    Vector2D gravity;

public:
    GravityGenerator(const Vector2D &g) : gravity(g) {}

    void applyForce() override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    void clearAffected() { affectedBodies.clear(); };
    void updateTime(float timeDelta) {};

    const Vector2D &
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
class DragGenerator : public ForceGenerator
{
private:
    std::vector<std::shared_ptr<RigidBody>> affectedBodies;
    float k1; // linear drag coefficient
    float k2; // quadratic drag coefficient

public:
    DragGenerator(float linear, float quadratic) : k1(linear), k2(quadratic) {}

    void applyForce() override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    void clearAffected() { affectedBodies.clear(); };
    void updateTime(float timeDelta) {};

    float getLinearDrag() const { return k1; }
    void setLinearDrag(float drag) { k1 = drag; }

    float getQuadraticDrag() const { return k2; }
    void setQuadraticDrag(float drag) { k2 = drag; }
};

/**
 * @brief spring force generator
 *
 * creates a spring connection between two rigid bodies or
 * between a rigid body and a fixed point in world space.
 */
class SpringGenerator : public ForceGenerator
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
    SpringGenerator(std::shared_ptr<RigidBody> a, std::shared_ptr<RigidBody> b,
                    const Vector2D &localA, const Vector2D &localB,
                    float k, float restLen, float damp = 0.0f);

    // spring to fixed point
    SpringGenerator(std::shared_ptr<RigidBody> body, const Vector2D &worldPoint,
                    const Vector2D &localPoint, float k, float restLen, float damp = 0.0f);

    void applyForce() override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    void clearAffected() { affectedBodies.clear(); };
    void updateTime(float timeDelta) {};

    // getters and setters
    float getSpringConstant() const { return springConstant; }
    void setSpringConstant(float k) { springConstant = k; }

    float getRestLength() const { return restLength; }
    void setRestLength(float length) { restLength = length; }

    float getDamping() const { return damping; }
    void setDamping(float damp) { damping = damp; }

private:
    Vector2D getWorldPoint(std::shared_ptr<RigidBody> body, const Vector2D &localPoint) const;
};

/**
 * @brief constant force generator
 *
 * applies a constant force to specific bodies.
 */
class ConstantForceGenerator : public ForceGenerator
{
private:
    std::vector<std::shared_ptr<RigidBody>> affectedBodies;
    Vector2D force;

public:
    ConstantForceGenerator(const Vector2D &f) : force(f) {}

    void applyForce() override;
    bool isAffected(std::shared_ptr<RigidBody> body) override;
    bool affect(std::shared_ptr<RigidBody> body) override;
    bool release(std::shared_ptr<RigidBody> body) override;
    void clearAffected() { affectedBodies.clear(); };
    void updateTime(float timeDelta) {};

    const Vector2D &getForce() const { return force; }
    void setForce(const Vector2D &f) { force = f; }
};

/**
 * @brief explosion force generator
 *
 * applies an explosion force that decreases with distance from a center point.
 * this is typically used as a one-time effect.
 */
class ExplosionGenerator : public ForceGenerator
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
    bool hasExpired() const override;

    void clearAffected() { affectedBodies.clear(); };
};
