//
//  RigidBody.hpp
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#include <memory>

#include "Vector2D.hpp"

#ifndef RigidBody_h
#define RigidBody_h
#endif /* RigidBody_h */

/**
 * @brief represents a rigid body in 2D physics simulation
 *
 * a rigid body has position, velocity, acceleration, mass, and other physical properties.
 * it can be static (immovable) or dynamic (affected by forces).
 */
class RigidBody
{
public:
    enum class BodyType
    {
        STATIC, // immovable objects (like walls)
        DYNAMIC // objects affected by forces and gravity
    };

private:
    // transform
    Vector2D position;
    float rotation; // rotation in radians

    // physics properties
    Vector2D velocity;
    Vector2D acceleration;
    float angularVelocity;
    float angularAcceleration;

    float mass;
    float inverseMass;     // 1/mass, or 0 for static bodies
    float momentOfInertia; // rotational inertia
    float inverseInertia;  // 1/momentOfInertia

    // material properties
    float restitution; // bounciness (0 = no bounce, 1 = perfect bounce)
    float friction;    // surface friction coefficient
    float drag;        // air resistance

    // forces and torques
    Vector2D forceAccumulator;
    float torqueAccumulator;

    BodyType bodyType;
    bool awake; // sleep optimization

    // shape properties (simple circle for now)
    float radius;

public:
    // constructors
    RigidBody();
    RigidBody(const Vector2D &pos, float mass = 1.0f, float radius = 1.0f);

    // destructor
    ~RigidBody();

    // transform methods
    const Vector2D &getPosition() const { return position; }
    void setPosition(const Vector2D &pos) { position = pos; }

    float getRotation() const { return rotation; }
    void setRotation(float rot) { rotation = rot; }

    // velocity methods
    const Vector2D &getVelocity() const { return velocity; }
    void setVelocity(const Vector2D &vel) { velocity = vel; }

    float getAngularVelocity() const { return angularVelocity; }
    void setAngularVelocity(float angVel) { angularVelocity = angVel; }

    // acceleration methods
    const Vector2D &getAcceleration() const { return acceleration; }
    void setAcceleration(const Vector2D &acc) { acceleration = acc; }

    // mass and inertia
    float getMass() const { return mass; }
    float getInverseMass() const { return inverseMass; }
    void setMass(float mass);

    float getMomentOfInertia() const { return momentOfInertia; }
    void setMomentOfInertia(float inertia);

    // material properties
    float getRestitution() const { return restitution; }
    void setRestitution(float rest) { restitution = std::max(0.0f, std::min(1.0f, rest)); }

    float getFriction() const { return friction; }
    void setFriction(float fric) { friction = std::max(0.0f, fric); }

    float getDrag() const { return drag; }
    void setDrag(float d) { drag = std::max(0.0f, d); }

    // body type
    BodyType getBodyType() const { return bodyType; }
    void setBodyType(BodyType type);

    bool isStatic() const { return bodyType == BodyType::STATIC; }
    bool isDynamic() const { return bodyType == BodyType::DYNAMIC; }

    // shape properties
    float getRadius() const { return radius; }
    void setRadius(float r) { radius = std::max(0.1f, r); }

    // sleep state
    bool isAwake() const { return awake; }
    void setAwake(bool a) { awake = a; }

    // force and impulse application
    void addForce(const Vector2D &force);
    void addForceAtPoint(const Vector2D &force, const Vector2D &point);
    void addTorque(float torque);

    void addImpulse(const Vector2D &impulse);
    void addImpulseAtPoint(const Vector2D &impulse, const Vector2D &point);

    // integration (called by physics world)
    void integrate(float deltaTime);
    void clearAccumulators();

    // utility methods
    Vector2D getPointVelocity(const Vector2D &point) const;
    bool hasFiniteMass() const { return inverseMass > 0.0f; }

    // debug information
    void printDebugInfo() const;
};
