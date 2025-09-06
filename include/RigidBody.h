//
//  RigidBody.h
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//
#pragma once

#ifndef PHYSICSENGINE_RIGIDBODY_H
#define PHYSICSENGINE_RIGIDBODY_H
#endif //PHYSICSENGINE_RIGIDBODY_H

#include <AABB.h>
#include <Polygon.h>
#include <Vector2D.h>

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
    enum class BodyShape { // standard shapes and polygons
        CIRCLE,
        POINT,
        LINE,
        POLYGON,
    };

private:
    // transform
    Vector2D* position;
    float rotation; // rotation in radians

    // physics properties
    Vector2D velocity;
    Vector2D acceleration;
    float angularVelocity;
    float angularAcceleration;

    // mass and inertia
    float mass;
    float inverseMass;      // 1/mass, or 0 for static bodies
    float momentOfInertia;  // rotational inertia
    float inverseInertia;   // 1/momentOfInertia

    // material properties
    float restitution;      // bounciness (0 = no bounce, 1 = perfect bounce)
    float friction;         // surface friction coefficient
    float drag;             // air resistance

    // forces and torques
    Vector2D forceAccumulator;
    float torqueAccumulator;

    // body metadata
    BodyShape bodyShape;
    BodyType bodyType;
    bool awake; // sleep optimization

    // shape properties
    float radius;
    Polygon polygon;    // relative to 'position'

public:
    // constructors
    RigidBody();
    explicit RigidBody(const Vector2D &pos);
    explicit RigidBody(const Vector2D &pos, float mass = 1.0f);

    // destructor
    ~RigidBody();

    bool makePolygon(const std::vector<Vector2D> &points);

    void makeCircle(float radius);

    void makePoint();

    void makeLine(const Vector2D &line);

    void makeTriangle(float width, float height);

    void makeRectangle(float width, float height);

    [[nodiscard]] bool contains(const Vector2D &point) const;

    [[nodiscard]] bool collidesWith(const RigidBody &other) const;

    void rotate(float degrees);

    // transform methods
    [[nodiscard]] const Vector2D &getPosition() const { return *position; }
    void setPosition(const Vector2D &pos) { position = new Vector2D(pos); }

    [[nodiscard]] float getRotation() const { return rotation; }
    void setRotation(const float rot) { rotation = rot; }

    // velocity methods
    [[nodiscard]] const Vector2D &getVelocity() const { return velocity; }
    void setVelocity(const Vector2D &vel) { velocity = vel; }

    [[nodiscard]] float getAngularVelocity() const { return angularVelocity; }
    void setAngularVelocity(const float angVel) { angularVelocity = angVel; }

    // acceleration methods
    [[nodiscard]] const Vector2D &getAcceleration() const { return acceleration; }
    void setAcceleration(const Vector2D &acc) { acceleration = acc; }

    // mass and inertia
    [[nodiscard]] float getMass() const { return mass; }
    [[nodiscard]] float getInverseMass() const { return inverseMass; }
    void setMass(float m);

    [[nodiscard]] float getMomentOfInertia() const { return momentOfInertia; }
    void setMomentOfInertia(float inertia);

    // material properties
    [[nodiscard]] float getRestitution() const { return restitution; }
    void setRestitution(const float rest) { restitution = std::max(0.0f, std::min(1.0f, rest)); }

    [[nodiscard]] float getFriction() const { return friction; }
    void setFriction(const float fric) { friction = std::max(0.0f, fric); }

    [[nodiscard]] float getDrag() const { return drag; }
    void setDrag(const float d) { drag = std::max(0.0f, d); }

    // body type
    [[nodiscard]] BodyType getBodyType() const { return bodyType; }
    void setBodyType(BodyType type);

    [[nodiscard]] bool isStatic() const { return bodyType == BodyType::STATIC; }
    [[nodiscard]] bool isDynamic() const { return bodyType == BodyType::DYNAMIC; }

    // shape properties
    [[nodiscard]] float getRadius() const { return radius; }
    void setRadius(const float r) { radius = std::max(0.1f, r); }
    [[nodiscard]] const Polygon& getPolygon() const { return polygon; }
    [[nodiscard]] BodyShape getBodyShape() const { return bodyShape; }

    // sleep state
    [[nodiscard]] bool isAwake() const { return awake; }
    void setAwake(const bool a) { awake = a; }

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
    [[nodiscard]] Vector2D getPointVelocity(const Vector2D &point) const;
    [[nodiscard]] bool hasFiniteMass() const { return inverseMass > 0.0f; }
    
    // spatial partitioning support
    [[nodiscard]] AABB getAABB() const;
    [[nodiscard]] AABB getAABB(float margin) const;

    // debug information
    void printDebugInfo() const;
};
