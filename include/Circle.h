//
//  Circle.h
//  physics-engine
//
//  Created by Arthur Hennig on 07.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_CIRCLE_H
#define PHYSICSENGINE_CIRCLE_H
#endif //PHYSICSENGINE_CIRCLE_H

#include <Vector2D.h>

class Circle {
public:
    // constructors
    explicit Circle();
    explicit Circle(Vector2D* center, float radius = DEFAULT_DIMENSION);

    // destructor
    ~Circle();

    // utility methods

    // returns the moment of inertia, assuming the circle has (evenly distributed) given mass and rotates around center
    [[nodiscard]] float momentOfInertia(float mass) const;

    // checks if the given point is contained within this circle
    [[nodiscard]] bool contains(const Vector2D& point) const;

    // checks if the given circle is contained within this circle
    [[nodiscard]] bool contains(const Circle &other) const;

    // checks if the given circle and this circle overlap
    [[nodiscard]] bool overlaps(const Circle &other) const;

    // returns the maximum position vector such that the entire circle is at least as big as that vector
    [[nodiscard]] Vector2D getMinimum() const;
    // returns the minimum position vector such that the entire circle is at most as big as that vector
    [[nodiscard]] Vector2D getMaximum() const;

    // getters and setters
    [[nodiscard]] Vector2D getCenter() const;
    void setCenter(Vector2D *newCenter);
    [[nodiscard]] float getRadius() const;
    void setRadius(float newRadius);
    [[nodiscard]] float getArea() const;
    [[nodiscard]] float getCircumference() const;
    [[nodiscard]] float getDiameter() const;

    // clear
    void clear();

private:
    Vector2D* center;
    float radius{};

    // metadata
    float area;
    float circumference;

    // metadata computation
    void computeArea();
    void computeCircumference();
};