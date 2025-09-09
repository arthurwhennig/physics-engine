//
//  AABB.h
//  physics-engine
//
//  Created by Arthur Hennig on 06.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_AABB_H
#define PHYSICSENGINE_AABB_H
#endif //PHYSICSENGINE_AABB_H

#include <Vector2D.h>

/**
 * @brief Axis-Aligned Bounding Box for spatial partitioning
 * 
 * Represents a rectangular bounding box aligned with coordinate axes.
 * Used for efficient collision detection broad-phase and spatial queries.
 */
class AABB {
public:
    // constructors
    AABB();
    AABB(const Vector2D& min, const Vector2D& max);
    AABB(float minX, float minY, float maxX, float maxY);

    // returns the center position in the middle of min and max
    [[nodiscard]] Vector2D getCenter() const;

    // getter and setter for the position vector (used for rotation)

    // returns a copy of the position vector
    [[nodiscard]] Vector2D getPosition() const;
    // sets the position vector to the new position (if it is contained within min and max)
    void setPosition(Vector2D* newPosition);

    // getters and setters of min and max

    // returns the minimum vector (bottom left corner)
    [[nodiscard]] Vector2D getMinimum() const;
    // sets the minimum vector (if it is at most max)
    void setMinimum(const Vector2D& minimum);
    // returns the minimum vector (bottom left corner)
    [[nodiscard]] Vector2D getMaximum() const;
    // sets the maximum vector (if it is at least min)
    void setMaximum(const Vector2D& maximum);

    // properties
    [[nodiscard]] float width() const;
    [[nodiscard]] float height() const;
    [[nodiscard]] Vector2D size() const;
    [[nodiscard]] float area() const;

    // returns true if this and the other AABB intersect
    [[nodiscard]] bool intersects(const AABB& other) const;

    // returns true if this AABB contains the given point
    [[nodiscard]] bool contains(const Vector2D& point) const;

    // returns true if this AABB contains the given AABB
    [[nodiscard]] bool contains(const AABB& other) const;

    // move the AABB by the given translation vector
    void translate(const Vector2D& translation);

    // rotate the AABB around its center point by the given number of radians
    void rotate(float radians);

    // expand AABB to include another AABB
    void expand(const AABB& other);

    // expand AABB to include a point
    void expand(const Vector2D& point);

    // expand by a margin
    void expand(float margin);

    // expand by a margin in the x and y direction
    void expand(float x, float y);

    // check if AABB is valid (min <= max and contains its center point)
    [[nodiscard]] bool isValid() const;

    // create AABB from center point with given half-extents
    static AABB fromCenter(Vector2D& centerPos, const Vector2D& halfExtents);

    // create AABB from center point with given radius (for circles)
    static AABB fromCenter(Vector2D& centerPos, float radius);

    // create union of two AABBs
    static AABB unionOf(const AABB& a, const AABB& b);

    // create intersection of two AABBs
    static AABB intersection(const AABB& a, const AABB& b);

    // stream operators for debugging
    friend std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
        os << "AABB(min: " << aabb.min << ", max: " << aabb.max << ")";
        return os;
    }

private:
    Vector2D* position;   // center position of the rigid body
    Vector2D min;       // bottom-left corner
    Vector2D max;       // top-right corner
};
