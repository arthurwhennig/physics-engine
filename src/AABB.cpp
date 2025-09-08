//
//  AABB.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 08.09.2025.
//

#include <AABB.h>

#include "Matrix2D.h"

AABB::AABB(): min(0, 0), max(0, 0)
{
    center = new Vector2D();
}

AABB::AABB(const float minX, const float minY, const float maxX, const float maxY): min(minX, minY), max(maxX, maxY)
{
    center = new Vector2D(minX + (maxX-minX)/2, minY + (maxY-minY)/2);
}

AABB::AABB(const Vector2D& min, const Vector2D& max): min(min), max(max)
{
    center = new Vector2D(min.x + (max.x-min.x)/2, min.y + (max.y-min.y)/2);
}

Vector2D AABB::getCenter() const
{
    return *center;
}

void AABB::setCenter(Vector2D *newCenter)
{
    if (contains(*newCenter)) center = newCenter;
}

float AABB::width() const
{
    return max.x - min.x;
}

float AABB::height() const
{
    return max.y - min.y;
}

Vector2D AABB::size() const
{
    return max - min;
}

float AABB::area() const
{
    return width() * height();
}

bool AABB::intersects(const AABB& other) const
{
    return !(max.x < other.min.x || min.x > other.max.x ||
                 max.y < other.min.y || min.y > other.max.y);
}

bool AABB::contains(const Vector2D& point) const
{
    return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y;
}

bool AABB::contains(const AABB& other) const
{
    return contains(other.min) && contains(other.max);
}

void AABB::translate(const Vector2D& translation)
{
    min += translation;
    max += translation;
    *center += translation;
}

void AABB::rotate(const float radians)
{
    const auto rotation = Matrix2D(radians);
    const Vector2D minRel = (min-*center);
    const Vector2D maxRel = (max-*center);
    min = *center + (rotation * minRel);
    max = *center + (rotation * maxRel);
}

void AABB::expand(const AABB& other)
{
    min.x = std::min(min.x, other.min.x);
    min.y = std::min(min.y, other.min.y);
    max.x = std::max(max.x, other.max.x);
    max.y = std::max(max.y, other.max.y);
}

void AABB::expand(const Vector2D& point)
{
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
}

void AABB::expand(const float margin)
{
    min.x -= margin;
    min.y -= margin;
    max.x += margin;
    max.y += margin;
}

void AABB::expand(const float x, const float y) {
    min.x -= x;
    min.y -= y;
    max.x += x;
    max.y += y;
}

bool AABB::isValid() const
{
    return contains(*center) && min.x <= max.x && min.y <= max.y;
}

AABB AABB::fromCenter(Vector2D& centerPos, const Vector2D& halfExtents)
{
    auto aabb = AABB(centerPos - halfExtents, centerPos + halfExtents);
    aabb.center = &centerPos;
    return aabb;
}

AABB AABB::fromCenter(Vector2D& centerPos, const float radius)
{
    auto aabb = AABB(centerPos.x - radius, centerPos.y - radius, centerPos.x + radius, centerPos.y + radius);
    aabb.center = &centerPos;
    return aabb;
}

AABB AABB::unionOf(const AABB& a, const AABB& b)
{
    return {
        std::min(a.min.x, b.min.x),
        std::min(a.min.y, b.min.y),
        std::max(a.max.x, b.max.x),
        std::max(a.max.y, b.max.y)
    };
}

AABB AABB::intersection(const AABB& a, const AABB& b)
{
    return {
        std::max(a.min.x, b.min.x),
        std::max(a.min.y, b.min.y),
        std::min(a.max.x, b.max.x),
        std::min(a.max.y, b.max.y)
    };
}

