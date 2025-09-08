//
//  Circle.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 07.09.2025.
//

#include <Circle.h>
#include <Utility.h>
#include <Vector2D.h>

Circle::Circle() {
    center = new Vector2D(0,0);
    radius = DEFAULT_DIMENSION;
    area = 0;
    circumference = 0;
    computeArea();
    computeCircumference();
}

Circle::Circle(Vector2D* center, const float radius): center(center), area(0), circumference(0) {
    setRadius(radius);
    computeArea();
    computeCircumference();
};

// destructor
Circle::~Circle() = default;

float Circle::momentOfInertia(const float mass) const
{
    if (center == nullptr) return 0;
    return 0.5f * mass * radius * radius;
}

bool Circle::contains(const Vector2D& point) const
{
    if (center == nullptr) return false;
    const Vector2D distance = point - *center;
    return distance.magnitude() <= radius;
}

bool Circle::contains(const Circle& other) const
{
    if (center == nullptr || other.center == nullptr) return false;
    const Vector2D distance = *center - other.getCenter();
    return distance.magnitude() + other.getRadius() <= radius;
}

bool Circle::overlaps(const Circle& other) const
{
    if (center == nullptr || other.center == nullptr) return false;
    const Vector2D distance = *center - other.getCenter();
    return distance.magnitudeSquared() <= (radius + other.getRadius()) * (radius + other.getRadius());
}

Vector2D Circle::getCenter() const
{
    if (center == nullptr) return {};
    return *center;
}

void Circle::setCenter(Vector2D* newCenter)
{
    center = newCenter;
}

float Circle::getRadius() const
{
    return radius;
}

void Circle::setRadius(const float newRadius)
{
    radius = std::max(MIN_RADIUS, newRadius);
    computeArea();
    computeCircumference();
}

float Circle::getArea() const
{
    return area;
}

float Circle::getCircumference() const
{
    return circumference;
}

float Circle::getDiameter() const
{
    return radius * 2;
}

void Circle::clear()
{
    center = nullptr;
    radius = 0;
}

Vector2D Circle::getMinimum() const
{
    if (center == nullptr) return {};
    return {center->x - radius, center->y - radius};
}

Vector2D Circle::getMaximum() const
{
    if (center == nullptr) return {};
    return {center->x + radius, center->y + radius};
}

void Circle::computeArea() {
    if (center == nullptr) return;
    area = PI * radius * radius;
}

void Circle::computeCircumference() {
    if (center == nullptr) return;
    circumference = PI * 2 * radius;
}