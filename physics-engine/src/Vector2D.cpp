//
//  Vector2D.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#include <cmath>
#include <algorithm>

#include "Vector2D.hpp"

// constructors
Vector2D::Vector2D() : x(0.0f), y(0.0f) {}

Vector2D::Vector2D(float x, float y) : x(x), y(y) {}

Vector2D::Vector2D(const Vector2D &other) : x(other.x), y(other.y) {}

// assignment operator
Vector2D &Vector2D::operator=(const Vector2D &other)
{
    if (this != &other)
    {
        x = other.x;
        y = other.y;
    }
    return *this;
}

// arithmetic operators
Vector2D Vector2D::operator+(const Vector2D &other) const
{
    return Vector2D(x + other.x, y + other.y);
}

Vector2D Vector2D::operator-(const Vector2D &other) const
{
    return Vector2D(x - other.x, y - other.y);
}

Vector2D Vector2D::operator*(float scalar) const
{
    return Vector2D(x * scalar, y * scalar);
}

Vector2D Vector2D::operator/(float scalar) const
{
    if (std::abs(scalar) < 1e-6f)
    {
        return Vector2D(0.0f, 0.0f); // avoid division by zero
    }
    return Vector2D(x / scalar, y / scalar);
}

// compound assignment operators
Vector2D &Vector2D::operator+=(const Vector2D &other)
{
    x += other.x;
    y += other.y;
    return *this;
}

Vector2D &Vector2D::operator-=(const Vector2D &other)
{
    x -= other.x;
    y -= other.y;
    return *this;
}

Vector2D &Vector2D::operator*=(float scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

Vector2D &Vector2D::operator/=(float scalar)
{
    if (std::abs(scalar) >= 1e-6f)
    {
        x /= scalar;
        y /= scalar;
    }
    else
    {
        x = y = 0.0f;
    }
    return *this;
}

// comparison operators
bool Vector2D::operator==(const Vector2D &other) const
{
    const float epsilon = 1e-6f;
    return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
}

bool Vector2D::operator!=(const Vector2D &other) const
{
    return !(*this == other);
}

// unary operators
Vector2D Vector2D::operator-() const
{
    return Vector2D(-x, -y);
}

// vector operations
float Vector2D::magnitude() const
{
    return std::sqrt(x * x + y * y);
}

float Vector2D::magnitudeSquared() const
{
    return x * x + y * y;
}

Vector2D Vector2D::normalized() const
{
    float mag = magnitude();
    if (mag < 1e-6f)
    {
        return Vector2D(0.0f, 0.0f);
    }
    return Vector2D(x / mag, y / mag);
}

void Vector2D::normalize()
{
    float mag = magnitude();
    if (mag >= 1e-6f)
    {
        x /= mag;
        y /= mag;
    }
    else
    {
        x = y = 0.0f;
    }
}

float Vector2D::dot(const Vector2D &other) const
{
    return x * other.x + y * other.y;
}

float Vector2D::cross(const Vector2D &other) const
{
    // 2D cross product returns scalar (z-component of 3D cross product)
    return x * other.y - y * other.x;
}

float Vector2D::distanceTo(const Vector2D &other) const
{
    float dx = x - other.x;
    float dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
}

float Vector2D::distanceSquaredTo(const Vector2D &other) const
{
    float dx = x - other.x;
    float dy = y - other.y;
    return dx * dx + dy * dy;
}

// utility functions
void Vector2D::setZero()
{
    x = y = 0.0f;
}

bool Vector2D::isZero(float epsilon) const
{
    return std::abs(x) < epsilon && std::abs(y) < epsilon;
}

// static utility functions
Vector2D Vector2D::zero()
{
    return Vector2D(0.0f, 0.0f);
}

Vector2D Vector2D::up()
{
    return Vector2D(0.0f, 1.0f);
}

Vector2D Vector2D::down()
{
    return Vector2D(0.0f, -1.0f);
}

Vector2D Vector2D::left()
{
    return Vector2D(-1.0f, 0.0f);
}

Vector2D Vector2D::right()
{
    return Vector2D(1.0f, 0.0f);
}

// stream operators
std::ostream &operator<<(std::ostream &os, const Vector2D &vec)
{
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
}

std::istream &operator>>(std::istream &is, Vector2D &vec)
{
    is >> vec.x >> vec.y;
    return is;
}

// global operator for scalar-first multiplication
Vector2D operator*(float scalar, const Vector2D &vector)
{
    return vector * scalar;
}
