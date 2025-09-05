//
//  Vector2D.h
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_VECTOR2D_H
#define PHYSICSENGINE_VECTOR2D_H
#endif //PHYSICSENGINE_VECTOR2D_H

/**
 * @brief 2D vector class for physics calculations
 *
 * provides basic vector operations needed for 2D physics simulation
 * including addition, subtraction, dot product, cross product, and normalization.
 */
class Vector2D
{
public:
    float x, y;

    // constructors
    Vector2D();
    Vector2D(float x, float y);
    Vector2D(const Vector2D &other);

    // assignment operator
    Vector2D &operator=(const Vector2D &other);

    // arithmetic operators
    Vector2D operator+(const Vector2D &other) const;
    Vector2D operator-(const Vector2D &other) const;
    Vector2D operator*(float scalar) const;
    Vector2D operator/(float scalar) const;

    // compound assignment operators
    Vector2D &operator+=(const Vector2D &other);
    Vector2D &operator-=(const Vector2D &other);
    Vector2D &operator*=(float scalar);
    Vector2D &operator/=(float scalar);

    // comparison operators
    bool operator==(const Vector2D &other) const;
    bool operator!=(const Vector2D &other) const;

    // unary operators
    Vector2D operator-() const;

    // vector operations
    [[nodiscard]] float magnitude() const;
    [[nodiscard]] float magnitudeSquared() const;
    [[nodiscard]] Vector2D normalized() const;
    [[nodiscard]] Vector2D orthogonal() const;
    void normalize();

    [[nodiscard]] float dot(const Vector2D &other) const;
    [[nodiscard]] float cross(const Vector2D &other) const; // returns scalar for 2D cross product

    [[nodiscard]] float distanceTo(const Vector2D &other) const;
    [[nodiscard]] float distanceSquaredTo(const Vector2D &other) const;

    // utility functions
    void setZero();
    [[nodiscard]] bool isZero() const;
    [[nodiscard]] bool isZero(float epsilon = 1e-6f) const;

    // static utility functions
    static Vector2D zero();
    static Vector2D up();
    static Vector2D down();
    static Vector2D left();
    static Vector2D right();

    // stream operators
    friend std::ostream &operator<<(std::ostream &os, const Vector2D &vec);
    friend std::istream &operator>>(std::istream &is, Vector2D &vec);
};

// global operator for scalar-first multiplication
Vector2D operator*(float scalar, const Vector2D &vector);
