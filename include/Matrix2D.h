//
//  Matrix2D.h
//  physics-engine
//
//  Created by Arthur Hennig on 05.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_MATRIX2D_H
#define PHYSICSENGINE_MATRIX2D_H
#endif //PHYSICSENGINE_MATRIX2D_H

#include <Vector2D.h>
#include <Utility.h>

class Matrix2D
{
public:
    float a, b, c, d;

    // constructors
    Matrix2D();                                     // zero matrix
    Matrix2D(float a, float b, float c, float d);   // new matrix with rows [a,b] and [c,d]
    Matrix2D(const Matrix2D& other);                // copy
    explicit Matrix2D(float radians);               // rotation matrix around (0,0)

    // assignment operator
    Matrix2D &operator=(const Matrix2D &other);

    // arithmetic operators
    Matrix2D operator+(const Matrix2D &other) const;
    Matrix2D operator-(const Matrix2D &other) const;
    Matrix2D operator*(float scalar) const;
    Matrix2D operator/(float scalar) const;

    // compound assignment operators
    Matrix2D &operator+=(const Matrix2D &other);
    Matrix2D &operator-=(const Matrix2D &other);
    Matrix2D &operator*=(float scalar);
    Matrix2D &operator/=(float scalar);

    // comparison operators
    bool operator==(const Matrix2D& other) const;
    bool operator!=(const Matrix2D& other) const;

    // unary operators
    Matrix2D operator-() const;

    // matrix operations
    [[nodiscard]] float det() const;                // determinant
    [[nodiscard]] Matrix2D inverse() const;         // inverse
    [[nodiscard]] Matrix2D transpose() const;       // transposed
    [[nodiscard]] Matrix2D power(int pow) const;    // raise to power

    // utility functions
    void setZero();
    [[nodiscard]] bool isZero() const;
    [[nodiscard]] bool isZero(float epsilon = EPSILON) const;

    // static utility functions
    static Matrix2D zero() {
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }
    static Matrix2D identity() {
        return {1.0f, 0.0f, 0.0f, 1.0f};
    }
};

// global operators
// matrix-vector multiplication
Vector2D operator*(const Matrix2D& matrix, const Vector2D& vector);
// matrix-matrix multiplication
Matrix2D operator*(const Matrix2D& first, const Matrix2D& second);