//
//  Matrix2D.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 05.09.2025.
//

#include <cmath>

#include <Matrix2D.h>
#include <Utility.h>
#include <Vector2D.h>

// constructors
Matrix2D::Matrix2D() : a(0.0f), b(0.0f), c(0.0f), d(0.0f) {}

Matrix2D::Matrix2D(const Matrix2D &other) = default;

Matrix2D::Matrix2D(const float a, float const b, const float c, const float d): a(a), b(b), c(c), d(d) {}

Matrix2D::Matrix2D(const float radians) {
    const float sinVal = sin(radians);
    const float cosVal = cos(radians);
    a = cosVal;
    b = -sinVal;
    c = sinVal;
    d = cosVal;
}

// assignment operator
Matrix2D &Matrix2D::operator=(const Matrix2D &other)
{
    if (this != &other)
    {
        a = other.a;
        b = other.b;
        c = other.c;
        d = other.d;
    }
    return *this;
}

// arithmetic operators
Matrix2D Matrix2D::operator+(const Matrix2D &other) const
{
    return {a + other.a, b + other.b, c + other.c, d + other.d};
}

Matrix2D Matrix2D::operator-(const Matrix2D &other) const
{
    return {a - other.a, b - other.b, c - other.c, d - other.d};
}

Matrix2D Matrix2D::operator*(const float scalar) const
{
    return {a * scalar, b * scalar, c * scalar, d * scalar};
}

Matrix2D Matrix2D::operator/(const float scalar) const
{
    if (scalar == 0.0f) return {0, 0, 0, 0};
    return {a / scalar, b / scalar, c / scalar, d / scalar};
}

Matrix2D &Matrix2D::operator+=(const Matrix2D &other)
{
    a += other.a;
    b += other.b;
    c += other.c;
    d += other.d;
    return *this;
}

Matrix2D &Matrix2D::operator-=(const Matrix2D &other)
{
    a -= other.a;
    b -= other.b;
    c -= other.c;
    d -= other.d;
    return *this;
}

Matrix2D &Matrix2D::operator*=(const float scalar)
{
    a *= scalar;
    b *= scalar;
    c *= scalar;
    d *= scalar;
    return *this;
}

Matrix2D &Matrix2D::operator/=(const float scalar)
{
    if (scalar == 0.0f) return *this;
    a /= scalar;
    b /= scalar;
    c /= scalar;
    d /= scalar;
    return *this;
}

// comparison operators
bool Matrix2D::operator==(const Matrix2D &other) const
{
    const bool firstRow = std::abs(a - other.a) < EPSILON && std::abs(b - other.b) < EPSILON;
    const bool secondRow = std::abs(c - other.c) < EPSILON && std::abs(d - other.d) < EPSILON;
    return firstRow && secondRow;
}

bool Matrix2D::operator!=(const Matrix2D &other) const
{
    return !(*this == other);
}

// unary operator
Matrix2D Matrix2D::operator-() const
{
    return {-a, -b, -c, -d};
}

float Matrix2D::det() const
{
    return a * d - c * b;
}

Matrix2D Matrix2D::inverse() const
{
    const Matrix2D inv(d, -b, -c, a);
    const float det = this->det();
    if (det == 0) return Matrix2D();
    return inv / det;
}

Matrix2D Matrix2D::transpose() const
{
    return {a, c, b, d};
}

Matrix2D Matrix2D::power(const int pow) const
{
    Matrix2D result = *this;
    for (int i = 0; i < pow; ++i) {
        result = result * *this;
    }
    return result;
}

bool Matrix2D::isZero() const
{
    return a == 0.0f && b == 0.0f && c == 0.0f && d == 0.0f;
}

bool Matrix2D::isZero(const float epsilon) const
{
    return std::abs(a) < epsilon && std::abs(b) < epsilon && std::abs(c) < epsilon && std::abs(d) < epsilon;
}

void Matrix2D::setZero() {
    a = 0.0f;
    b = 0.0f;
    c = 0.0f;
    d = 0.0f;
}

// matrix-vector multiplication
Vector2D operator*(const Matrix2D &matrix, const Vector2D &vector)
{
    float newX = matrix.a * vector.x + matrix.b * vector.y;
    float newY = matrix.c * vector.x + matrix.d * vector.y;
    return {newX, newY};
}

// matrix-matrix multiplication
Matrix2D operator*(const Matrix2D &first, const Matrix2D &second)
{
    float newA = first.a * second.a + first.b * second.c;
    float newB = first.a * second.b + first.b * second.d;
    float newC = first.c * second.a + first.d * second.c;
    float newD = first.c * second.b + first.d * second.d;
    return {newA, newB, newC, newD};
}
