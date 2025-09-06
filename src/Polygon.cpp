//
//  Polygon.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 05.09.2025.
//

#include <Polygon.h>
#include <Matrix2D.h>
#include "Utility.h"

Projection::Projection(const float min, const float max) : min(min), max(max) {};

// returns true if this (one-dimensional) projection and the other overlap
bool Projection::overlaps(const Projection &other) const
{
    return !(max < other.min || other.max < min);
}

// constructors
Polygon::Polygon()
{
    center = new Vector2D();
    points = std::vector<Vector2D>();
    ownsCenter = true; // we created the center, so we own it
}

Polygon::Polygon(Vector2D* center) : center(center)
{
    points = std::vector<Vector2D>();
    ownsCenter = false; // we don't own this center pointer
}

Polygon::Polygon(Vector2D* center, const std::vector<Vector2D> &points) : center(center) {
    addPoints(points);
    ownsCenter = false; // we don't own this center pointer
}

// destructor
Polygon::~Polygon() = default;


// rotate the points around 'center' by the given number of degrees
void Polygon::rotate(const float degrees)
{
    const float radians = degreesToRadians(degrees);
    const Matrix2D rotation_matrix(radians);
    for (size_t i = 0; i < points.size(); ++i)
    {
        Vector2D point = points[i];
        points[i] = rotation_matrix * point;
    }
}

// returns true if the given point is located within the polygon
bool Polygon::contains(const Vector2D &point) const
{
    const double x = point.x;
    const double y = point.y;

    bool inside = false;
    Vector2D curr = points[0];
    for (size_t i = 1; i <= points.size(); ++i)
    {
        const Vector2D next = points[i % points.size()];
        float curr_x = center->x + curr.x;
        float curr_y = center->y + curr.y;
        float next_x = center->x + next.x;
        float next_y = center->y + next.y;
        if (std::min(curr_y, next_y) < y && y <= std::max(curr_y, next_y)) { // in bounds of the y coordinates
            if (x <= std::max(curr_x, next_x)) { // in upper-bound of x coordinates
                // calculate the x-intersection of the line connecting the point to the edge
                const double x_intersect = (y-curr_y) * (next_x-curr_x) / (next_y-curr_y) + curr_x;
                if (curr_x == next_x || x <= x_intersect) {
                    inside = !inside; // flip the flag 'inside'
                }
            }
        }
        curr = next;
    }
    return inside;
}

// returns true if this polygon and the other overlap
bool Polygon::overlaps(const Polygon &other) const
{
    std::vector<Vector2D> allNormals = normals;
    allNormals.insert(allNormals.end(), other.normals.begin(), other.normals.end());

    for (Vector2D &normal : allNormals)
    {
        Projection first = projectionOnEdge(normal);
        Projection second = other.projectionOnEdge(normal);
        if (!first.overlaps(second)) return false; // SAT: if any axis separates, no collision
    }
    return true; // All axes overlap, collision detected
}

std::vector<int> Polygon::overlap(const Polygon &other) const
{
    std::vector<int> output {};



    return output;
}

// projects this polygon onto the given edge vector
Projection Polygon::projectionOnEdge(const Vector2D &edge) const
{
    float min = std::numeric_limits<float>::infinity();
    float max = -std::numeric_limits<float>::infinity();
    for (const Vector2D &point : points)
    {
        Vector2D vertex = *center + point;
        const float projection = edge.dot(vertex);
        if (projection < min) min = projection;
        if (projection > max) max = projection;
    }
    return Projection(min, max);
}

// adds a point (relative to center)
void Polygon::addPoint(const Vector2D &point)
{
    points.emplace_back(point);
}

// adds a list of points (relative to center)
void Polygon::addPoints(const std::vector<Vector2D> &pointsList)
{
    for (auto &point : pointsList)
    {
        points.emplace_back(point.x, point.y);
    }
}

// computes the edges between all consecutive points
void Polygon::computeEdges()
{
    edges.clear();
    for (size_t i = 0; i < points.size(); ++i)
    {
        Vector2D first = points[i];
        Vector2D second = points[(i+1) % points.size()];
        const Vector2D edge = second - first;  // Edge vector from first to second
        edges.emplace_back(edge);
        // Get perpendicular normal (rotate 90 degrees counterclockwise)
        Vector2D normal = edge.orthogonal();
        normal.normalize();
        normals.emplace_back(normal);  // Store edge normals for SAT
    }
}

Vector2D Polygon::getPoint(const size_t index) const
{
    if (index < points.size()) {
        return points[index];
    }
    return Vector2D::zero();
}

Vector2D Polygon::getWorldPoint(const size_t index) const
{
    if (index < points.size()) {
        return *center + points[index];
    }
    return Vector2D::zero();
}

Vector2D Polygon::getEdge(const size_t index) const
{
    if (index < edges.size()) {
        return edges[index];
    }
    return Vector2D::zero();
}

Vector2D Polygon::getEdgeNormal(const size_t index) const
{
    if (index < normals.size()) {
        return normals[index];
    }
    return Vector2D::zero();
}

void Polygon::clear()
{
    points.clear();
    edges.clear();
    normals.clear();
}

void Polygon::clearEdges()
{
    edges.clear();
    normals.clear();
}

void Polygon::clearPoints()
{
    points.clear();
}