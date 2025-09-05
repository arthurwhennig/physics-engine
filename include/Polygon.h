//
//  Polygon.h
//  physics-engine
//
//  Created by Arthur Hennig on 05.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_POLYGON_H
#define PHYSICSENGINE_POLYGON_H

#endif //PHYSICSENGINE_POLYGON_H

#include <vector>
#include "Vector2D.h"

class Projection {
public:
    float min;
    float max;

    explicit Projection(float min, float max) : min(min), max(max) {};

    bool overlaps(const Projection &other) const;
};

class Polygon {
public:
    explicit Polygon(Vector2D* center, const std::vector<Vector2D> &points) : center(center)
    {
        addPoints(points);
    }

    explicit Polygon(Vector2D* center) : center(center)
    {
        points = std::vector<Vector2D>();
    }

    Polygon()
    {
        Vector2D zero = Vector2D::zero();
        center = &zero;
        points = std::vector<Vector2D>();
    }

    // check if the other polygon overlaps with this one
    bool overlaps(const Polygon &other) const;
    // check if the given point is contained within the polygon
    bool contains(const Vector2D &point) const;

    // project the polygon onto the given axis
    Projection projectionOnEdge(const Vector2D &edge) const;
    // clear all edges and points
    void clear();

    // add a single point to the polygon
    void addPoint(const Vector2D &point);
    // add a list of points (relative to the center)
    void addPoints(const std::vector<Vector2D> &pointsList);
    // compute the vectors between points (edges)
    void computeEdges();

private:
    Vector2D* center;
    std::vector<Vector2D> points;
    std::vector<Vector2D> edges;

    // reset all points
    void clearPoints();
    // reset all edges
    void clearEdges();
};