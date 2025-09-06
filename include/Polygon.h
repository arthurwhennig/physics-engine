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

#include <Vector2D.h>

class Projection {
public:
    float min;
    float max;

    // constructor
    explicit Projection(float min, float max);

    // destructor
    ~Projection() = default;

    // returns true if the projection overlaps with the other
    [[nodiscard]] bool overlaps(const Projection &other) const;
};

class Polygon {
public:
    // constructors
    Polygon();
    explicit Polygon(Vector2D* center);
    explicit Polygon(Vector2D* center, const std::vector<Vector2D> &points);

    // destructor
    ~Polygon();

    // check if the other polygon overlaps with this one using SAT
    [[nodiscard]] bool overlaps(const Polygon &other) const;

    // returns the indices of the edges of this polygon that overlap with the other polygon
    [[nodiscard]] std::vector<int> overlap(const Polygon &other) const;

    // point from other polygon which is closest to 'center'
    [[nodiscard]] Vector2D closest(const Polygon &other) const;

    // rotates the polygon point around 'center' by the given number of degrees
    void rotate(float degrees);

    // check if the given point is contained within the polygon
    [[nodiscard]] bool contains(const Vector2D &point) const;

    // project the polygon onto the given axis
    [[nodiscard]] Projection projectionOnEdge(const Vector2D &edge) const;
    // clear all edges (and their normals) and all points
    void clear();

    // add a single point to the polygon
    void addPoint(const Vector2D &point);
    // add a list of points (relative to the center)
    void addPoints(const std::vector<Vector2D> &pointsList);
    // compute the vectors between points (edges) and their normals
    void computeEdges();
    
    // accessors
    [[nodiscard]] const std::vector<Vector2D>& getPoints() const { return points; }
    [[nodiscard]] const std::vector<Vector2D>& getEdges() const { return edges; }
    [[nodiscard]] const std::vector<Vector2D>& getEdgeNormals() const { return normals; }
    [[nodiscard]] const Vector2D* getCenter() const { return center; }
    [[nodiscard]] size_t size() const { return points.size(); }

    // retrieves the vector of the point at 'index' (relative to 'center')
    [[nodiscard]] Vector2D getPoint(size_t index) const;
    // retrieves the world-space vector of the point at 'index'
    [[nodiscard]] Vector2D getWorldPoint(size_t index) const;
    // retrieves the edge between points at 'index' and 'index'+1 (mod number of points)
    [[nodiscard]] Vector2D getEdge(size_t index) const;
    // retrieves the edge normal between points at 'index' and 'index'+1 (mod number of points)
    [[nodiscard]] Vector2D getEdgeNormal(size_t index) const;


private:
    Vector2D* center;
    std::vector<Vector2D> points;
    std::vector<Vector2D> edges;
    std::vector<Vector2D> normals;
    bool ownsCenter; // tracks if we need to delete center in destructor

    // reset all points
    void clearPoints();
    // reset all edges
    void clearEdges();
};