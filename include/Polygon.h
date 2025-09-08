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

#include <Circle.h>
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

    // checks if the other polygon overlaps with this one using SAT
    [[nodiscard]] bool overlaps(const Polygon &other) const;

    // checks if this polygon and the other circle overlap
    [[nodiscard]] bool overlaps(const Circle& other) const;

    // rotates the polygon point around 'center' by the given number of degrees
    void rotate(float radians);

    // compute the moment of inertia when trying to rotate this polygon around its center
    [[nodiscard]] float momentOfInertia(float mass) const;

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
    // compute the area of the polygon
    void computeArea();
    
    // accessors
    [[nodiscard]] const std::vector<Vector2D>& getPoints() const;
    [[nodiscard]] const std::vector<Vector2D>& getEdges() const;
    [[nodiscard]] const std::vector<Vector2D>& getEdgeNormals() const;
    [[nodiscard]] Vector2D getCenter() const;
    [[nodiscard]] size_t size() const;

    // retrieves the vector of the point at 'index' (relative to 'center')
    [[nodiscard]] Vector2D getPoint(size_t index) const;
    // retrieves the world-space vector of the point at 'index'
    [[nodiscard]] Vector2D getWorldPoint(size_t index) const;
    // retrieves the edge between points at 'index' and 'index'+1 (mod number of points)
    [[nodiscard]] Vector2D getEdge(size_t index) const;
    // retrieves the edge normal between points at 'index' and 'index'+1 (mod number of points)
    [[nodiscard]] Vector2D getEdgeNormal(size_t index) const;
    // retrieve the area of the polygon
    [[nodiscard]] float getArea() const;
    // retrieve the bounds of the polygon
    [[nodiscard]] Vector2D getMinimum() const;
    [[nodiscard]] Vector2D getMaximum() const;

private:
    Vector2D* center;                       // center position of the polygon
    std::vector<Vector2D> points;           // point coordinates relative to the center
    std::vector<Vector2D> edges;            // edges between consecutive points
    std::vector<Vector2D> normals;          // normalized orthogonal vectors of the edges, pointing inside

    // metadata
    float area{};
    Vector2D min;
    Vector2D max;

    // reset all points
    void clearPoints();
    // reset all edges
    void clearEdges();
    // update min and max if necessary
    void updateBounds(const Vector2D& point);
};