//
//  Polygon.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 05.09.2025.
//

#include <Polygon.h>

bool Projection::overlaps(const Projection &other) const
{
    return !(max < other.min || min > other.max);
}

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

bool Polygon::overlaps(const Polygon &other) const
{
    std::vector<Vector2D> allEdges = edges;
    allEdges.insert(allEdges.end(), other.edges.begin(), other.edges.end());

    for (Vector2D &edge : allEdges)
    {
        Projection first = projectionOnEdge(edge);
        Projection second = other.projectionOnEdge(edge);
        if (! first.overlaps(second)) return false;
    }
    return true;
}

Projection Polygon::projectionOnEdge(const Vector2D &edge) const
{
    float min = std::numeric_limits<float>::infinity();
    float max = -std::numeric_limits<float>::infinity();
    for (const Vector2D &point : points)
    {
        Vector2D vertex = *center + point;
        const double projection = edge.dot(vertex);
        if (projection < min) min = projection;
        if (projection > max) max = projection;
    }
    return Projection(min, max);
}

void Polygon::addPoint(const Vector2D &point)
{
    points.emplace_back(point);
}

void Polygon::addPoints(const std::vector<Vector2D> &pointsList)
{
    for (auto &point : pointsList)
    {
        points.emplace_back(Vector2D(point.x, point.y));
    }
    clearEdges();
    computeEdges();
}

void Polygon::computeEdges()
{
    for (size_t i = 0; i < edges.size(); ++i)
    {
        Vector2D first = edges[i];
        Vector2D second = edges[(i+1) % edges.size()];
        Vector2D edge = first - second;
        edges.emplace_back(edge);
    }
}

void Polygon::clear()
{
    points.clear();
    edges.clear();
}

void Polygon::clearEdges()
{
    edges.clear();
}

void Polygon::clearPoints()
{
    points.clear();
}