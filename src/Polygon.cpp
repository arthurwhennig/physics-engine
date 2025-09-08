//
//  Polygon.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 05.09.2025.
//

#include <Circle.h>
#include <Matrix2D.h>
#include <Polygon.h>

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
    area = 0.0f;
    min = Vector2D(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    max = Vector2D(-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
}

Polygon::Polygon(Vector2D* center) : center(center)
{
    points = std::vector<Vector2D>();
    area = 0.0f;
}

Polygon::Polygon(Vector2D* center, const std::vector<Vector2D> &points) : center(center) {
    addPoints(points);
    computeEdges();
    computeArea();
}

// destructor
Polygon::~Polygon() = default;

// rotate the points around 'center' by the given number of degrees
void Polygon::rotate(const float radians)
{
    const Matrix2D rotation_matrix(radians);
    for (auto & i : points)
    {
        Vector2D point = i;
        i = rotation_matrix * point;
    }
}

// compute the moment of inertia around the z-axis going through the center point
float Polygon::momentOfInertia(const float mass) const
{
    const float density = mass / area;
    float sumX = 0;
    float sumY = 0;
    // calculate the sum of the moment of inertia for each triangle
    for (size_t i = 0; i < points.size(); i++)
    {
        const Vector2D first = points[i];
        Vector2D second;
        if (i == points.size()-1) second = points[0];
        else second = points[i+1];
        // triangle of first, second and center
        const float zComponent = first.cross(second);
        sumX += zComponent * (first.y*first.y + first.y * second.y + second.y * second.y);
        sumY += zComponent * (first.x*first.x + first.x * second.x + second.x * second.x);
    }
    return (density / 12) * (sumX + sumY);
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
bool Polygon::overlaps(const Polygon& other) const
{
    std::vector<Vector2D> allNormals = normals;
    allNormals.insert(allNormals.end(), other.normals.begin(), other.normals.end());

    for (Vector2D &normal : allNormals)
    {
        Projection first = projectionOnEdge(normal);
        Projection second = other.projectionOnEdge(normal);
        if (!first.overlaps(second)) return false; // SAT: if any axis separates the two, then there is no overlap
    }
    return true;
}

bool Polygon::overlaps(const Circle& other) const
{
    if (contains(other.getCenter())) return true;
    for (const Vector2D& point : points) {
        if (other.contains(point)) return true;
    }
    for (size_t i = 0; i < points.size(); i++) {
        const Vector2D first = getWorldPoint(i);
        const Vector2D circleCenter = other.getCenter();
        const float radius = other.getRadius();
        const Vector2D edge = getEdge(i);
        const Vector2D toCenter = circleCenter - first;
        const float coeff = edge.dot(toCenter) / (edge.dot(edge));
        Vector2D dist;
        if (coeff > 1.0f) { // check distance between second and circle center
            dist = getWorldPoint((i+1)%points.size()) - circleCenter;
        } else {
            dist = (first + coeff * edge) - circleCenter;
        }
        if (dist.magnitudeSquared() <= radius * radius) return true;
    }
    return false;
}

// projects this polygon onto the given edge vector
Projection Polygon::projectionOnEdge(const Vector2D &edge) const
{
    float minval = std::numeric_limits<float>::infinity();
    float maxval = -std::numeric_limits<float>::infinity();
    for (const Vector2D &point : points)
    {
        Vector2D vertex = *center + point;
        const float projection = edge.dot(vertex);
        if (projection < minval) minval = projection;
        if (projection > maxval) maxval = projection;
    }
    return Projection(minval, maxval);
}

// adds a point (relative to center)
void Polygon::addPoint(const Vector2D &point)
{
    updateBounds(point);
    points.emplace_back(point);
}

// adds a list of points (relative to center)
void Polygon::addPoints(const std::vector<Vector2D> &pointsList) {
    for (auto &point : pointsList)
    {
        updateBounds(point);
        points.emplace_back(point.x, point.y);
    }
}

void Polygon::computeEdges()
{
    edges.clear();
    for (size_t i = 0; i < points.size(); ++i)
    {
        Vector2D first = points[i];
        Vector2D second = points[(i+1) % points.size()];
        const Vector2D edge = second - first;  // edge vector from first to second
        edges.emplace_back(edge);
        // get perpendicular normal (rotate 90 degrees counterclockwise)
        Vector2D normal = edge.orthogonal();
        normal.normalize();
        normals.emplace_back(normal);  // store edge normals for SAT
    }
}

void Polygon::computeArea()
{
    float sum = 0;
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Vector2D first = points[i];
        Vector2D second;
        if (i == points.size() -1) second = points[0];
        else second = points[i+1];
        sum += (first.x * second.y - second.x * first.y);
    }
    area = sum / 2;
}

Vector2D Polygon::getPoint(const size_t index) const
{
    if (index < points.size()) {
        return points[index];
    }
    return Vector2D::zero();
}

const std::vector<Vector2D>& Polygon::getPoints() const
{
    return points;
}

const std::vector<Vector2D>& Polygon::getEdges() const
{
    return edges;
}

const std::vector<Vector2D>& Polygon::getEdgeNormals() const
{
    return normals;
}

Vector2D Polygon::getCenter() const
{
    return *center;
}

size_t Polygon::size() const
{
    return points.size();
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

float Polygon::getArea() const
{
    return area;
}

Vector2D Polygon::getMaximum() const
{
    return max;
}

Vector2D Polygon::getMinimum() const
{
    return min;
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

void Polygon::updateBounds(const Vector2D& point)
{
    if (point.x < min.x) min.x = point.x;
    if (point.y < min.y) min.y = point.y;
    if (point.x > max.x) max.x = point.x;
    if (point.y > max.y) max.y = point.y;
}