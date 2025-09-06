//
//  CollisionDetector.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 04.09.2025.
//

#include <iostream>
#include <cmath>
#include <algorithm>

#include <CollisionDetector.h>
#include <Quadtree.h>

CollisionDetector::CollisionDetector(bool enableQuadtree)
    : useQuadtree(enableQuadtree), worldBounds(-1000, -1000, 1000, 1000)
{
    if (useQuadtree) {
        quadtree = std::make_unique<Quadtree>(worldBounds);
    }
}

CollisionDetector::CollisionDetector(const AABB& worldBounds, bool enableQuadtree)
    : useQuadtree(enableQuadtree), worldBounds(worldBounds)
{
    if (useQuadtree) {
        quadtree = std::make_unique<Quadtree>(worldBounds);
    }
}

CollisionDetector::~CollisionDetector() = default;

// main collision detection methods
void CollisionDetector::detectCollisions(const std::vector<std::shared_ptr<RigidBody>> &bodies)
{
    collisions.clear();
    
    if (useQuadtree && quadtree) {
        // use quadtree for efficient broad-phase collision detection
        quadtree->build(bodies);
        auto potentialPairs = quadtree->getPotentialPairs();
        
        // perform narrow-phase collision detection on potential pairs
        for (const auto& [bodyA, bodyB] : potentialPairs) {
            CollisionInfo collision;
            collision.bodyA = bodyA;
            collision.bodyB = bodyB;
            
            if (testCollision(*bodyA, *bodyB, collision)) {
                collision.separatingVelocity = calculateSeparatingVelocity(collision);
                // only add collision if objects are moving towards each other
                if (collision.separatingVelocity < 0.0f) {
                    collisions.push_back(collision);
                }
            }
        }
    } else {
        // fallback to naive O(n²) collision detection
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                CollisionInfo collision;
                collision.bodyA = bodies[i];
                collision.bodyB = bodies[j];
                
                if (testCollision(*bodies[i], *bodies[j], collision)) {
                    collision.separatingVelocity = calculateSeparatingVelocity(collision);
                    // only add collision if objects are moving towards each other
                    if (collision.separatingVelocity < 0.0f) {
                        collisions.push_back(collision);
                    }
                }
            }
        }
    }
}
// collision resolution
void CollisionDetector::resolveCollisions() const {
    for (auto &collision : collisions)
    {
        resolveCollision(collision);
    }
}

void CollisionDetector::resolveCollision(const CollisionInfo &collision)
{
    // resolve velocity (bounce)
    resolveVelocity(collision);

    // resolve position (separate overlapping objects)
    resolvePosition(collision);
}

// collision test for two circles
bool CollisionDetector::testCircleCircle(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision)
{
    const Vector2D& centerA = bodyA.getPosition();
    const Vector2D& centerB = bodyB.getPosition();
    const float radiusA = bodyA.getRadius();
    const float radiusB = bodyB.getRadius();
    const Vector2D direction = centerB - centerA;
    const float distance = direction.magnitude();

    if (const float combinedRadius = radiusA + radiusB; distance < combinedRadius && distance > 0.0f)
    {
        // collision detected
        collision.contactNormal = direction.normalized();
        const float penetration_depth = combinedRadius - distance;
        collision.penetration = collision.contactNormal * penetration_depth;
        const Vector2D contactPoint = centerA + collision.contactNormal * radiusA;
        collision.addContactPoint(contactPoint);
        return true;
    }
    return false;
}

// collision test for a circle and a polygon
bool CollisionDetector::testCirclePolygon(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision)
{
    // determine which is circle and which is polygon
    const RigidBody* circle;
    const RigidBody* polygon;

    if (bodyA.getBodyShape() == RigidBody::BodyShape::CIRCLE) {
        circle = &bodyA;
        polygon = &bodyB;
    } else {
        circle = &bodyB;
        polygon = &bodyA;
    }

    const Vector2D& circleCenter = circle->getPosition();
    const float circleRadius = circle->getRadius();
    const Polygon& poly = polygon->getPolygon();

    // find the closest point on polygon to circle center
    float minDistance = std::numeric_limits<float>::infinity();
    Vector2D closestPoint;

    // check each edge of the polygon
    for (size_t i = 0; i < poly.size(); ++i) {
        // find the closest point on edge to circle center
        Vector2D vec = poly.getWorldPoint(i);
        Vector2D edge = poly.getEdge(i);
        Vector2D toCenter = circleCenter - vec;
        float coeff = toCenter.dot(edge) / edge.dot(edge);
        coeff = std::max(0.0f, std::min(1.0f, coeff));
        Vector2D pointOnEdge = vec + edge * coeff;
        const float distance = (pointOnEdge - circleCenter).magnitude();
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = pointOnEdge;
        }
    }

    // check if collision occurred
    if (minDistance >= circleRadius) {
        return false;
    }

    // fill collision info
    const Vector2D normal = (circleCenter - closestPoint).normalized();

    collision.contactNormal = normal;
    collision.penetration = -normal * (circleRadius - minDistance);
    collision.addContactPoint(closestPoint);

    return true;
}

// collision test for two polygons
bool CollisionDetector::testPolygonPolygon(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision)
{
    const Polygon& polyA = bodyA.getPolygon();
    const Polygon& polyB = bodyB.getPolygon();

    // perform SAT collision detection
    const SATResult result = performSAT(polyA, polyB);

    if (!result.collision) {
        return false;
    }

    // fill collision info
    collision.penetration = -result.mtv;
    collision.contactNormal = collision.penetration.normalized();

    // find contact points using clipping method
    findContactPoints(bodyA, bodyB, result, collision);

    return true;
}

// SAT implementation with contact manifold generation
CollisionDetector::SATResult CollisionDetector::performSAT(const Polygon& polyA, const Polygon& polyB)
{
    SATResult result;
    result.collision = true;
    result.penetration = std::numeric_limits<float>::infinity();

    // test all edge normals from both polygons
    const auto& normalsA = polyA.getEdgeNormals();
    const auto& normalsB = polyB.getEdgeNormals();

    // test normals from polygon A
    for (size_t i = 0; i < normalsA.size(); ++i) {
        const Vector2D& normal = normalsA[i];

        Projection projA = polyA.projectionOnEdge(normal);
        Projection projB = polyB.projectionOnEdge(normal);

        if (!projA.overlaps(projB)) {
            result.collision = false;
            result.referenceEdge = 0;
        }

        // calculate overlap
        const float overlap = std::min(projA.max - projB.min, projB.max - projA.min);
        if (overlap < result.penetration) {
            result.penetration = overlap;
            result.mtv = normal * overlap;
            result.referenceEdge = i;
        }
    }

    // test normals from polygon B
    for (size_t i = 0; i < normalsB.size(); ++i) {
        const Vector2D& edge = normalsB[i];

        Projection projA = polyA.projectionOnEdge(edge);
        Projection projB = polyB.projectionOnEdge(edge);

        if (!projA.overlaps(projB)) {
            result.collision = false;
            result.referenceEdge = 0;
            return result;
        }

        // calculate the overlap
        const float overlap = std::min(projA.max - projB.min, projB.max - projA.min);
        if (overlap < result.penetration) {
            result.penetration = overlap;
            result.mtv = edge * overlap;
            result.referenceEdge = i;
        }
    }

    // ensure MTV points from A to B
    const Vector2D centerA = *polyA.getCenter();
    const Vector2D centerB = *polyB.getCenter();
    const Vector2D centerDiff = centerB - centerA;

    if (result.mtv.dot(centerDiff) < 0) {
        result.mtv = -result.mtv;
    }
    return result;
}

void CollisionDetector::findContactPoints(const RigidBody& bodyA, const RigidBody& bodyB,
                                        const SATResult& satResult, CollisionInfo& collision)
{
    const Polygon& polyA = bodyA.getPolygon();
    const Polygon& polyB = bodyB.getPolygon();

    // determine reference and incident polygons
    const Polygon* refPoly = &polyB;
    const Polygon* incPoly = &polyA;
    const size_t refEdgeIndex = satResult.referenceEdge;

    // get reference edge
    const Vector2D refV1 = refPoly->getWorldPoint(refEdgeIndex);
    const Vector2D refV2 = refPoly->getWorldPoint((refEdgeIndex + 1) % refPoly->size());
    const Vector2D refEdge = refPoly->getEdge(refEdgeIndex);
    const Vector2D refNormal = satResult.mtv.normalized();

    // find most anti-parallel edge on incident polygon
    float maxDot = -std::numeric_limits<float>::infinity();
    size_t incEdgeIndex = 0;

    const auto& incNormals = incPoly->getEdgeNormals();
    for (size_t i = 0; i < incNormals.size(); ++i) {
        const float dot = incNormals[i].dot(-refNormal);
        if (dot > maxDot) {
            maxDot = dot;
            incEdgeIndex = i;
        }
    }

    // get incident edge vertices
    const Vector2D incV1 = incPoly->getWorldPoint(incEdgeIndex);
    const Vector2D incV2 = incPoly->getWorldPoint((incEdgeIndex + 1) % incPoly->size());

    // clip incident edge against reference edge side planes
    std::vector clippedPoints = {incV1, incV2};

    // clip against reference edge sides
    Vector2D refSideNormal(-refEdge.y, refEdge.x);
    refSideNormal.normalize();

    std::vector first_clipped = clipPolygonToLine(clippedPoints, refV1, refSideNormal);
    std::vector second_clipped = clipPolygonToLine(clippedPoints, refV2, refSideNormal);

    clippedPoints.insert(clippedPoints.end(), first_clipped.begin(), first_clipped.end());
    clippedPoints.insert(clippedPoints.end(), second_clipped.begin(), second_clipped.end());

    // keep points behind reference edge
    for (const Vector2D& point : clippedPoints) {
        const float penetration = (point - refV1).dot(refNormal);
        if (penetration <= 0.0f) {
            collision.addContactPoint(point);
        }
    }

    // ensure we have at least one contact point
    if (collision.contactCount == 0) {
        // fallback: use edge midpoints
        const Vector2D contactPoint = (refV1 + refV2) * 0.5f;
        collision.addContactPoint(contactPoint);
    }
}

std::vector<Vector2D> CollisionDetector::clipPolygonToLine(const std::vector<Vector2D>& polygon,
                                                         const Vector2D& linePoint, const Vector2D& lineNormal)
{
    std::vector<Vector2D> output;

    if (polygon.empty()) return output;

    Vector2D prevVertex = polygon.back();
    float prevDistance = (prevVertex - linePoint).dot(lineNormal);

    for (const Vector2D& currVertex : polygon) {
        const float currDistance = (currVertex - linePoint).dot(lineNormal);

        if (currDistance >= 0.0f) {
            // current vertex is in front of or on the line
            if (prevDistance < 0.0f) {
                // previous was behind, current is in front - add intersection
                const float t = prevDistance / (prevDistance - currDistance);
                Vector2D intersection = prevVertex + (currVertex - prevVertex) * t;
                output.push_back(intersection);
            }
            output.push_back(currVertex);
        }
        else if (prevDistance >= 0.0f) {
            // previous was in front, current is behind - add intersection
            const float t = prevDistance / (prevDistance - currDistance);
            Vector2D intersection = prevVertex + (currVertex - prevVertex) * t;
            output.push_back(intersection);
        }

        prevVertex = currVertex;
        prevDistance = currDistance;
    }

    return output;
}

float CollisionDetector::projectVertexOnAxis(const Vector2D& vertex, const Vector2D& axis)
{
    return vertex.dot(axis);
}

bool CollisionDetector::testCollision(const RigidBody &bodyA, const RigidBody &bodyB, CollisionInfo &collision) {
    // dispatch to appropriate collision test based on body shapes
    const RigidBody::BodyShape shapeA = bodyA.getBodyShape();
    const RigidBody::BodyShape shapeB = bodyB.getBodyShape();
    
    if (shapeA == RigidBody::BodyShape::CIRCLE && shapeB == RigidBody::BodyShape::CIRCLE) {
        return testCircleCircle(bodyA, bodyB, collision);
    }
    if (shapeA == RigidBody::BodyShape::POLYGON && shapeB == RigidBody::BodyShape::POLYGON) {
        return testPolygonPolygon(bodyA, bodyB, collision);
    }
    if ((shapeA == RigidBody::BodyShape::CIRCLE && shapeB == RigidBody::BodyShape::POLYGON) ||
             (shapeA == RigidBody::BodyShape::POLYGON && shapeB == RigidBody::BodyShape::CIRCLE)) {
        return testCirclePolygon(bodyA, bodyB, collision);
    }
    // unsupported shape combination
    return false;
}

// utility methods
Vector2D CollisionDetector::getContactPoint(const RigidBody &bodyA, const RigidBody &bodyB)
{
    const Vector2D direction = bodyB.getPosition() - bodyA.getPosition();
    if (const float distance = direction.magnitude(); distance > 0.0f)
    {
        const Vector2D normal = direction / distance;
        return bodyA.getPosition() + normal * bodyA.getRadius();
    }
    return bodyA.getPosition(); // fallback for zero distance
}

float CollisionDetector::calculateSeparatingVelocity(const CollisionInfo &collision)
{
    const Vector2D relativeVelocity = collision.bodyB->getVelocity() - collision.bodyA->getVelocity();
    return relativeVelocity.dot(collision.contactNormal);
}

// broad phase collision detection
std::vector<std::pair<size_t, size_t>> CollisionDetector::broadPhase(const std::vector<std::shared_ptr<RigidBody>> &bodies)
{
    // simple implementation - just return all pairs
    // in a real engine, this would use spatial partitioning (quad-tree, grid, etc.)
    std::vector<std::pair<size_t, size_t>> pairs;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        for (size_t j = i + 1; j < bodies.size(); ++j)
        {
            pairs.emplace_back(i, j);
        }
    }

    return pairs;
}

// debug information
void CollisionDetector::printCollisionInfo() const
{
    std::cout << "Active Collisions: " << collisions.size() << "\n";
    for (size_t i = 0; i < collisions.size(); ++i)
    {
        const auto &collision = collisions[i];
        std::cout << "Collision " << i << ":\n";
        std::cout << "  Contact Points (" << collision.contactCount << "): ";
        for (int j = 0; j < collision.contactCount; ++j) {
            std::cout << collision.contactPoints[j];
            if (j < collision.contactCount - 1) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "  Contact Normal: " << collision.contactNormal << "\n";
        std::cout << "  Penetration: " << collision.getPenetrationDepth() << "\n";
        std::cout << "  Separating Velocity: " << collision.separatingVelocity << "\n";
    }
}

// private helper methods for collision resolution
void CollisionDetector::resolveVelocity(const CollisionInfo &collision) {
    const float separatingVelocity = collision.separatingVelocity;
    // don't resolve if separating velocity is positive (moving apart)
    if (separatingVelocity > 0.0f)
        return;
    // calculate restitution
    const float restitution = calculateRestitution(collision);
    // calculate new separating velocity
    const float newSepVelocity = -separatingVelocity * restitution;
    // calculate change in velocity
    const float deltaVelocity = newSepVelocity - separatingVelocity;
    // calculate total inverse mass
    const float totalInverseMass = collision.bodyA->getInverseMass() + collision.bodyB->getInverseMass();
    // if both objects are static/infinite mass, no resolution needed
    if (totalInverseMass <= 0.0f)
        return;
    // calculate impulse per unit mass
    const float impulse = deltaVelocity / totalInverseMass;
    // calculate impulse vector
    const Vector2D impulseVector = collision.contactNormal * impulse;
    // apply impulse to both bodies
    if (collision.bodyA->hasFiniteMass())
    {
        collision.bodyA->addImpulse(-impulseVector);
    }
    if (collision.bodyB->hasFiniteMass())
    {
        collision.bodyB->addImpulse(impulseVector);
    }
}

void CollisionDetector::resolvePosition(const CollisionInfo &collision)
{
    // only resolve position if there's penetration
    if (collision.getPenetrationDepth() <= 0.0f)
        return;
    const float totalInverseMass = collision.bodyA->getInverseMass() + collision.bodyB->getInverseMass();
    // if both objects are static/infinite mass, no resolution needed
    if (totalInverseMass <= 0.0f)
        return;
    // calculate movement per unit of inverse mass
    const Vector2D movePerIMass = collision.contactNormal * (collision.getPenetrationDepth() / totalInverseMass);
    // calculate position adjustments
    const Vector2D bodyAMovement = movePerIMass * collision.bodyA->getInverseMass();
    const Vector2D bodyBMovement = movePerIMass * collision.bodyB->getInverseMass();
    // apply position corrections
    if (collision.bodyA->hasFiniteMass())
    {
        collision.bodyA->setPosition(collision.bodyA->getPosition() - bodyAMovement);
    }
    if (collision.bodyB->hasFiniteMass())
    {
        collision.bodyB->setPosition(collision.bodyB->getPosition() + bodyBMovement);
    }
}

float CollisionDetector::calculateRestitution(const CollisionInfo &collision) {
    // use the minimum restitution of the two bodies
    return std::min(collision.bodyA->getRestitution(), collision.bodyB->getRestitution());
}

float CollisionDetector::calculateFriction(const CollisionInfo &collision)
{
    // use the geometric mean of the friction coefficients
    return std::sqrt(collision.bodyA->getFriction() * collision.bodyB->getFriction());
}

// Quadtree control methods
void CollisionDetector::setWorldBounds(const AABB& bounds)
{
    worldBounds = bounds;
    if (useQuadtree) {
        quadtree = std::make_unique<Quadtree>(worldBounds);
    }
}

std::vector<AABB> CollisionDetector::getQuadtreeVisualization() const
{
    if (useQuadtree && quadtree) {
        return quadtree->getVisualizationBounds();
    }
    return {};
}

void CollisionDetector::printQuadtreeStats() const
{
    if (useQuadtree && quadtree) {
        auto stats = quadtree->getStats();
        std::cout << "Quadtree Statistics:\n";
        std::cout << "  Total Nodes: " << stats.totalNodes << "\n";
        std::cout << "  Leaf Nodes: " << stats.leafNodes << "\n";
        std::cout << "  Max Depth: " << stats.maxDepth << "\n";
        std::cout << "  Total Objects: " << stats.totalObjects << "\n";
        std::cout << "  Avg Objects/Leaf: " << stats.averageObjectsPerLeaf << "\n";
    } else {
        std::cout << "Quadtree not enabled or not initialized\n";
    }
}
