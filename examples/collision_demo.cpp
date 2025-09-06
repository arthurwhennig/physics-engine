//
// polygon_collision_demo.cpp 
// Physics Engine Polygon Collision Demo
//
// Demonstrates polygon collision detection with contact manifold generation
//

#include <iostream>
#include <memory>
#include "PhysicsWorld.h"
#include "CollisionDetector.h"

int main() {
    std::cout << "=== Polygon Collision Detection Demo ===\n\n";
    
    // create two rectangular rigid bodies
    const auto boxA = std::make_shared<RigidBody>(Vector2D(0, 0), 1.0f);
    boxA->makeRectangle(4.0f, 2.0f);  // 4x2 rectangle
    boxA->setBodyType(RigidBody::BodyType::DYNAMIC);
    
    const auto boxB = std::make_shared<RigidBody>(Vector2D(2, 0), 1.0f);
    boxB->makeRectangle(2.0f, 3.0f);  // 2x3 rectangle  
    boxB->setBodyType(RigidBody::BodyType::DYNAMIC);
    
    // create a triangular rigid body
    const auto triangle = std::make_shared<RigidBody>(Vector2D(1, 4), 1.0f);
    triangle->makeTriangle(4.0f, 4.0f);
    triangle->setBodyType(RigidBody::BodyType::DYNAMIC);
    
    // create a circular rigid body for mixed collision testing
    const auto circle = std::make_shared<RigidBody>(Vector2D(-2, 1), 1.0f);
    circle->makeCircle(1.0);
    circle->setBodyType(RigidBody::BodyType::DYNAMIC);
    
    // test different collision scenarios
    std::cout << "Testing polygon collision detection...\n\n";
    
    // scenario 1: Rectangle vs Rectangle (overlapping)
    std::cout << "1. Rectangle A vs Rectangle B:\n";
    std::cout << "   Box A position: " << boxA->getPosition() << ", size: 4x2\n";
    std::cout << "   Box B position: " << boxB->getPosition() << ", size: 2x3\n";
    
    CollisionInfo collision1;
    collision1.bodyA = boxA;
    collision1.bodyB = boxB;
    
    if (CollisionDetector::testCollision(*boxA, *boxB, collision1)) {
        std::cout << "   COLLISION DETECTED!\n";
        std::cout << "   Contact Normal: " << collision1.contactNormal << "\n";
        std::cout << "   Penetration Depth: " << collision1.getPenetrationDepth() << "\n";
        std::cout << "   Contact Points (" << collision1.contactCount << "): ";
        for (int i = 0; i < collision1.contactCount; ++i) {
            std::cout << collision1.contactPoints[i];
            if (i < collision1.contactCount - 1) std::cout << ", ";
        }
        std::cout << "\n\n";
    } else {
        std::cout << "   No collision detected.\n\n";
    }
    
    // scenario 2: Triangle vs Rectangle (move triangle down to intersect)
    triangle->setPosition(Vector2D(1, 0.5));
    std::cout << "2. Triangle vs Rectangle A:\n";
    std::cout << "   Triangle position: " << triangle->getPosition() << ", radius: 2.0\n";
    std::cout << "   Box A position: " << boxA->getPosition() << ", size: 4x2\n";
    
    CollisionInfo collision2;
    collision2.bodyA = triangle;
    collision2.bodyB = boxA;
    
    if (CollisionDetector::testCollision(*triangle, *boxA, collision2)) {
        std::cout << "   COLLISION DETECTED!\n";
        std::cout << "   Contact Normal: " << collision2.contactNormal << "\n";
        std::cout << "   Penetration Depth: " << collision2.getPenetrationDepth() << "\n";
        std::cout << "   Contact Points (" << collision2.contactCount << "): ";
        for (int i = 0; i < collision2.contactCount; ++i) {
            std::cout << collision2.contactPoints[i];
            if (i < collision2.contactCount - 1) std::cout << ", ";
        }
        std::cout << "\n\n";
    } else {
        std::cout << "   No collision detected.\n\n";
    }
    
    // scenario 3: Circle vs Rectangle (mixed shape collision)
    circle->setPosition(Vector2D(-2, 1));
    boxB->setPosition(Vector2D(-0.5f, 0));
    std::cout << "3. Circle vs Rectangle A:\n";
    std::cout << "   Circle position: " << circle->getPosition() << ", radius: 1\n";
    std::cout << "   Box A position: " << boxB->getPosition() << ", size: 3x2\n";
    
    CollisionInfo collision3;
    collision3.bodyA = circle;
    collision3.bodyB = boxB;
    
    if (CollisionDetector::testCollision(*circle, *boxB, collision3)) {
        std::cout << "   COLLISION DETECTED!\n";
        std::cout << "   Contact Normal: " << collision3.contactNormal << "\n";
        std::cout << "   Penetration Depth: " << collision3.getPenetrationDepth() << "\n";
        std::cout << "   Contact Points (" << collision3.contactCount << "): ";
        for (int i = 0; i < collision3.contactCount; ++i) {
            std::cout << collision3.contactPoints[i];
            if (i < collision3.contactCount - 1) std::cout << ", ";
        }
        std::cout << "\n\n";
    } else {
        std::cout << "   No collision detected.\n\n";
    }
    
    // show how to use contact points for collision response
    std::cout << "=== Contact Point Usage for Physics Response ===\n";
    if (collision1.contactCount > 0) {
        std::cout << "For Box A vs Box B collision:\n";
        for (int i = 0; i < collision1.contactCount; ++i) {
            Vector2D contactPoint = collision1.contactPoints[i];
            std::cout << "Contact point " << (i+1) << ": " << contactPoint << "\n";
            
            // Calculate relative velocity at contact point
            Vector2D velA = collision1.bodyA->getPointVelocity(contactPoint);
            Vector2D velB = collision1.bodyB->getPointVelocity(contactPoint);
            Vector2D relativeVelocity = velB - velA;
            
            std::cout << "  Relative velocity at contact: " << relativeVelocity << "\n";
            std::cout << "  Normal component: " << relativeVelocity.dot(collision1.contactNormal) << "\n";
        }
    }
    
    std::cout << "\n=== Demo Complete ===\n";
    std::cout << "The collision detection system now provides:\n";
    std::cout << "- Accurate contact points for polygon collisions\n";
    std::cout << "- Contact normals for proper collision response\n";
    std::cout << "- Penetration depths for position correction\n";
    std::cout << "- Support for mixed shape types (circle-polygon)\n";
    
    return 0;
}
