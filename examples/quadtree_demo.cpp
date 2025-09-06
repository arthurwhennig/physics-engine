//
// quadtree_demo.cpp
// Demonstrates the quadtree spatial partitioning system
//

#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include "PhysicsWorld.h"
#include "CollisionDetector.h"
#include "Quadtree.h"

int main() {
    std::cout << "=== Quadtree Spatial Partitioning Demo ===\n\n";
    
    // Create world bounds
    AABB worldBounds(-500, -500, 500, 500);
    
    // Create collision detector with quadtree enabled
    CollisionDetector detectorWithQuadtree(worldBounds, true);
    CollisionDetector detectorWithoutQuadtree(false);
    
    // Create a physics world
    PhysicsWorld world(Vector2D(0, 0)); // No gravity for this demo
    world.setDimensions(Vector2D(1000, 1000));
    
    std::cout << "Creating bodies for performance comparison...\n";
    
    // Create many bodies for performance testing
    std::vector<std::shared_ptr<RigidBody>> bodies;
    const int numBodies = 100;
    
    for (int i = 0; i < numBodies; ++i) {
        float x = -400 + (rand() % 800);
        float y = -400 + (rand() % 800);
        
        auto body = world.createBody(Vector2D(x, y), 1.0f);
        
        // Mix of shapes
        if (i % 3 == 0) {
            body->makeCircle(10 + rand() % 20);
        } else if (i % 3 == 1) {
            body->makeRectangle(15 + rand() % 25, 15 + rand() % 25);
        } else {
            body->makeTriangle(15 + rand() % 20);
        }
        
        body->setBodyType(RigidBody::BodyType::DYNAMIC);
        bodies.push_back(body);
    }
    
    std::cout << "Created " << numBodies << " bodies\n\n";
    
    // Performance comparison
    std::cout << "=== Performance Comparison ===\n";
    
    // Test without quadtree (O(n²) approach)
    auto startTime = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 10; ++i) {
        detectorWithoutQuadtree.detectCollisions(bodies);
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto durationWithout = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    int collisionsWithout = detectorWithoutQuadtree.getCollisionCount();
    
    std::cout << "Without Quadtree (O(n²)):\n";
    std::cout << "  Time for 10 iterations: " << durationWithout.count() << " μs\n";
    std::cout << "  Average time per iteration: " << durationWithout.count() / 10.0f << " μs\n";
    std::cout << "  Collisions detected: " << collisionsWithout << "\n\n";
    
    // Test with quadtree
    startTime = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 10; ++i) {
        detectorWithQuadtree.detectCollisions(bodies);
    }
    
    endTime = std::chrono::high_resolution_clock::now();
    auto durationWith = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    int collisionsWith = detectorWithQuadtree.getCollisionCount();
    
    std::cout << "With Quadtree:\n";
    std::cout << "  Time for 10 iterations: " << durationWith.count() << " μs\n";
    std::cout << "  Average time per iteration: " << durationWith.count() / 10.0f << " μs\n";
    std::cout << "  Collisions detected: " << collisionsWith << "\n\n";
    
    // Performance improvement
    if (durationWith.count() > 0) {
        float speedup = static_cast<float>(durationWithout.count()) / durationWith.count();
        std::cout << "Performance improvement: " << speedup << "x faster\n";
        
        if (speedup > 1.0f) {
            std::cout << "Quadtree reduced collision detection time by " 
                      << ((speedup - 1.0f) * 100.0f) << "%\n";
        }
    }
    
    std::cout << "\n=== Quadtree Statistics ===\n";
    detectorWithQuadtree.printQuadtreeStats();
    
    // Demonstrate quadtree structure
    std::cout << "\n=== Quadtree Structure ===\n";
    auto leafBounds = detectorWithQuadtree.getQuadtreeVisualization();
    std::cout << "Number of leaf nodes: " << leafBounds.size() << "\n";
    
    std::cout << "Sample leaf nodes:\n";
    for (size_t i = 0; i < std::min(leafBounds.size(), size_t(5)); ++i) {
        const AABB& leaf = leafBounds[i];
        std::cout << "  Leaf " << (i+1) << ": " << leaf << "\n";
        std::cout << "    Size: " << leaf.width() << " x " << leaf.height() << "\n";
        std::cout << "    Area: " << leaf.area() << "\n";
    }
    
    if (leafBounds.size() > 5) {
        std::cout << "  ... and " << (leafBounds.size() - 5) << " more leaf nodes\n";
    }
    
    // Test spatial queries
    std::cout << "\n=== Spatial Query Test ===\n";
    
    if (!bodies.empty()) {
        // Create a standalone quadtree for query testing
        Quadtree queryTree(worldBounds);
        queryTree.build(bodies);
        
        // Query around the first body
        auto testBody = bodies[0];
        std::cout << "Querying objects near body at position " << testBody->getPosition() << "\n";
        
        auto nearbyBodies = queryTree.query(testBody);
        std::cout << "Found " << nearbyBodies.size() << " nearby bodies\n";
        
        // Show some nearby bodies
        for (size_t i = 0; i < std::min(nearbyBodies.size(), size_t(5)); ++i) {
            Vector2D pos = nearbyBodies[i]->getPosition();
            float distance = (pos - testBody->getPosition()).magnitude();
            std::cout << "  Body " << (i+1) << " at " << pos << " (distance: " << distance << ")\n";
        }
        
        if (nearbyBodies.size() > 5) {
            std::cout << "  ... and " << (nearbyBodies.size() - 5) << " more nearby bodies\n";
        }
    }
    
    // Complexity analysis
    std::cout << "\n=== Complexity Analysis ===\n";
    std::cout << "Bodies: " << numBodies << "\n";
    std::cout << "Naive O(n²) comparisons: " << (numBodies * (numBodies - 1)) / 2 << "\n";
    
    // Estimate quadtree comparisons (very rough estimate)
    auto stats = detectorWithQuadtree.getStats();
    if (stats.averageObjectsPerLeaf > 0) {
        // Rough estimate: each body compares with others in its leaf
        float avgComparisons = stats.averageObjectsPerLeaf * (stats.averageObjectsPerLeaf - 1) / 2.0f;
        float totalComparisons = avgComparisons * stats.leafNodes;
        
        std::cout << "Estimated quadtree comparisons: ~" << static_cast<int>(totalComparisons) << "\n";
        
        if (totalComparisons > 0) {
            float theoretical_speedup = ((numBodies * (numBodies - 1)) / 2.0f) / totalComparisons;
            std::cout << "Theoretical speedup: ~" << theoretical_speedup << "x\n";
        }
    }
    
    std::cout << "\n=== Demo Complete ===\n";
    std::cout << "The quadtree spatial partitioning system provides:\n";
    std::cout << "- Efficient O(n log n) collision broad-phase\n";
    std::cout << "- Adaptive subdivision based on object density\n";
    std::cout << "- Significant performance improvements for many objects\n";
    std::cout << "- Spatial query capabilities for AI, rendering, etc.\n";
    
    return 0;
}
