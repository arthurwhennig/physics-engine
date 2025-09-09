//
//  Quadtree.h
//  physics-engine
//
//  Created by Arthur Hennig on 06.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_QUADTREE_H
#define PHYSICSENGINE_QUADTREE_H
#endif //PHYSICSENGINE_QUADTREE_H

#include <memory>
#include <vector>

#include <AABB.h>
#include <RigidBody.h>

/**
 * @brief Quadtree node for spatial partitioning
 * 
 * Recursively subdivides 2D space into quadrants to enable efficient
 * spatial queries for collision detection broad-phase.
 */

// Statistics metadata object
struct Statistics {
    size_t totalNodes = 0;
    size_t leafNodes = 0;
    int maxDepth = 0;
    size_t totalObjects = 0;
    float avgObjectsPerNode = 0.0f;
};

class QuadtreeNode {
public:
    // configuration constants
    static constexpr int MAX_DEPTH = 8;
    static constexpr size_t SPLIT_THRESHOLD = 4;
    static constexpr size_t NE = 0, NW = 1, SW = 2, SE = 3, NONE=4;     // mathematical quadrant numbering

    // constructor
    explicit QuadtreeNode(AABB  bounds, QuadtreeNode* parent, int depth = 0, size_t quadrant = NE);

    // destructor
    ~QuadtreeNode();

    // tries to insert the given body into this node (based on the body's AABB)
    void insert(const std::shared_ptr<RigidBody>& body);

    // removes the given body from this node and returns true if the resulting node and all its descendants have no objects anymore
    void remove(const std::shared_ptr<RigidBody>& body);

    // relocates this node's objects if their positions changed
    void update(QuadtreeNode* root);

    // finds (within this node) the smallest existing node whose bounds entirely contain the given bounding box
    [[nodiscard]] QuadtreeNode* findNode(const AABB &box);

    // finds all list of pairs of intersections between objects within this node and adds them to the given results list
    void findAllIntersections(std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<RigidBody>>>& results) const;

    // query objects that intersect with the given AABB and add them to the given results list
    void query(const AABB& box, std::vector<std::shared_ptr<RigidBody>>& results) const;

    // query objects that intersect with the given rigid body and add them to the given results list
    void query(const std::shared_ptr<RigidBody> &body, std::vector<std::shared_ptr<RigidBody>>& results) const;

    // clear all objects from the quadtree
    void clear();

    // returns true if this node is a leaf node (without any children)
    [[nodiscard]] bool isLeaf() const;

    // recursively collects statistics about this node and its children
    void getStatisticsRecursive(Statistics& stats) const;

    // get all leaf nodes (for visualization)
    void getLeafNodes(std::vector<AABB>& leafs) const;

private:
    AABB bounds;            // bounding box of this quadtree node
    int depth;              // the depth of this quadtree node with respect to the root (depth = 0)
    bool leaf;              // true if this quadtree node is a leaf, otherwise false
    size_t quadrant;        // the current quadrant that this node is contained in (not applicable to root)
    QuadtreeNode* parent;   // pointer to the parent quadtree node

    // child nodes (NW, NE, SW, SE)
    std::array<std::unique_ptr<QuadtreeNode>, 4> children;      // the children of this quadtree node

    // objects stored at this node
    std::vector<std::shared_ptr<RigidBody>> objects;            // the objects belonging to this quadtree node

    // splits this node into four children nodes (the quadrants)
    void split();

    // helper function to remove the given body from the 'objects' list
    void removeObject(const std::shared_ptr<RigidBody>& body);

    // returns the quadrant index (0 to 3) for a given AABB (returns 4 if no quadrant matches)
    [[nodiscard]] size_t getQuadrant(const AABB &object) const;

    // returns a list of quadrant numbers which intersect with the given bounding box (which is contained in this node)
    void findIntersectingQuadrants(const AABB& box, std::vector<size_t>& results) const;

    // check if an AABB fits entirely within a quadrant
    [[nodiscard]] bool fitsInQuadrant(const AABB& objectBounds, size_t quad) const;

    // get bounds for a specific quadrant
    [[nodiscard]] AABB getQuadrantBounds(size_t quad) const;
};

/**
 * @brief Quadtree spatial partitioning system
 * 
 * High-level interface for the quadtree spatial partitioning system.
 * Manages the root node and provides convenient methods for physics simulation.
 */
class Quadtree {
public:
    // constructor
    explicit Quadtree(const AABB& worldBounds);
    
    // destructor
    ~Quadtree();

    // tries to insert the given body (based on its AABB) into this Quadtree (fails if not contained within world dimensions)
    void insert(const std::shared_ptr<RigidBody>& body);

    // tries to insert the given list of bodies into this Quadtree (fails if not contained within world dimensions)
    void insert(const std::vector<std::shared_ptr<RigidBody>>& bodies);

    // tries to remove the given body from the Quadtree
    void remove(const std::shared_ptr<RigidBody> &body);

    // tries to remove the given list of bodies from the Quadtree
    void remove(const std::vector<std::shared_ptr<RigidBody>>& bodies);
    
    // build the quadtree from a list of rigid bodies
    void build(const std::vector<std::shared_ptr<RigidBody>>& bodies);

    // updates the location of the bodies within the quadtree if necessary
    void update() const;

    // finds all intersecting bodies
    void findAllIntersections(std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<RigidBody>>>& results) const;
    
    // query all objects that collide with the given body's bounding box (AABB)
    void query(const std::shared_ptr<RigidBody> &body, std::vector<std::shared_ptr<RigidBody>>& results) const;

    // query all objects that collide with the given bounding box (AABB)
    void query(const AABB& box, std::vector<std::shared_ptr<RigidBody>>& results) const;
    
    // clear the quadtree
    void clear();
    
    // get visualization data
    [[nodiscard]] std::vector<AABB> getVisualizationBounds() const;

    // get statistics about the quadtree
    [[nodiscard]] Statistics getStatistics() const;
    
    // update world bounds
    void setWorldBounds(const AABB& newWorldBounds);

private:
    std::unique_ptr<QuadtreeNode> root;
    AABB worldBounds;
    
    // all bodies currently in the quadtree
    std::vector<std::shared_ptr<RigidBody>> allBodies;

    // returns the smallest existing node whose bounds entirely contain the given bounding box
    [[nodiscard]] const QuadtreeNode* findNode(const AABB& box) const;
};

