//
//  Quadtree.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 06.09.2025.
//

#include <algorithm>
#include <cassert>
#include <iostream>
#include <utility>

#include <Quadtree.h>

// QuadtreeNode implementation
QuadtreeNode::QuadtreeNode(AABB  bounds, QuadtreeNode* parent, const int depth, const size_t quadrant)
    : bounds(std::move(bounds)), depth(depth), leaf(true), quadrant(quadrant), parent(parent)
{
    for (size_t i = 0; i < NONE; ++i) {
        children[i] = nullptr;
    }
}

QuadtreeNode::~QuadtreeNode() = default;

void QuadtreeNode::insert(const std::shared_ptr<RigidBody>& body)
{
    assert (body != nullptr);
    const AABB bodyAABB = body->getAABB();
    
    // ensure that this the body fits in this node's bounds
    if (!bounds.contains(bodyAABB)) {
        assert(parent != nullptr); // should not happen with root node
        return;
    }
    const size_t quad = getQuadrant(bodyAABB);
    // base case: if the body does not fit into any quadrant or if it is a leaf, always add to this node's objects list
    if (leaf || quad == NONE) {
        objects.emplace_back(body);
        if (leaf && depth < MAX_DEPTH && SPLIT_THRESHOLD <= objects.size()) split();
    }
    // if the body fits into a
    children[quad]->insert(body);
}

void QuadtreeNode::split()
{
    if (!leaf) return;
    leaf = false;
    const Vector2D center = bounds.getCenter();
    const Vector2D min = bounds.getMinimum();
    const Vector2D max = bounds.getMaximum();

    // NE quadrant (top-right)
    children[NE] = std::make_unique<QuadtreeNode>(
        AABB(center.x, center.y, max.x, max.y), this, depth + 1, NE
    );

    // NW quadrant (top-left)
    children[NW] = std::make_unique<QuadtreeNode>(
        AABB(min.x, center.y, center.x, max.y), this, depth + 1, NW
    );

    // SW quadrant (bottom-left)
    children[SW] = std::make_unique<QuadtreeNode>(
        AABB(min.x, min.y, center.x, center.y), this, depth + 1, SW
    );

    // SE quadrant (bottom-right)
    children[SE] = std::make_unique<QuadtreeNode>(
        AABB(center.x, min.y, max.x, center.y), this, depth + 1, SE
    );
    // redistribute the objects across the children (if possible) and otherwise keep them in 'newObjects'
    auto newObjects = std::vector<std::shared_ptr<RigidBody>>();
    for (const auto& body : objects) {
        AABB box = body->getAABB();
        const size_t quad = getQuadrant(box);
        if (quad < NONE) children[quad]->objects.emplace_back(body);
        else newObjects.emplace_back(body);
    }
    objects = std::move(newObjects);
}

void QuadtreeNode::update(QuadtreeNode* root)
{
    auto i = objects.begin();
    while (i != objects.end()) {
        auto body = *i;
        AABB box = body->getAABB();
        if (body->isStatic() || !body->isDynamic() || bounds.contains(box)) {
            ++i;
            continue;
        }
        // now we know that body is dynamic, awake and not contained in this box anymore -> relocate 'body'
        i = objects.erase(i);
        // if the objects list is now empty don't bother since we could use it later
        if (const auto node = root->findNode(box)) node->insert(body);
    }
    // recursively update its children
    for (const auto& child : children) {
        child->update(root);
    }
}

void QuadtreeNode::remove(const std::shared_ptr<RigidBody>& body)
{
    if (!body) return;
    const AABB box = body->getAABB();
    if (!bounds.contains(box)) return;
    if (leaf) {
        removeObject(body);
    }
    const size_t quad = getQuadrant(box);
    if (quad < NONE) {
        children[quad]->remove(body);
    } else {
        removeObject(body);
    }
}

void QuadtreeNode::removeObject(const std::shared_ptr<RigidBody>& body) {
    auto it = objects.begin();
    while (it != objects.end()) {
        if (it->get() == body.get()) {
            objects.erase(it);
            return;
        }
        ++it;
    }
}

// used by continuous collision detection
void QuadtreeNode::query(const AABB& box, std::vector<std::shared_ptr<RigidBody>>& results) const
{
    // check if query bounds intersect with this node's bounds
    if (!bounds.intersects(box)) return;
    
    // add objects from this node whose bounding box intersect with the given bounding box
    for (const auto& body : objects) {
        if (body && body->getAABB().intersects(box)) {
            results.emplace_back(body);
        }
    }
    // query children if this is a non-leaf node
    if (!isLeaf()) {
        for (const auto& child : children) {
            child->query(box, results);
        }
    }
}

void QuadtreeNode::query(const std::shared_ptr<RigidBody>&  body, std::vector<std::shared_ptr<RigidBody>>& results) const
{
    if (body) query(body->getAABB(), results);
}

void QuadtreeNode::findIntersectingQuadrants(const AABB& box, std::vector<size_t>& results) const {
    if (leaf || !bounds.contains(box)) return;
    for (size_t i = 0; i < children.size(); ++i) {
        if (children[i]->bounds.intersects(box)) results.emplace_back(i);
    }
}

void QuadtreeNode::findAllIntersections(std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<RigidBody>>>& results) const
{
    // iterate over all (non-repeating) pairs of this node's objects
    for (size_t i = 0; i < objects.size(); ++i) {
        for (size_t j = i+1; j < objects.size(); ++j) {
            AABB first = objects[i]->getAABB(), second = objects[j]->getAABB();
            if (first.intersects(second)) {
                results.emplace_back(objects[i], objects[j]);
            }
        }
    }
    if (!isLeaf()) {
        for (const auto& child : children) {
            child->findAllIntersections(results);
        }
    }
}

QuadtreeNode* QuadtreeNode::findNode(const AABB &box)
{
    assert (bounds.contains(box));
    if (leaf) return this;
    const size_t quad = getQuadrant(box);
    if (quad < NONE) return children[quad]->findNode(box);
    return this;
}

void QuadtreeNode::clear()
{
    objects.clear();
    if (!leaf) {
        for (size_t i = 0; i < NONE; ++i) {
            if (children[i]) {
                children[i]->clear();
                children[i].reset();
            }
        }
        leaf = true;
    }
}

size_t QuadtreeNode::getQuadrant(const AABB &object) const
{
    // make sure the object bounds are contained within the node bounds
    if (!bounds.contains(object)) return NONE;
    const Vector2D center = bounds.getCenter();
    const Vector2D min = object.getMinimum();
    const Vector2D max = object.getMaximum();
    if (max.x < center.x) { // left side
        if (max.y > center.y) return SW;    // bottom left quadrant
        if (min.y <= center.y) return NW;   // top left quadrant
        return NONE; // in both SW and NW quadrants
    }
    if (min.x >= center.x) { // right side
        if (max.y > center.y) return SE;    // bottom right quadrant
        if (min.y <= center.y) return NE;   // top right quadrant
        return NONE; // in both SE and NE quadrants
    }
    return NONE; // overlaps both west and east side
}

bool QuadtreeNode::fitsInQuadrant(const AABB& objectBounds, const size_t quad) const {
    // make sure the node is subdivided and the given quadrant is valid
    if (leaf || quad >= NONE) return false;
    const size_t otherQuad = getQuadrant(objectBounds);
    if (otherQuad < NONE) return otherQuad == quad;
    return false;
}

bool QuadtreeNode::isLeaf() const {
    return leaf;
}

// adds all leaf nodes to the given list of AABBs
void QuadtreeNode::getLeafNodes(std::vector<AABB>& leafs) const
{
    if (leaf) {
        // this is a leaf node
        leafs.push_back(bounds);
    } else {
        // recurse into children
        for (int i = 0; i < 4; ++i) {
            if (children[i]) {
                children[i]->getLeafNodes(leafs);
            }
        }
    }
}

AABB QuadtreeNode::getQuadrantBounds(const size_t quad) const
{
    if (leaf || quad >= NONE) return {};
    return children[quad]->bounds;
}

void QuadtreeNode::getStatisticsRecursive(Statistics& stats) const
{
    stats.totalNodes++;
    stats.maxDepth = std::max(stats.maxDepth, depth);
    
    if (leaf) {
        stats.leafNodes++;
        // add objects stored at this leaf
        stats.totalObjects += objects.size();
        return;
    }
    for (int i = 0; i < 4; ++i) {
        if (children[i]) {
            children[i]->getStatisticsRecursive(stats);
        }
    }
    // add objects stored at this level
    stats.totalObjects += objects.size();
}

// Quadtree implementation
Quadtree::Quadtree(const AABB& worldBounds)
    : worldBounds(worldBounds)
{
    root = std::make_unique<QuadtreeNode>(worldBounds, nullptr, 0);
}

Quadtree::~Quadtree() = default;

void Quadtree::insert(const std::shared_ptr<RigidBody>& body) {
    if (root && body) {
        root->insert(body);
        allBodies.emplace_back(body);
    }
}

void Quadtree::insert(const std::vector<std::shared_ptr<RigidBody>>& bodies)
{
    if (!root) return;
    for (auto& body : bodies) {
        root->insert(body);
        allBodies.emplace_back(body);
    }
}

void Quadtree::remove(const std::shared_ptr<RigidBody>& body)
{
    if (root && body) {
        root->remove(body);
        const auto it = std::find(allBodies.begin(), allBodies.end(), body);
        if (it != allBodies.end()) allBodies.erase(it);
    }
}

void Quadtree::remove(const std::vector<std::shared_ptr<RigidBody>>& bodies)
{
    if (!root) return;
    for (const auto& body : bodies) {
        if (!body) continue;
        const auto it = std::find(allBodies.begin(), allBodies.end(), body);
        if (it != allBodies.end()) {
            root->remove(body);
            allBodies.erase(it);
        }
    }
}

void Quadtree::build(const std::vector<std::shared_ptr<RigidBody>>& bodies)
{
    clear();
    allBodies.insert(allBodies.end(), bodies.begin(), bodies.end());
    for (auto& body : bodies) {
        if (body) {
            root->insert(body);
        }
    }
}

void Quadtree::findAllIntersections(std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<RigidBody>>>& results) const
{
    if (root) root->findAllIntersections(results);
}

void Quadtree::query(const std::shared_ptr<RigidBody> &body, std::vector<std::shared_ptr<RigidBody>>& results) const
{
    if (root && body) {
        root->query(body, results);

        // remove the query body itself from results
        const auto it = std::find(results.begin(), results.end(), body);
        assert (it != results.end());
        results.erase(it);
    }
}

void Quadtree::query(const AABB& box, std::vector<std::shared_ptr<RigidBody>>& results) const
{
    if (root) root->query(box, results);
}

void Quadtree::update() const {
    root->update(root.get());
}

const QuadtreeNode* Quadtree::findNode(const AABB& box) const {
    return root->findNode(box);
}

void Quadtree::clear()
{
    if (root) {
        root->clear();
    }
    allBodies.clear();
}

std::vector<AABB> Quadtree::getVisualizationBounds() const
{
    std::vector<AABB> bounds;
    if (root) {
        root->getLeafNodes(bounds);
    }
    return bounds;
}

Statistics Quadtree::getStatistics() const
{
    if (root) {
        Statistics stats;
        root->getStatisticsRecursive(stats);
        stats.avgObjectsPerNode = static_cast<float>(stats.totalObjects) / static_cast<float>(stats.totalNodes);
        return stats;
    }
    return Statistics{};
}

void Quadtree::setWorldBounds(const AABB& newWorldBounds)
{
    worldBounds = newWorldBounds;
    root = std::make_unique<QuadtreeNode>(worldBounds, nullptr, 0);
    
    // rebuild with existing bodies if any
    if (!allBodies.empty()) {
        const auto bodies = allBodies; // Copy the vector
        build(bodies);
    }
}
