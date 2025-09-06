//
// Polygon Collision Detection Graphics Demo
// Created by Arthur Hennig on 05.09.2025.
//

#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include "PhysicsWorld.h"
#include "CollisionDetector.h"
#include <Utility.h>

constexpr char FONT_PATH[33] = "/System/Library/Fonts/Monaco.ttf";

// helper function to create SFML polygon shape from RigidBody
sf::ConvexShape createPolygonShape(const RigidBody& body, sf::Color color) {
    const Polygon& poly = body.getPolygon();
    sf::ConvexShape shape;
    
    if (body.getBodyShape() == RigidBody::BodyShape::POLYGON) {
        shape.setPointCount(poly.size());
        for (size_t i = 0; i < poly.size(); ++i) {
            Vector2D worldPoint = poly.getWorldPoint(i);
            shape.setPoint(i, sf::Vector2f(worldPoint.x, worldPoint.y));
        }
    } else if (body.getBodyShape() == RigidBody::BodyShape::CIRCLE) {
        // create circular approximation for rendering
        const int segments = 16;
        shape.setPointCount(segments);
        float radius = body.getRadius();
        const Vector2D& center = body.getPosition();
        for (int i = 0; i < segments; ++i) {
            float angle = 2.0f * PI * i / segments;
            float x = center.x + radius * cos(angle);
            float y = center.y + radius * sin(angle);
            shape.setPoint(i, sf::Vector2f(x, y));
        }
    }
    
    shape.setFillColor(color);
    shape.setOutlineThickness(2);
    shape.setOutlineColor(sf::Color::White);
    return shape;
}

// helper function to draw contact points
void drawContactPoints(sf::RenderWindow& window, const std::vector<CollisionInfo>& collisions) {
    for (const auto& collision : collisions) {
        for (int i = 0; i < collision.contactCount; ++i) {
            sf::CircleShape contactDot(3);
            contactDot.setFillColor(sf::Color::Yellow);
            contactDot.setPosition(sf::Vector2f(collision.contactPoints[i].x - 3, collision.contactPoints[i].y - 3));
            window.draw(contactDot);

            sf::Vertex first;
            first.position = sf::Vector2f(collision.contactPoints[i].x, collision.contactPoints[i].y);
            first.color = sf::Color::Red;
            sf::Vertex second;
            second.position = sf::Vector2f(
                    collision.contactPoints[i].x + collision.contactNormal.x * 30,
                    collision.contactPoints[i].y + collision.contactNormal.y * 30
                    );
            second.color = sf::Color::Red;
            // draw contact normal
            const sf::Vertex line[] = {
                first, second
            };
            window.draw(line, 2, sf::PrimitiveType::Lines);
        }
    }
}

int main()
{
    constexpr unsigned int screen_width = 1200u;
    constexpr unsigned int screen_height = 800u;
    constexpr float gravity = 50.0f;
    constexpr int FPS = 60;
    constexpr float time_delta = 1.0f / FPS;

    PhysicsWorld world(Vector2D(0, gravity));
    world.setDimensions(Vector2D(screen_width, screen_height));
    
    CollisionDetector detector;

    sf::RenderWindow window(sf::VideoMode({screen_width, screen_height}), "Polygon Collision Detection Demo");
    window.setFramerateLimit(FPS);

    // create various test objects
    std::vector<std::shared_ptr<RigidBody>> bodies;
    
    // falling rectangular box
    auto fallingBox = world.createBody(Vector2D(300, 100), 5.0f);
    fallingBox->makeRectangle(60, 40);
    fallingBox->setBodyType(RigidBody::BodyType::DYNAMIC);
    fallingBox->setRestitution(0.6f);
    bodies.push_back(fallingBox);
    
    // static ground rectangle
    auto ground = world.createBody(Vector2D(600, 700), 0.0f);
    ground->makeRectangle(800, 40);
    ground->setBodyType(RigidBody::BodyType::STATIC);
    bodies.push_back(ground);
    
    // triangular objects
    auto triangle1 = world.createBody(Vector2D(500, 150), 3.0f);
    triangle1->makeTriangle(30, 30);
    triangle1->setBodyType(RigidBody::BodyType::DYNAMIC);
    triangle1->setRestitution(0.7f);
    bodies.push_back(triangle1);
    
    auto triangle2 = world.createBody(Vector2D(700, 200), 2.0f);
    triangle2->makeTriangle(25, 25);
    triangle2->setBodyType(RigidBody::BodyType::DYNAMIC);
    triangle2->setRestitution(0.8f);
    bodies.push_back(triangle2);
    
    // static wall rectangles
    auto leftWall = world.createBody(Vector2D(50, 400), 0.0f);
    leftWall->makeRectangle(20, 400);
    leftWall->setBodyType(RigidBody::BodyType::STATIC);
    bodies.push_back(leftWall);
    
    auto rightWall = world.createBody(Vector2D(1150, 400), 0.0f);
    rightWall->makeRectangle(20, 400);
    rightWall->setBodyType(RigidBody::BodyType::STATIC);
    bodies.push_back(rightWall);
    
    // mixed circle for testing circle-polygon collisions
    auto circle = world.createBody(Vector2D(400, 50), 4.0f);
    circle->makeCircle(25);
    circle->setBodyType(RigidBody::BodyType::DYNAMIC);
    circle->setRestitution(0.9f);
    bodies.push_back(circle);
    
    // platform rectangles
    auto platform1 = world.createBody(Vector2D(200, 500), 0.0f);
    platform1->makeRectangle(120, 20);
    platform1->setBodyType(RigidBody::BodyType::STATIC);
    bodies.push_back(platform1);
    
    auto platform2 = world.createBody(Vector2D(800, 400), 0.0f);
    platform2->makeRectangle(100, 20);
    platform2->setBodyType(RigidBody::BodyType::STATIC);
    bodies.push_back(platform2);

    std::cout << "=== Polygon Collision Detection Demo ===" << "\n";
    std::cout << "Yellow dots: Contact points" << "\n";
    std::cout << "Red lines: Contact normals" << "\n";
    std::cout << "Press SPACE to add a new falling box" << "\n";
    std::cout << "Press R to reset simulation" << "\n";
    std::cout << "Close the window to exit." << "\n";

    sf::Font font;
    sf::Text debugText = sf::Text(font);
    bool fontLoaded = font.openFromFile(FONT_PATH);
    if (fontLoaded) {
        debugText.setFont(font);
        debugText.setCharacterSize(16);
        debugText.setFillColor(sf::Color::White);
        debugText.setPosition(sf::Vector2f(10, 10));
    }
    
    int frameCount = 0;
    
    while (window.isOpen()) {
        // handle events
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
                if (keyPressed->code == sf::Keyboard::Key::Space) {
                    // add a new falling box
                    auto newBox = world.createBody(
                        Vector2D(200 + (rand() % 400), 50), 
                        2.0f + (rand() % 5)
                    );
                    newBox->makeRectangle(30 + (rand() % 40), 30 + (rand() % 40));
                    newBox->setBodyType(RigidBody::BodyType::DYNAMIC);
                    newBox->setRestitution(0.3f + 0.4f * (rand() % 100) / 100.0f);
                    bodies.push_back(newBox);
                }
                if (keyPressed->code == sf::Keyboard::Key::R) {
                    // reset simulation
                    world.clearBodies();
                    bodies.clear();
                    
                    // recreate initial objects
                    fallingBox = world.createBody(Vector2D(300, 100), 5.0f);
                    fallingBox->makeRectangle(60, 40);
                    fallingBox->setBodyType(RigidBody::BodyType::DYNAMIC);
                    fallingBox->setRestitution(0.6f);
                    bodies.push_back(fallingBox);
                    
                    ground = world.createBody(Vector2D(600, 700), 0.0f);
                    ground->makeRectangle(800, 40);
                    ground->setBodyType(RigidBody::BodyType::STATIC);
                    bodies.push_back(ground);
                    
                    // add other static objects back...
                }
            }
        }
        
        // physics step
        world.step(time_delta);
        frameCount++;
        
        // detect collisions for visualization
        detector.detectCollisions(world.getBodies());
        const auto& collisions = detector.getCollisions();
        
        // render
        window.clear(sf::Color(30, 30, 50));
        
        // draw all bodies
        for (const auto& body : bodies) {
            sf::Color bodyColor = sf::Color::Blue;
            if (body->getBodyType() == RigidBody::BodyType::STATIC) {
                bodyColor = sf::Color::Green;
            } else if (body->getBodyShape() == RigidBody::BodyShape::CIRCLE) {
                bodyColor = sf::Color::Red;
            }
            
            sf::ConvexShape shape = createPolygonShape(*body, bodyColor);
            window.draw(shape);
        }
        
        // draw contact points and normals
        drawContactPoints(window, collisions);
        
        // draw debug info
        if (fontLoaded) {
            debugText.setString(
                "Bodies: " + std::to_string(world.getBodyCount()) + 
                "\nCollisions: " + std::to_string(collisions.size()) +
                "\nFrame: " + std::to_string(frameCount) +
                "\nPress SPACE for new box, R to reset"
            );
            window.draw(debugText);
        }
        
        window.display();
    }

    std::cout << "=== Polygon Collision Demo finished ===" << "\n";
    return 0;
}
