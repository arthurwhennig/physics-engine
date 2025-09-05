//
// Created by Arthur Hennig on 05.09.2025.
//

#include <iostream>
#include <SFML/Graphics.hpp>

#include "PhysicsWorld.hpp"

int main()
{
    constexpr unsigned int screen_width = 800u;
    constexpr unsigned int screen_height = 600u;
    constexpr float gravity = 20.0f;
    constexpr int FPS = 60;
    constexpr int DURATION = 10;
    constexpr float time_delta = 1.0f / FPS;

    PhysicsWorld world(Vector2D(0, gravity));

    sf::RenderWindow window(sf::VideoMode({screen_width, screen_height}), "Physics Engine Graphics Demo");
    window.setFramerateLimit(FPS);

    // first object: simple circle with radius 30 which is falling down
    std::shared_ptr<RigidBody> falling = world.createBody(Vector2D(200, 200), 10.0f, 30.0f); // dynamic body
    falling->setBodyType(RigidBody::BodyType::DYNAMIC);
    const Vector2D *falling_position = &falling->getPosition();

    sf::CircleShape fallingShape(30);
    fallingShape.setFillColor(sf::Color::Red);

    // second object: simple static circle which serves as a ground
    std::shared_ptr<RigidBody> ground = world.createBody(Vector2D(200, 350), 0.0f, 30.0f); // static body
    ground->setBodyType(RigidBody::BodyType::STATIC);
    const Vector2D *ground_position = &ground->getPosition();

    sf::CircleShape groundShape(30);
    groundShape.setFillColor(sf::Color::Green);

    std::cout << "=== Basic SFML 3.0 demo running ===" << "\n";
    std::cout << "Close the window to exit." << "\n";

    int frame = 0;
    while (window.isOpen() && frame < DURATION * FPS) {
        // handle events
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }
        // render
        window.clear(sf::Color::Black);
        fallingShape.setPosition({falling_position->x, falling_position->y});
        groundShape.setPosition({ground_position->x, ground_position->y});
        window.draw(fallingShape);
        window.draw(groundShape);
        window.display();

        world.step(time_delta);
        frame++;
    }

    std::cout << "=== Basic SFML 3.0 demo finished ===" << "\n";
    return 0;
}
