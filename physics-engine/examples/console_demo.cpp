//
// Created by Arthur Hennig on 04.09.2025.
//

#include <iomanip>
#include <iostream>
#include <thread>

#include "PhysicsWorld.hpp"

int main() {
    constexpr int DURATION = 12; // simulation duration in seconds
    constexpr int FPS = 60; // frames per second
    constexpr float GRAVITY = 9.81f; // gravity constant
    constexpr float TIME_STEP = 1.0f / FPS; // time delta for each integration

    auto world = PhysicsWorld(Vector2D(0, GRAVITY));
    const auto falling = world.createBody(Vector2D(200, 200), 10, 30);
    falling->setBodyType(RigidBody::BodyType::DYNAMIC);
    falling->setRestitution(0.9f);
    const auto ground = world.createBody(Vector2D(200, 300), 0, 30);
    ground->setBodyType(RigidBody::BodyType::STATIC);
    ground->setRestitution(0.8f);

    // simulation
    const auto startTime = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < DURATION * FPS; i++) {
        if (i % (FPS/2) == 0) { // print body data every half second
            world.printBodiesInfo();
        }

        if (i == (3*DURATION/4) * FPS) { // at 3/4 time, add a constant wind force to all objects
            auto windGenerator = std::make_shared<ConstantForceGenerator>(Vector2D(2.0f, -5.0f));
            windGenerator->affect(falling);
            world.addForceGenerator(windGenerator);
        }

        world.step(TIME_STEP);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(TIME_STEP * 1000))); // ~60 FPS
    }
    const auto endTime = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "\n=== Simulation Completed! ===\n";
    std:: cout << "Total duration: " << duration.count() << "ms\n";

    return 0;
}
