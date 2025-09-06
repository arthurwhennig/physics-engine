//
//  Utility.h
//  physics-engine
//
//  Created by Arthur Hennig on 05.09.2025.
//

#pragma once

#ifndef PHYSICSENGINE_UTILITY_H
#define PHYSICSENGINE_UTILITY_H
#endif //PHYSICSENGINE_UTILITY_H

constexpr float PI = 3.14159265359f;
constexpr float EPSILON = 1e-6f;

// converts the given number of degrees to radians
float degreesToRadians(float degrees);

// converts the given number of radians to degrees
float radiansToDegrees(float radians);

// normalizes the given radians to the interval [0, 2π)
float normalizeRadians(float radians);