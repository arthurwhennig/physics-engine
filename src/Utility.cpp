//
//  Utility.cpp
//  physics-engine
//
//  Created by Arthur Hennig on 05.09.2025.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H
#endif // CONSTANTS_H

#include <cmath>

#include <Utility.h>

float degreesToRadians(const float degrees) {
    return (degrees/180.0f) * PI;
}

float radiansToDegrees(const float radians) {
    return (radians*180.0f) / PI;
}

// normalizes the given radians to the interval [0, 2π)
float normalizeRadians(const float radians) {
    const float num = floor(radians / (2*PI));
    return radians - (num * 2*PI);
}