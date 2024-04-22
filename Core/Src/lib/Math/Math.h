#pragma once

#include <cmath>
#include <limits>
#include <stdint.h>

//#define M_PI      (3.141592653589793)
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

// degrees -> radians
static inline constexpr float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

// radians -> degrees
static inline constexpr float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

static inline float map(float value, float inMin, float inMax, float outMin, float outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

template <typename T>
T abs(T value) {
    return (value < 0) ? -value : value;
}

