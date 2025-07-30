/**
 * @file math_util.hpp
 * @brief Mathematical utility functions for drone control calculations
 *
 * This header file contains inline mathematical utility functions used in drone control,
 * including angle constraints, distance calculations, and tolerance checks.
 *
 * @author UAV team@/flyscan
 * @date 2025/07/29
 */

#pragma once

#include <cmath>

namespace flyscan {
namespace common {

/**
 * @brief Constrain angle to range [-180, 180] degrees
 * @param angle Input angle in degrees
 * @return Constrained angle in degrees
 */
inline float constrainAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief Calculate 3D Euclidean distance between two points
 * @param x1,y1,z1 Coordinates of first point
 * @param x2,y2,z2 Coordinates of second point
 * @return Distance between the points
 */
inline float calculateDistance3D(float x1, float y1, float z1, float x2, float y2, float z2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * @brief Calculate 2D Euclidean distance between two points
 * @param x1,y1 Coordinates of first point
 * @param x2,y2 Coordinates of second point
 * @return Distance between the points
 */
inline float calculateDistance2D(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * @brief Check if a value is within tolerance of a target
 * @param value Current value
 * @param target Target value
 * @param tolerance Acceptable tolerance
 * @return true if value is within tolerance of target
 */
inline bool isWithinTolerance(float value, float target, float tolerance) {
    return std::abs(value - target) <= tolerance;
}

static constexpr float DEG_TO_RAD = M_PI / 180.0f;  ///< Degrees to radians conversion factor
static constexpr float RAD_TO_DEG = 180.0f / M_PI;  ///< Radians to degrees conversion factor

} // namespace common
} // namespace flyscan