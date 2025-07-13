#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace flyscan_drone_controller {
namespace util {

/**
 * @brief Mathematical utility functions for drone control calculations
 * 
 * This class provides common mathematical operations used in drone control,
 * including angle constraints, distance calculations, and tolerance checks.
 */
class MathUtils {
public:
    /**
     * @brief Constrain angle to range [-180, 180] degrees
     * @param angle Input angle in degrees
     * @return Constrained angle in degrees
     */
    static float constrainAngle(float angle);
    
    /**
     * @brief Calculate 3D Euclidean distance between two points
     * @param x1,y1,z1 Coordinates of first point
     * @param x2,y2,z2 Coordinates of second point
     * @return Distance between the points
     */
    static float calculateDistance3D(float x1, float y1, float z1, float x2, float y2, float z2);
    
    /**
     * @brief Calculate 2D Euclidean distance between two points
     * @param x1,y1 Coordinates of first point
     * @param x2,y2 Coordinates of second point
     * @return Distance between the points
     */
    static float calculateDistance2D(float x1, float y1, float x2, float y2);
    
    /**
     * @brief Check if a value is within tolerance of a target
     * @param value Current value
     * @param target Target value
     * @param tolerance Acceptable tolerance
     * @return true if value is within tolerance of target
     */
    static bool isWithinTolerance(float value, float target, float tolerance);
    
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;  ///< Degrees to radians conversion factor
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;  ///< Radians to degrees conversion factor
};

/**
 * @brief String formatting utilities for display and logging
 * 
 * This class provides formatting functions for common data types
 * used in drone control applications.
 */
class StringUtils {
public:
    /**
     * @brief Format position coordinates as a string
     * @param north North coordinate in meters
     * @param east East coordinate in meters
     * @param down Down coordinate in meters
     * @param precision Number of decimal places
     * @return Formatted position string
     */
    static std::string formatPosition(float north, float east, float down, int precision = 2);
    
    /**
     * @brief Format float value with specified precision
     * @param value Float value to format
     * @param precision Number of decimal places
     * @return Formatted string
     */
    static std::string formatFloat(float value, int precision = 2);
    
    /**
     * @brief Format value as percentage with specified precision
     * @param value Value to format (0.0-1.0 range)
     * @param precision Number of decimal places
     * @return Formatted percentage string
     */
    static std::string formatPercentage(float value, int precision = 1);
};

/**
 * @brief Print usage information for command-line applications
 * @param bin_name Name of the executable
 */
void printUsage(const std::string& bin_name);

} // namespace util
} // namespace flyscan_drone_controller