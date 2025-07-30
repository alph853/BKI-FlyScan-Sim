/**
 * @file string_util.hpp
 * @brief String formatting utilities for display and logging
 *
 * This header file contains inline string formatting functions for common data types
 * used in drone control applications.
 *
 * @author UAV team@/flyscan
 * @date 2025/07/29
 */

#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace flyscan {
namespace common {

/**
 * @brief Format position coordinates as a string
 * @param north North coordinate in meters
 * @param east East coordinate in meters
 * @param down Down coordinate in meters
 * @param precision Number of decimal places
 * @return Formatted position string
 */
inline std::string formatPosition(float north, float east, float down, int precision = 2) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision);
    ss << "N:" << north << "m, E:" << east << "m, D:" << down << "m";
    return ss.str();
}

/**
 * @brief Format float value with specified precision
 * @param value Float value to format
 * @param precision Number of decimal places
 * @return Formatted string
 */
inline std::string formatFloat(float value, int precision = 2) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

/**
 * @brief Format value as percentage with specified precision
 * @param value Value to format (0.0-1.0 range)
 * @param precision Number of decimal places
 * @return Formatted percentage string
 */
inline std::string formatPercentage(float value, int precision = 1) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << (value * 100.0f) << "%";
    return ss.str();
}


} // namespace common
} // namespace flyscan