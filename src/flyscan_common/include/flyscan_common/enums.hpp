/**
 * @file enums.hpp
 * @brief Common enums for autonomous UAV nodes
 *
 * This file contains the common enums used in the autonomous UAV nodes.
 *
 * @author UAV team@/flyscan
 * @date 2025/06/18
 */

#pragma once

#include <cstdint>
#include <string>

namespace flyscan {

namespace common {

/**
 * @brief Operation status for function return values
 */
enum class OperationStatus : uint8_t
{
    kOK             = 0,
    kPending        = 1,
    kOutOfMemory    = 2,
    kMalformedInput = 3,
    kNotInitialized = 4,
    kServiceNA      = 5,
    kTimeout        = 6,
    kNotImplemented = 7,
    kAlreadyExists  = 8,
    kNotFound       = 9,

    kUnknown        = 99,
};

/**
 * @brief Node type
 */
enum class NodeType : uint8_t
{
    kBaseNode    = 0,
    kNavigation  = 1,
    kPerception  = 2,
    kExploration = 3,
    kPlanning    = 4,
    kManaging    = 5,
    kVision      = 6,

    kUnknown     = 99,
};

/**
 * @brief Convert operation status to string
 * @param status Operation status
 * @return String representation of operation status
 */
inline std::string OperationStatusToString(const OperationStatus& status)
{
    switch (status)
    {
        case OperationStatus::kOK:
            return "SUCCESS";
        case OperationStatus::kPending:
            return "PENDING";
        case OperationStatus::kOutOfMemory:
            return "OUT_OF_MEMORY";
        case OperationStatus::kMalformedInput:
            return "MALFORMED_INPUT";
        case OperationStatus::kNotInitialized:
            return "NOT_INITIALIZED";
        case OperationStatus::kServiceNA:
            return "SERVICE_NOT_AVAILABLE";
        case OperationStatus::kTimeout:
            return "TIMEOUT";
        case OperationStatus::kNotImplemented:
            return "NOT_IMPLEMENTED";
        default:
            return "UNKNOWN";
    }
}

inline std::string NodeTypeToString(const NodeType& type)
{
    switch (type)
    {
        case NodeType::kBaseNode:
            return "BaseNode";
        case NodeType::kNavigation:
            return "Navigation";
        case NodeType::kPerception:
            return "Perception";
        case NodeType::kExploration:
            return "Exploration";
        case NodeType::kPlanning:
            return "Planning";
        case NodeType::kManaging:
            return "Managing";
        case NodeType::kVision:
            return "Vision";
        default:
            return "Unknown";
    }
}

} // namespace common

} // namespace flyscan
