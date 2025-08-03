/**
 * @file constants.hpp
 * @brief Constants specific to flyscan_drone_controller package
 * 
 * This file contains constants used by PX4Controller and TeleopNode.
 * 
 * @author UAV Team
 * @date 2025-07-30
 */

#pragma once

namespace flyscan {
namespace drone_controller {

namespace topic {
    // PX4 input topics (commands to PX4)
    constexpr const char* PX4_OFFBOARD_CONTROL_MODE = "/fmu/in/offboard_control_mode";
    constexpr const char* PX4_TRAJECTORY_SETPOINT   = "/fmu/in/trajectory_setpoint";
    constexpr const char* PX4_VEHICLE_COMMAND       = "/fmu/in/vehicle_command";
    
    // PX4 output topics (telemetry from PX4)
    constexpr const char* PX4_VEHICLE_LOCAL_POSITION = "/fmu/out/vehicle_local_position_v1";
    constexpr const char* PX4_VEHICLE_STATUS         = "/fmu/out/vehicle_status_v1";
    constexpr const char* PX4_VEHICLE_LAND_DETECTED  = "/fmu/out/vehicle_land_detected";
    
    // Internal teleop communication
    constexpr const char* TELEOP_COMMAND = "/px4_controller/teleop_command";

    // Exploration topics
    constexpr const char* EXPLORATION_GOAL = "/exploration_goal";

} // namespace topic

namespace srv {
    constexpr const char* SET_CONTROL_MODE = "/px4_controller/set_control_mode";
} // namespace srv


} // namespace drone_controller
} // namespace flyscan