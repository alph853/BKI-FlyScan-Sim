/**
 * @file waypoint_mission.hpp
 * @author Flyscan
 * @brief Waypoint mission executor for autonomous drone missions
 * @version 0.1
 * @date 2025-08-06
 * @copyright Copyright (c) 2025
 */

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include <flyscan_interfaces/srv/set_control_mode.hpp>
#include <flyscan_interfaces/srv/navigate_to_pose.hpp>

#include "flyscan_core/base_node.hpp"
#include "flyscan_common/enums.hpp"

namespace flyscan {
namespace exploration {

using flyscan::common::OperationStatus;
using flyscan::common::NodeType;
using flyscan::core::BaseNode;

/**
 * @brief Structure representing a single waypoint in the mission
 */
struct Waypoint {
    geometry_msgs::msg::Point position;     ///< 3D position (x, y, z in map frame)
    double yaw;                            ///< Yaw angle in degrees
    double work_time;                      ///< Time to hover at this waypoint (seconds)
    std::string description;               ///< Optional description of waypoint
    
    Waypoint() : yaw(0.0), work_time(0.0), description("") {}
};

/**
 * @brief Mission configuration loaded from YAML file
 */
struct Mission {
    std::string name;
    std::string description;
    double waypoint_tolerance;
    double hover_time;
    double max_velocity;
    double safety_margin;
    std::vector<Waypoint> waypoints;
    std::string completion_action;
    geometry_msgs::msg::Point completion_position;
    
    Mission() : waypoint_tolerance(0.5), hover_time(3.0), max_velocity(1.0), 
                safety_margin(1.5), completion_action("hover") {}
};

/**
 * @brief Mission execution states
 */
enum class MissionState {
    kIdle,                    ///< Mission not started
    kLoaded,                  ///< Mission loaded from file
    kSwitchingMode,           ///< Switching to mission mode
    kNavigating,              ///< Navigating to current waypoint
    kWorking,                 ///< Hovering/working at current waypoint
    kCompleted,               ///< Mission completed successfully
    kAborted,                 ///< Mission aborted due to error
    kPaused                   ///< Mission temporarily paused
};

/**
 * @brief Waypoint Mission Executor Node
 * 
 * This node loads waypoint missions from YAML files and executes them
 * by commanding the PX4 controller to navigate through predefined waypoints.
 */
class WaypointMission : public BaseNode {
public:
    explicit WaypointMission(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
        const std::string& node_name = "waypoint_mission",
        const NodeType& node_type = NodeType::kMission,
        const std::vector<std::string>& capabilities = {"waypoint_mission", "file_mission"}
    );

    ~WaypointMission();

    /**
     * @brief Load mission from YAML file
     * @param mission_file Path to mission YAML file
     * @return OperationStatus indicating success or failure
     */
    OperationStatus LoadMission(const std::string& mission_file);

    /**
     * @brief Start executing the loaded mission
     * @return OperationStatus indicating success or failure
     */
    OperationStatus StartMission();

    /**
     * @brief Pause the current mission execution
     * @return OperationStatus indicating success or failure
     */
    OperationStatus PauseMission();

    /**
     * @brief Resume paused mission execution
     * @return OperationStatus indicating success or failure
     */
    OperationStatus ResumeMission();

    /**
     * @brief Abort the current mission
     * @return OperationStatus indicating success or failure
     */
    OperationStatus AbortMission();

protected:
    // ============================================================================
    // Lifecycle Management (BaseNode overrides)
    // ============================================================================
    
    OperationStatus HandleConfigure() override;
    OperationStatus HandleActivate() override;
    OperationStatus HandleDeactivate() override;
    OperationStatus HandleCleanup() override;
    OperationStatus HandleShutdown() override;
    OperationStatus HandleError() override;

private:
    // ============================================================================
    // Mission Execution Methods
    // ============================================================================
    
    /**
     * @brief Timer callback for mission execution state machine
     */
    void MissionTimerCallback();
    
    /**
     * @brief Switch to mission mode in PX4 controller
     * @return OperationStatus indicating success or failure
     */
    OperationStatus SwitchToMissionMode();
    
    /**
     * @brief Navigate to the current waypoint
     * @return OperationStatus indicating success or failure
     */
    OperationStatus NavigateToCurrentWaypoint();
    
    /**
     * @brief Check if current waypoint is reached
     * @return True if waypoint is reached within tolerance
     */
    bool IsCurrentWaypointReached();
    
    /**
     * @brief Advance to the next waypoint in the mission
     */
    void AdvanceToNextWaypoint();
    
    /**
     * @brief Complete the mission and perform completion action
     */
    void CompleteMission();
    
    /**
     * @brief Parse waypoint from YAML node
     * @param node YAML node containing waypoint data
     * @return Parsed waypoint structure
     */
    Waypoint ParseWaypoint(const YAML::Node& node);

    // ============================================================================
    // ROS2 Communication
    // ============================================================================
    
    rclcpp::Client<flyscan_interfaces::srv::SetControlMode>::SharedPtr set_control_mode_client_;
    rclcpp::Client<flyscan_interfaces::srv::NavigateToPose>::SharedPtr navigate_to_pose_client_;
    
    rclcpp::TimerBase::SharedPtr mission_timer_;
    
    // ============================================================================
    // Mission State Management
    // ============================================================================
    
    Mission current_mission_;
    MissionState mission_state_;
    size_t current_waypoint_index_;
    rclcpp::Time waypoint_arrival_time_;
    rclcpp::Time mission_start_time_;
    
    // ============================================================================
    // ROS Parameters
    // ============================================================================
    
    std::string default_mission_file_;
    double mission_update_rate_;           // Hz
    bool auto_start_mission_;
    
    std::mutex mission_mutex_;
};

} // namespace exploration
} // namespace flyscan