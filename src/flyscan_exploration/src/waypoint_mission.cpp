#include <fstream>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "flyscan_exploration/waypoint_mission.hpp"
#include "flyscan_common/sigint_handler.hpp"

namespace flyscan {
namespace exploration {

WaypointMission::WaypointMission(const rclcpp::NodeOptions& options,
                                 const std::string& node_name,
                                 const NodeType& node_type,
                                 const std::vector<std::string>& capabilities)
    : BaseNode(options, node_name, node_type, capabilities)
    , mission_state_(MissionState::kIdle)
    , current_waypoint_index_(0)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Waypoint Mission Node: %s", node_name.c_str());
    
    // Declare parameters
    if (!this->has_parameter("default_mission_file")) {
        this->declare_parameter("default_mission_file", "config/sample_mission.yaml");
    }
    if (!this->has_parameter("mission_update_rate")) {
        this->declare_parameter("mission_update_rate", 5.0);
    }
    if (!this->has_parameter("auto_start_mission")) {
        this->declare_parameter("auto_start_mission", false);
    }
    
    RCLCPP_INFO(this->get_logger(), "Waypoint Mission Node initialized");
}

WaypointMission::~WaypointMission() {
    RCLCPP_INFO(this->get_logger(), "WaypointMission node destroyed");
}

// ============================================================================
// Lifecycle Management Implementation
// ============================================================================

OperationStatus WaypointMission::HandleConfigure() {
    RCLCPP_INFO(this->get_logger(), "Configuring Waypoint Mission...");
    
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    // Cache ROS parameters
    default_mission_file_ = this->get_parameter("default_mission_file").as_string();
    mission_update_rate_ = this->get_parameter("mission_update_rate").as_double();
    auto_start_mission_ = this->get_parameter("auto_start_mission").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Mission parameters: file=%s, rate=%.1fHz, auto_start=%s",
                default_mission_file_.c_str(), mission_update_rate_, 
                auto_start_mission_ ? "true" : "false");

    // Create service clients
    set_control_mode_client_ = this->create_client<flyscan_interfaces::srv::SetControlMode>(
        "/px4_controller/set_control_mode",
        rclcpp::ServicesQoS()
    );
    
    navigate_to_pose_client_ = this->create_client<flyscan_interfaces::srv::NavigateToPose>(
        "/px4_controller/navigate_to_pose",
        rclcpp::ServicesQoS()
    );
    
    // Create mission execution timer
    mission_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / mission_update_rate_),
        std::bind(&WaypointMission::MissionTimerCallback, this));
    mission_timer_->cancel(); // Start paused
    
    RCLCPP_INFO(this->get_logger(), "Created service clients and mission timer");
    
    RCLCPP_INFO(this->get_logger(), "Waypoint Mission configured successfully");
    return OperationStatus::kOK;
}

OperationStatus WaypointMission::HandleActivate() {
    RCLCPP_INFO(this->get_logger(), "Activating Waypoint Mission...");

    // Load default mission file if specified
    if (!default_mission_file_.empty()) {
        std::string mission_path = default_mission_file_;
        
        // Make path absolute if it's relative
        if (!std::filesystem::path(mission_path).is_absolute()) {
            // Try to find the file relative to the package
            std::string package_path = ament_index_cpp::get_package_share_directory("flyscan_exploration");
            mission_path = package_path + "/" + default_mission_file_;
        }
        
        OperationStatus load_status = LoadMission(mission_path);
        if (load_status != OperationStatus::kOK) {
            RCLCPP_WARN(this->get_logger(), "Failed to load default mission file: %s", mission_path.c_str());
        }
    }
    
    // Auto-start mission if enabled and mission is loaded
    if (auto_start_mission_ && mission_state_ == MissionState::kLoaded) {
        OperationStatus start_status = StartMission();
        if (start_status != OperationStatus::kOK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to auto-start mission");
            return start_status;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Waypoint Mission activated successfully");
    return OperationStatus::kOK;
}

OperationStatus WaypointMission::HandleDeactivate() {
    RCLCPP_INFO(this->get_logger(), "Deactivating Waypoint Mission...");

    // Abort any active mission
    if (mission_state_ != MissionState::kIdle && mission_state_ != MissionState::kCompleted) {
        AbortMission();
    }
    
    // Stop mission timer
    if (mission_timer_) {
        mission_timer_->cancel();
    }
    
    RCLCPP_INFO(this->get_logger(), "Waypoint Mission deactivated successfully");
    return OperationStatus::kOK;
}

OperationStatus WaypointMission::HandleCleanup() {
    RCLCPP_INFO(this->get_logger(), "Cleaning up Waypoint Mission...");
    
    // Reset mission state
    mission_state_ = MissionState::kIdle;
    current_waypoint_index_ = 0;
    
    // Reset components
    mission_timer_.reset();
    set_control_mode_client_.reset();
    navigate_to_pose_client_.reset();
    
    RCLCPP_INFO(this->get_logger(), "Waypoint Mission cleanup complete");
    return OperationStatus::kOK;
}

OperationStatus WaypointMission::HandleShutdown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Waypoint Mission...");
    return HandleCleanup();
}

OperationStatus WaypointMission::HandleError() {
    RCLCPP_ERROR(this->get_logger(), "Waypoint Mission error state - aborting mission...");
    
    // Abort mission on error
    AbortMission();
    
    RCLCPP_INFO(this->get_logger(), "Mission aborted for safety");
    return OperationStatus::kOK;
}

// ============================================================================
// Mission Management Implementation
// ============================================================================

OperationStatus WaypointMission::LoadMission(const std::string& mission_file) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    RCLCPP_INFO(this->get_logger(), "Loading mission from file: %s", mission_file.c_str());
    
    try {
        // Check if file exists
        if (!std::filesystem::exists(mission_file)) {
            RCLCPP_ERROR(this->get_logger(), "Mission file does not exist: %s", mission_file.c_str());
            return OperationStatus::kFileError;
        }
        
        // Load and parse YAML file
        YAML::Node config = YAML::LoadFile(mission_file);
        
        if (!config["mission"]) {
            RCLCPP_ERROR(this->get_logger(), "Invalid mission file: missing 'mission' section");
            return OperationStatus::kFileError;
        }
        
        YAML::Node mission_node = config["mission"];
        
        // Parse mission metadata
        current_mission_.name = mission_node["name"].as<std::string>("unnamed_mission");
        current_mission_.description = mission_node["description"].as<std::string>("");
        
        // Parse mission parameters
        current_mission_.waypoint_tolerance = mission_node["waypoint_tolerance"].as<double>(0.5);
        current_mission_.hover_time = mission_node["hover_time"].as<double>(3.0);
        current_mission_.max_velocity = mission_node["max_velocity"].as<double>(1.0);
        current_mission_.safety_margin = mission_node["safety_margin"].as<double>(1.5);
        current_mission_.completion_action = mission_node["completion_action"].as<std::string>("hover");
        
        // Parse completion position
        if (mission_node["completion_position"]) {
            auto pos = mission_node["completion_position"].as<std::vector<double>>();
            if (pos.size() >= 3) {
                current_mission_.completion_position.x = pos[0];
                current_mission_.completion_position.y = pos[1];
                current_mission_.completion_position.z = pos[2];
            }
        }
        
        // Parse waypoints
        current_mission_.waypoints.clear();
        if (mission_node["waypoints"]) {
            for (const auto& wp_node : mission_node["waypoints"]) {
                Waypoint waypoint = ParseWaypoint(wp_node);
                current_mission_.waypoints.push_back(waypoint);
            }
        }
        
        if (current_mission_.waypoints.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Mission file contains no waypoints");
            return OperationStatus::kFileError;
        }
        
        // Reset mission state
        mission_state_ = MissionState::kLoaded;
        current_waypoint_index_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Mission loaded successfully: '%s' with %zu waypoints",
                   current_mission_.name.c_str(), current_mission_.waypoints.size());
                   
        return OperationStatus::kOK;
        
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse mission file: %s", e.what());
        return OperationStatus::kFileError;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading mission file: %s", e.what());
        return OperationStatus::kFileError;
    }
}

Waypoint WaypointMission::ParseWaypoint(const YAML::Node& node) {
    Waypoint waypoint;
    
    // Parse position
    if (node["position"]) {
        auto pos = node["position"].as<std::vector<double>>();
        if (pos.size() >= 3) {
            waypoint.position.x = pos[0];
            waypoint.position.y = pos[1];
            waypoint.position.z = pos[2];
        }
    }
    
    // Parse orientation
    waypoint.yaw = node["yaw"].as<double>(0.0);
    
    // Parse work time
    waypoint.work_time = node["work_time"].as<double>(current_mission_.hover_time);
    
    // Parse description
    waypoint.description = node["description"].as<std::string>("");
    
    return waypoint;
}

OperationStatus WaypointMission::StartMission() {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    if (mission_state_ != MissionState::kLoaded) {
        RCLCPP_ERROR(this->get_logger(), "Cannot start mission: no mission loaded");
        return OperationStatus::kNotInitialized;
    }
    
    if (current_mission_.waypoints.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot start mission: no waypoints defined");
        return OperationStatus::kNotInitialized;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting mission: %s", current_mission_.name.c_str());
    
    // Reset mission state
    current_waypoint_index_ = 0;
    mission_start_time_ = this->now();
    mission_state_ = MissionState::kSwitchingMode;
    
    // Start mission timer
    mission_timer_->reset();
    
    RCLCPP_INFO(this->get_logger(), "Mission started with %zu waypoints", 
                current_mission_.waypoints.size());
    
    return OperationStatus::kOK;
}

OperationStatus WaypointMission::PauseMission() {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    if (mission_state_ != MissionState::kNavigating && mission_state_ != MissionState::kWorking) {
        RCLCPP_WARN(this->get_logger(), "Cannot pause mission: not in active state");
        return OperationStatus::kInvalidState;
    }
    
    mission_state_ = MissionState::kPaused;
    mission_timer_->cancel();
    
    RCLCPP_INFO(this->get_logger(), "Mission paused");
    return OperationStatus::kOK;
}

OperationStatus WaypointMission::ResumeMission() {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    if (mission_state_ != MissionState::kPaused) {
        RCLCPP_WARN(this->get_logger(), "Cannot resume mission: not paused");
        return OperationStatus::kInvalidState;
    }
    
    mission_state_ = MissionState::kNavigating;
    mission_timer_->reset();
    
    RCLCPP_INFO(this->get_logger(), "Mission resumed");
    return OperationStatus::kOK;
}

OperationStatus WaypointMission::AbortMission() {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    mission_state_ = MissionState::kAborted;
    mission_timer_->cancel();
    
    RCLCPP_WARN(this->get_logger(), "Mission aborted");
    return OperationStatus::kOK;
}

// ============================================================================
// Mission Execution Implementation
// ============================================================================

void WaypointMission::MissionTimerCallback() {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    switch (mission_state_) {
        case MissionState::kSwitchingMode:
            {
                OperationStatus switch_status = SwitchToMissionMode();
                if (switch_status == OperationStatus::kOK) {
                    mission_state_ = MissionState::kNavigating;
                    RCLCPP_INFO(this->get_logger(), "Switched to mission mode, starting navigation");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to switch to mission mode");
                    mission_state_ = MissionState::kAborted;
                }
            }
            break;
            
        case MissionState::kNavigating:
            {
                if (IsCurrentWaypointReached()) {
                    waypoint_arrival_time_ = this->now();
                    mission_state_ = MissionState::kWorking;
                    RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu/%zu: %s", 
                               current_waypoint_index_ + 1, current_mission_.waypoints.size(),
                               current_mission_.waypoints[current_waypoint_index_].description.c_str());
                } else {
                    // Continue navigation - the NavigateToCurrentWaypoint should be called once
                    static size_t last_waypoint_sent = SIZE_MAX;
                    if (last_waypoint_sent != current_waypoint_index_) {
                        NavigateToCurrentWaypoint();
                        last_waypoint_sent = current_waypoint_index_;
                    }
                }
            }
            break;
            
        case MissionState::kWorking:
            {
                auto current_time = this->now();
                double work_duration = (current_time - waypoint_arrival_time_).seconds();
                double required_work_time = current_mission_.waypoints[current_waypoint_index_].work_time;
                
                if (work_duration >= required_work_time) {
                    RCLCPP_INFO(this->get_logger(), "Completed work at waypoint %zu (%.1fs)", 
                               current_waypoint_index_ + 1, work_duration);
                    AdvanceToNextWaypoint();
                }
            }
            break;
            
        case MissionState::kCompleted:
        case MissionState::kAborted:
        case MissionState::kPaused:
        case MissionState::kIdle:
        case MissionState::kLoaded:
            // No action needed for these states
            break;
    }
}

OperationStatus WaypointMission::SwitchToMissionMode() {
    if (!set_control_mode_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Control mode service not ready");
        return OperationStatus::kServiceNA;
    }
    
    auto request = std::make_shared<flyscan_interfaces::srv::SetControlMode::Request>();
    request->mode = 3; // Mission mode
    
    auto future = set_control_mode_client_->async_send_request(request);
    
    // Wait for response with timeout
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 
                                          std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Successfully switched to mission mode");
            return OperationStatus::kOK;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch to mission mode: %s", 
                        response->message.c_str());
            return OperationStatus::kServiceError;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Timeout switching to mission mode");
        return OperationStatus::kTimeout;
    }
}

OperationStatus WaypointMission::NavigateToCurrentWaypoint() {
    if (current_waypoint_index_ >= current_mission_.waypoints.size()) {
        return OperationStatus::kInvalidIndex;
    }
    
    const Waypoint& waypoint = current_mission_.waypoints[current_waypoint_index_];
    
    if (!navigate_to_pose_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Navigation service not ready");
        return OperationStatus::kServiceNA;
    }
    
    // Create navigation request
    auto request = std::make_shared<flyscan_interfaces::srv::NavigateToPose::Request>();
    request->target_pose.header.stamp = this->now();
    request->target_pose.header.frame_id = "map";
    request->target_pose.pose.position = waypoint.position;
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, waypoint.yaw * M_PI / 180.0);
    request->target_pose.pose.orientation = tf2::toMsg(q);
    
    // Set navigation parameters
    request->max_velocity = current_mission_.max_velocity;
    request->safety_margin = current_mission_.safety_margin;
    request->use_obstacle_avoidance = true;
    
    // Send async request
    auto response_callback = [this, waypoint](
        rclcpp::Client<flyscan_interfaces::srv::NavigateToPose>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Started navigation to waypoint %zu: (%.2f, %.2f, %.2f)", 
                           current_waypoint_index_ + 1, waypoint.position.x, 
                           waypoint.position.y, waypoint.position.z);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Navigation request failed: %s", 
                           response->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Navigation service call failed: %s", e.what());
        }
    };
    
    navigate_to_pose_client_->async_send_request(request, response_callback);
    
    return OperationStatus::kOK;
}

bool WaypointMission::IsCurrentWaypointReached() {
    // For now, we'll use a simple time-based approach
    // In a real implementation, you'd check the actual drone position vs waypoint position
    // This is a placeholder - the PX4 controller will handle the actual navigation
    
    // This is simplified - in reality you'd get current position and check distance
    static auto navigation_start_time = this->now();
    static size_t last_waypoint_check = SIZE_MAX;
    
    // Reset timer when waypoint changes
    if (last_waypoint_check != current_waypoint_index_) {
        navigation_start_time = this->now();
        last_waypoint_check = current_waypoint_index_;
        return false;
    }
    
    // Simple time-based check (assumes navigation takes some time)
    auto current_time = this->now();
    double navigation_time = (current_time - navigation_start_time).seconds();
    
    // Estimate navigation time based on distance (simplified)
    return navigation_time > 5.0; // Assume 5 seconds per waypoint for demo
}

void WaypointMission::AdvanceToNextWaypoint() {
    current_waypoint_index_++;
    
    if (current_waypoint_index_ >= current_mission_.waypoints.size()) {
        // Mission completed
        CompleteMission();
    } else {
        // Continue to next waypoint
        mission_state_ = MissionState::kNavigating;
        RCLCPP_INFO(this->get_logger(), "Advancing to waypoint %zu/%zu", 
                   current_waypoint_index_ + 1, current_mission_.waypoints.size());
    }
}

void WaypointMission::CompleteMission() {
    mission_state_ = MissionState::kCompleted;
    mission_timer_->cancel();
    
    auto mission_duration = (this->now() - mission_start_time_).seconds();
    
    RCLCPP_INFO(this->get_logger(), "Mission '%s' completed successfully in %.1f seconds", 
               current_mission_.name.c_str(), mission_duration);
    
    // Handle completion action
    if (current_mission_.completion_action == "hover") {
        RCLCPP_INFO(this->get_logger(), "Mission complete - hovering at final position");
    } else if (current_mission_.completion_action == "land") {
        RCLCPP_INFO(this->get_logger(), "Mission complete - initiating landing");
        // Could send landing command here
    } else if (current_mission_.completion_action == "rtl") {
        RCLCPP_INFO(this->get_logger(), "Mission complete - returning to launch");
        // Could send RTL command here
    }
}

} // namespace exploration
} // namespace flyscan

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    
    flyscan::exploration::WaypointMission::SharedPtr mission_node;

    try {
        mission_node = std::make_shared<flyscan::exploration::WaypointMission>();
        flyscan::common::SetupSigintHandler(mission_node, "waypoint_mission_main");
        
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(mission_node->get_node_base_interface());
        
        auto configure_result = mission_node->configure();
        if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("waypoint_mission_main"), 
                         "Failed to configure WaypointMission");
            return 1;
        }
        
        auto activate_result = mission_node->activate();
        if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("waypoint_mission_main"), 
                         "Failed to activate WaypointMission");
            return 1;
        }
        
        executor.spin();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_mission_main"), "Exception in main: %s", e.what());
        if (mission_node) {
            mission_node->shutdown();
        }
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("waypoint_mission_main"), "WaypointMission main loop completed");
    return 0;
}