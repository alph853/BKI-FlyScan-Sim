#include <cmath>

#include "flyscan_exploration/frontier_explorer.hpp"
#include "flyscan_common/sigint_handler.hpp"

namespace flyscan {
namespace exploration {

FrontierExplorer::FrontierExplorer(const rclcpp::NodeOptions& options,
                                   const std::string& node_name,
                                   const NodeType& node_type,
                                   const std::vector<std::string>& capabilities)
    : BaseNode(options, node_name, node_type, capabilities)
    , exploration_active_(false)
    , exploration_complete_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Smart Frontier Explorer Node: %s", node_name.c_str());
    
    // Original parameters
    if (!this->has_parameter("exploration_radius")) {
        this->declare_parameter("exploration_radius", 10.0);
    }
    if (!this->has_parameter("exploration_rate")) {
        this->declare_parameter("exploration_rate", 1.0);
    }
    
    if (!this->has_parameter("max_frontier_distance")) {
        this->declare_parameter("max_frontier_distance", 50.0);
    }
    if (!this->has_parameter("robot_frame")) {
        this->declare_parameter("robot_frame", "base_link");
    }
    if (!this->has_parameter("map_frame")) {
        this->declare_parameter("map_frame", "map");
    }

    RCLCPP_INFO(this->get_logger(), "Simple Frontier Explorer initialized");
}

FrontierExplorer::~FrontierExplorer() {
    RCLCPP_INFO(this->get_logger(), "FrontierExplorer node destroyed");
}

// ============================================================================
// Lifecycle Management Implementation
// ============================================================================

OperationStatus FrontierExplorer::HandleConfigure() {
    RCLCPP_INFO(this->get_logger(), "Configuring Frontier Explorer...");
    
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    try {
        // Cache ROS parameters
        exploration_radius_ = this->get_parameter("exploration_radius").as_double();
        exploration_rate_ = this->get_parameter("exploration_rate").as_double();
        max_frontier_distance_ = this->get_parameter("max_frontier_distance").as_double();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();

        RCLCPP_INFO(this->get_logger(), "Cached parameters: radius=%.2f, rate=%.2f, max_distance=%.2f",
                   exploration_radius_, exploration_rate_, max_frontier_distance_);
    
        // Initialize TF components
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create subscribers with appropriate QoS
        auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos,
            std::bind(&FrontierExplorer::mapCallback, this, _1));
        
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            std::bind(&FrontierExplorer::odomCallback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Created map and odometry subscribers");
        
        // Create publishers
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/exploration_goal", qos);

        frontier_viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/frontier_markers", qos);

        // Service clients  
        set_control_mode_client_ = this->create_client<flyscan_interfaces::srv::SetControlMode>(
            "/px4_controller/set_control_mode", 
            rclcpp::ServicesQoS() 
        );
        
        RCLCPP_INFO(this->get_logger(), "Created exploration goal and visualization publishers");
        
        // Create exploration timer
        exploration_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / exploration_rate_),
            std::bind(&FrontierExplorer::explorationTimerCallback, this));
        exploration_timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "Created exploration timer (%.2f Hz)", exploration_rate_);
        
        RCLCPP_INFO(this->get_logger(), "Frontier Explorer configured successfully");
        return OperationStatus::kOK;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure Frontier Explorer: %s", e.what());
        return OperationStatus::kNotInitialized;
    }
}

OperationStatus FrontierExplorer::HandleActivate() {
    RCLCPP_INFO(this->get_logger(), "Activating Frontier Explorer...");

    using namespace std::placeholders;
    using namespace std::chrono_literals;

    RCLCPP_INFO(this->get_logger(), "Waiting for service %s...", set_control_mode_client_->get_service_name());
    if (!set_control_mode_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Service %s not available after 30s", set_control_mode_client_->get_service_name());
        return OperationStatus::kServiceNA;
    }

    RCLCPP_INFO(this->get_logger(), "Service available, calling set_control_mode with mode 2...");
    auto request = std::make_shared<flyscan_interfaces::srv::SetControlMode::Request>();
    request->mode = 2;

    // Send async request with callback
    auto response_callback = [this](rclcpp::Client<flyscan_interfaces::srv::SetControlMode>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully switched to autonomous mode");
                // Start exploration
                exploration_active_ = true;
                exploration_timer_->reset();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to switch to autonomous mode: %s", response->message.c_str());
                // Trigger error state transition
                this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Set control mode service call failed: %s", e.what());
            // Trigger error state transition
            this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR);
        }
    };

    set_control_mode_client_->async_send_request(request, response_callback);
    
    RCLCPP_INFO(this->get_logger(), "Set control mode request sent, continuing with activation");
    RCLCPP_INFO(this->get_logger(), "Frontier Explorer activated successfully");
    return OperationStatus::kOK;
}

OperationStatus FrontierExplorer::HandleDeactivate() {
    RCLCPP_INFO(this->get_logger(), "Deactivating Frontier Explorer...");
    
    // Stop exploration
    exploration_active_ = false;
    exploration_timer_->cancel();
    
    RCLCPP_INFO(this->get_logger(), "Frontier Explorer deactivated successfully");
    return OperationStatus::kOK;
}

OperationStatus FrontierExplorer::HandleCleanup() {
    RCLCPP_INFO(this->get_logger(), "Cleaning up Frontier Explorer...");
    
    // Reset all components
    map_subscription_.reset();
    odom_subscription_.reset();
    goal_publisher_.reset();
    frontier_viz_publisher_.reset();
    exploration_timer_.reset();
    tf_buffer_.reset();
    tf_listener_.reset();
    
    RCLCPP_INFO(this->get_logger(), "Frontier Explorer cleanup complete");
    return OperationStatus::kOK;
}

OperationStatus FrontierExplorer::HandleShutdown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Frontier Explorer...");
    return HandleCleanup();
}

OperationStatus FrontierExplorer::HandleError() {
    RCLCPP_ERROR(this->get_logger(), "Frontier Explorer error state - attempting recovery...");
    
    // Stop exploration on error
    exploration_active_ = false;
    if (exploration_timer_) {
        exploration_timer_->cancel();
    }
    
    RCLCPP_INFO(this->get_logger(), "Stopped exploration for safety");
    return OperationStatus::kOK;
}

// ============================================================================
// Core Exploration Methods Implementation
// ============================================================================

void FrontierExplorer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    current_map_ = msg;
}

void FrontierExplorer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = msg;
    robot_position_ = msg->pose.pose.position;
}

void FrontierExplorer::explorationTimerCallback() {
    if (!exploration_active_ || exploration_complete_) {
        return;
    }
    
    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!current_map_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                                "No map received yet, waiting...");
            return;
        }
        map = current_map_;
    }
    
    if (!UpdateRobotPosition()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                            "Could not get robot position, waiting...");
        return;
    }
    
    // Use simple frontier detection
    auto frontiers = DetectSimpleFrontiers(*map);
    
    if (frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "No frontiers detected - exploration complete!");
        exploration_complete_ = true;
        return;
    }
    
    // Select closest frontier
    auto best_frontier = SelectClosestFrontier(frontiers);
    
    if (best_frontier) {
        publishFrontierVisualization({*best_frontier});
        
        publishExplorationGoal(*best_frontier);
        
        RCLCPP_INFO(get_logger(), 
            "Published goal to frontier at (%.2f, %.2f), distance: %.2f",
            best_frontier->center.x, best_frontier->center.y, best_frontier->distance_to_robot);
    } else {
        RCLCPP_INFO(get_logger(), "No suitable frontiers found - exploration complete!");
        exploration_complete_ = true;
    }
}








void FrontierExplorer::publishExplorationGoal(const Frontier& frontier) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = this->now();
    goal.header.frame_id = "map";
    goal.pose.position = frontier.center;
    goal.pose.orientation.w = 1.0;
    
    goal_publisher_->publish(goal);
}

void FrontierExplorer::publishFrontierVisualization(const std::vector<Frontier>& frontiers) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (size_t i = 0; i < frontiers.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "frontiers";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = frontiers[i].center;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        
        marker.color.a = 0.8;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        marker_array.markers.push_back(marker);
    }
    
    frontier_viz_publisher_->publish(marker_array);
}



// ============================================================================
// Simple Frontier Detection Implementation
// ============================================================================

std::vector<Frontier> FrontierExplorer::DetectSimpleFrontiers(const nav_msgs::msg::OccupancyGrid& grid) {
    std::vector<Frontier> frontiers;
    
    int width = grid.info.width;
    int height = grid.info.height;
    
    RCLCPP_DEBUG(get_logger(), "Detecting frontiers in %dx%d grid", width, height);
    
    // Find frontier cells: free cells adjacent to unknown cells
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            int index = y * width + x;
            
            // Check if current cell is free (0-50 range)
            if (grid.data[index] >= 0 && grid.data[index] < 50) {
                
                // Check 8-connected neighbors for unknown cells (-1)
                bool has_unknown_neighbor = false;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int neighbor_x = x + dx;
                        int neighbor_y = y + dy;
                        int neighbor_index = neighbor_y * width + neighbor_x;
                        
                        if (grid.data[neighbor_index] == -1) {
                            has_unknown_neighbor = true;
                            break;
                        }
                    }
                    if (has_unknown_neighbor) break;
                }
                
                // If this free cell has unknown neighbors, it's a frontier
                if (has_unknown_neighbor) {
                    Frontier frontier;
                    
                    // Convert grid coordinates to world coordinates
                    frontier.center.x = grid.info.origin.position.x + x * grid.info.resolution;
                    frontier.center.y = grid.info.origin.position.y + y * grid.info.resolution;
                    frontier.center.z = 0.0;
                    
                    // Calculate distance to robot
                    double dx = frontier.center.x - robot_position_.x;
                    double dy = frontier.center.y - robot_position_.y;
                    frontier.distance_to_robot = sqrt(dx*dx + dy*dy);
                    
                    // Skip if too far
                    if (frontier.distance_to_robot > max_frontier_distance_) {
                        continue;
                    }
                    
                    frontier.size = 1.0;
                    frontier.exploration_value = 1.0 / (1.0 + frontier.distance_to_robot);
                    
                    frontiers.push_back(frontier);
                }
            }
        }
    }
    
    RCLCPP_INFO(get_logger(), "Found %zu raw frontier cells", frontiers.size());
    
    // Simple clustering: group nearby frontiers
    return GroupNearbyFrontiers(frontiers, grid.info.resolution * 5.0); // 5 grid cells
}

std::vector<Frontier> FrontierExplorer::GroupNearbyFrontiers(const std::vector<Frontier>& raw_frontiers, double group_distance) {
    if (raw_frontiers.empty()) return {};
    
    std::vector<Frontier> grouped_frontiers;
    std::vector<bool> used(raw_frontiers.size(), false);
    
    for (size_t i = 0; i < raw_frontiers.size(); ++i) {
        if (used[i]) continue;
        
        Frontier group;
        group.center = raw_frontiers[i].center;
        group.distance_to_robot = raw_frontiers[i].distance_to_robot;
        group.size = 1.0;
        
        double sum_x = raw_frontiers[i].center.x;
        double sum_y = raw_frontiers[i].center.y;
        int count = 1;
        used[i] = true;
        
        // Find nearby frontiers to group together
        for (size_t j = i + 1; j < raw_frontiers.size(); ++j) {
            if (used[j]) continue;
            
            double dx = raw_frontiers[i].center.x - raw_frontiers[j].center.x;
            double dy = raw_frontiers[i].center.y - raw_frontiers[j].center.y;
            double distance = sqrt(dx*dx + dy*dy);
            
            if (distance <= group_distance) {
                sum_x += raw_frontiers[j].center.x;
                sum_y += raw_frontiers[j].center.y;
                count++;
                used[j] = true;
            }
        }
        
        // Update group center to average position
        group.center.x = sum_x / count;
        group.center.y = sum_y / count;
        group.size = count;
        
        // Recalculate distance to robot from group center
        double dx = group.center.x - robot_position_.x;
        double dy = group.center.y - robot_position_.y;
        group.distance_to_robot = sqrt(dx*dx + dy*dy);
        group.exploration_value = group.size / (1.0 + group.distance_to_robot);
        
        grouped_frontiers.push_back(group);
    }
    
    RCLCPP_INFO(get_logger(), "Grouped %zu frontiers into %zu clusters", 
                raw_frontiers.size(), grouped_frontiers.size());
    
    return grouped_frontiers;
}

std::shared_ptr<Frontier> FrontierExplorer::SelectClosestFrontier(const std::vector<Frontier>& frontiers) {
    if (frontiers.empty()) return nullptr;
    
    auto closest = std::min_element(frontiers.begin(), frontiers.end(),
        [](const Frontier& a, const Frontier& b) {
            return a.distance_to_robot < b.distance_to_robot;
        });
    
    return std::make_shared<Frontier>(*closest);
}


bool FrontierExplorer::UpdateRobotPosition() {
    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
        
        robot_position_.x = transform.transform.translation.x;
        robot_position_.y = transform.transform.translation.y;
        robot_position_.z = 0.0;
        
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000, 
                            "Could not get robot position: %s", ex.what());
        return false;
    }
}

} // namespace exploration
} // namespace flyscan



int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    
    flyscan::exploration::FrontierExplorer::SharedPtr explorer;

    try {
        explorer = std::make_shared<flyscan::exploration::FrontierExplorer>();
        flyscan::common::SetupSigintHandler(explorer, "frontier_explorer_main");
        
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(explorer->get_node_base_interface());
        
        auto configure_result = explorer->configure();
        if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("frontier_explorer_main"), 
                         "Failed to configure FrontierExplorer");
            return 1;
        }
        
        auto activate_result = explorer->activate();
        if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("frontier_explorer_main"), 
                         "Failed to activate FrontierExplorer");
            return 1;
        }
        executor.spin();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("frontier_explorer_main"), "Exception in main: %s", e.what());
        if (explorer) {
            explorer->shutdown();
        }
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("frontier_explorer_main"), "FrontierExplorer main loop completed");
    return 0;
}