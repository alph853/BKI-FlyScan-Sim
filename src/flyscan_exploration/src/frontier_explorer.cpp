#include <algorithm>
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
{
    RCLCPP_INFO(this->get_logger(), "Initializing Frontier Explorer Node: %s", node_name.c_str());
    
    if (!this->has_parameter("exploration_radius")) {
        this->declare_parameter("exploration_radius", 10.0);
    }
    if (!this->has_parameter("min_frontier_size")) {
        this->declare_parameter("min_frontier_size", 5.0);
    }
    if (!this->has_parameter("frontier_cluster_distance")) {
        this->declare_parameter("frontier_cluster_distance", 2.0);
    }
    if (!this->has_parameter("exploration_rate")) {
        this->declare_parameter("exploration_rate", 1.0);
    }

    RCLCPP_INFO(this->get_logger(), "Starting with parameter-based configuration");
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
        min_frontier_size_ = this->get_parameter("min_frontier_size").as_double();
        frontier_cluster_distance_ = this->get_parameter("frontier_cluster_distance").as_double();
        exploration_rate_ = this->get_parameter("exploration_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Cached parameters: radius=%.2f, min_size=%.1f, cluster_dist=%.2f, rate=%.2f",
                   exploration_radius_, min_frontier_size_, frontier_cluster_distance_, exploration_rate_);
    
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
    if (!exploration_active_) {
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
    
    nav_msgs::msg::Odometry::SharedPtr odom;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!current_odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                                "No odometry received yet, waiting...");
            return;
        }
        odom = current_odom_;
    }
    
    auto frontiers = detectFrontiers(*map);
    
    if (frontiers.empty()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
                            "No frontiers detected, exploration may be complete");
        return;
    }

    publishFrontierVisualization(frontiers);

    auto best_frontier = selectBestFrontier(frontiers);
    publishExplorationGoal(best_frontier);

    RCLCPP_INFO(get_logger(), "Published exploration goal to frontier at (%.2f, %.2f) with value %.2f",
               best_frontier.center.x, best_frontier.center.y, best_frontier.exploration_value);
}

std::vector<Frontier> FrontierExplorer::detectFrontiers(const nav_msgs::msg::OccupancyGrid& map) {
    cv_bridge::CvImage cv_image;
    cv_image.header = map.header;
    cv_image.encoding = "mono8";
    cv_image.image = cv::Mat(map.info.height, map.info.width, CV_8UC1);
    
    for (size_t i = 0; i < map.data.size(); ++i) {
        int value = map.data[i];
        if (value == -1) {
            cv_image.image.data[i] = 127;
        } else if (value == 0) {
            cv_image.image.data[i] = 255;
        } else {
            cv_image.image.data[i] = 0;
        }
    }
    
    auto frontier_cells = findFrontierCells(cv_image.image);
    auto frontiers = clusterFrontierCells(frontier_cells, map);
    
    std::vector<Frontier> valid_frontiers;
    for (auto& frontier : frontiers) {
        if (frontier.size >= min_frontier_size_) {
            frontier.distance_to_robot = std::sqrt(
                std::pow(frontier.center.x - robot_position_.x, 2) +
                std::pow(frontier.center.y - robot_position_.y, 2));
            
            if (frontier.distance_to_robot <= exploration_radius_) {
                frontier.exploration_value = calculateExplorationValue(frontier);
                valid_frontiers.push_back(frontier);
            }
        }
    }
    
    return valid_frontiers;
}

std::vector<cv::Point> FrontierExplorer::findFrontierCells(const cv::Mat& map_image) {
    std::vector<cv::Point> frontier_cells;
    
    for (int y = 1; y < map_image.rows - 1; ++y) {
        for (int x = 1; x < map_image.cols - 1; ++x) {
            if (isFrontierCell(x, y, map_image)) {
                frontier_cells.emplace_back(x, y);
            }
        }
    }
    
    return frontier_cells;
}

bool FrontierExplorer::isFrontierCell(int x, int y, const cv::Mat& map) const {
    if (map.at<uint8_t>(y, x) != 127) {
        return false;
    }
    
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = x + dx;
            int ny = y + dy;
            
            if (isValidCell(nx, ny, map) && map.at<uint8_t>(ny, nx) == 255) {
                return true;
            }
        }
    }
    
    return false;
}

bool FrontierExplorer::isValidCell(int x, int y, const cv::Mat& map) const {
    return x >= 0 && x < map.cols && y >= 0 && y < map.rows;
}

std::vector<Frontier> FrontierExplorer::clusterFrontierCells(const std::vector<cv::Point>& frontier_cells,
                                                             const nav_msgs::msg::OccupancyGrid& map) {
    std::vector<Frontier> frontiers;
    std::vector<bool> visited(frontier_cells.size(), false);
    
    for (size_t i = 0; i < frontier_cells.size(); ++i) {
        if (visited[i]) continue;
        
        Frontier frontier;
        std::vector<size_t> cluster_indices;
        std::queue<size_t> queue;
        
        queue.push(i);
        visited[i] = true;
        
        while (!queue.empty()) {
            size_t current = queue.front();
            queue.pop();
            cluster_indices.push_back(current);
            
            for (size_t j = 0; j < frontier_cells.size(); ++j) {
                if (visited[j]) continue;
                
                double distance = std::sqrt(
                    std::pow(frontier_cells[current].x - frontier_cells[j].x, 2) +
                    std::pow(frontier_cells[current].y - frontier_cells[j].y, 2));
                
                if (distance <= frontier_cluster_distance_ / map.info.resolution) {
                    visited[j] = true;
                    queue.push(j);
                }
            }
        }
        
        if (!cluster_indices.empty()) {
            double sum_x = 0, sum_y = 0;
            for (size_t idx : cluster_indices) {
                auto world_point = mapToWorld(frontier_cells[idx].x, frontier_cells[idx].y, map);
                frontier.points.push_back(world_point);
                sum_x += world_point.x;
                sum_y += world_point.y;
            }
            
            frontier.center.x = sum_x / cluster_indices.size();
            frontier.center.y = sum_y / cluster_indices.size();
            frontier.center.z = 0.0;
            frontier.size = cluster_indices.size();
            
            frontiers.push_back(frontier);
        }
    }
    
    return frontiers;
}

Frontier FrontierExplorer::selectBestFrontier(const std::vector<Frontier>& frontiers) {
    if (frontiers.empty()) {
        return Frontier();
    }
    
    auto best_it = std::max_element(frontiers.begin(), frontiers.end(),
        [](const Frontier& a, const Frontier& b) {
            return a.exploration_value < b.exploration_value;
        });
    
    return *best_it;
}

double FrontierExplorer::calculateExplorationValue(const Frontier& frontier) const {
    double size_factor = frontier.size / 100.0;
    double distance_factor = std::max(0.1, 1.0 / (1.0 + frontier.distance_to_robot));
    
    return size_factor * distance_factor;
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

geometry_msgs::msg::Point FrontierExplorer::mapToWorld(int map_x, int map_y, 
                                                       const nav_msgs::msg::OccupancyGrid& map) const {
    geometry_msgs::msg::Point world_point;
    world_point.x = map.info.origin.position.x + map_x * map.info.resolution;
    world_point.y = map.info.origin.position.y + map_y * map.info.resolution;
    world_point.z = 0.0;
    return world_point;
}

cv::Point FrontierExplorer::worldToMap(double world_x, double world_y, 
                                       const nav_msgs::msg::OccupancyGrid& map) const {
    int map_x = static_cast<int>((world_x - map.info.origin.position.x) / map.info.resolution);
    int map_y = static_cast<int>((world_y - map.info.origin.position.y) / map.info.resolution);
    return cv::Point(map_x, map_y);
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