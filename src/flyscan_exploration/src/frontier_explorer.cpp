#include <cmath>
#include <rclcpp/qos.hpp>
#include <rmw/types.h>
#include <stack>
#include <queue>
#include <map>

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
        this->declare_parameter("exploration_radius", 100.0);
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
    // New parameters for optimization
    if (!this->has_parameter("min_frontier_size")) {
        this->declare_parameter("min_frontier_size", 5.0);
    }
    if (!this->has_parameter("dbscan_eps")) {
        this->declare_parameter("dbscan_eps", 3.0);
    }
    if (!this->has_parameter("dbscan_min_points")) {
        this->declare_parameter("dbscan_min_points", 3);
    }
    if (!this->has_parameter("information_radius")) {
        this->declare_parameter("information_radius", 8.0);
    }
    if (!this->has_parameter("size_weight")) {
        this->declare_parameter("size_weight", 0.3);
    }
    if (!this->has_parameter("information_weight")) {
        this->declare_parameter("information_weight", 0.4);
    }
    if (!this->has_parameter("min_utility_threshold")) {
        this->declare_parameter("min_utility_threshold", 0.2);
    }
    if (!this->has_parameter("distance_weight")) {
        this->declare_parameter("distance_weight", 0.3);
    }
    if (!this->has_parameter("prefer_far_frontiers")) {
        this->declare_parameter("prefer_far_frontiers", true);
    }
    if (!this->has_parameter("min_exploration_distance")) {
        this->declare_parameter("min_exploration_distance", 5.0);
    }
    if (!this->has_parameter("frontier_spread_factor")) {
        this->declare_parameter("frontier_spread_factor", 2.0);
    }
    
    // Similarity detection parameters
    if (!this->has_parameter("similarity_threshold")) {
        this->declare_parameter("similarity_threshold", 2.0);
    }
    if (!this->has_parameter("similarity_time_threshold")) {
        this->declare_parameter("similarity_time_threshold", 30.0);
    }
    
    // Removed camera cone filtering parameters - safe navigation now handles optimal positioning

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

    // Cache ROS parameters
    exploration_radius_ = this->get_parameter("exploration_radius").as_double();
    exploration_rate_ = this->get_parameter("exploration_rate").as_double();
    max_frontier_distance_ = this->get_parameter("max_frontier_distance").as_double();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    
    // Cache new parameters
    min_frontier_size_ = this->get_parameter("min_frontier_size").as_double();
    dbscan_eps_ = this->get_parameter("dbscan_eps").as_double();
    dbscan_min_points_ = this->get_parameter("dbscan_min_points").as_int();
    information_radius_ = this->get_parameter("information_radius").as_double();
    size_weight_ = this->get_parameter("size_weight").as_double();
    information_weight_ = this->get_parameter("information_weight").as_double();
    min_utility_threshold_ = this->get_parameter("min_utility_threshold").as_double();
    distance_weight_ = this->get_parameter("distance_weight").as_double();
    prefer_far_frontiers_ = this->get_parameter("prefer_far_frontiers").as_bool();
    min_exploration_distance_ = this->get_parameter("min_exploration_distance").as_double();
    frontier_spread_factor_ = this->get_parameter("frontier_spread_factor").as_double();
    
    // Cache similarity detection parameters
    similarity_threshold_ = this->get_parameter("similarity_threshold").as_double();
    similarity_time_threshold_ = this->get_parameter("similarity_time_threshold").as_double();
    
    // Removed camera cone filtering parameters - safe navigation now handles optimal positioning

    RCLCPP_INFO(this->get_logger(), "Cached parameters: radius=%.2f, rate=%.2f, max_distance=%.2f",
                exploration_radius_, exploration_rate_, max_frontier_distance_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    auto qos = rclcpp::SensorDataQoS();
    
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos,
        std::bind(&FrontierExplorer::MapCallback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "Created map subscriber (removed odometry for optimization)");
    
    // Create publishers
    frontiers_publisher_ = this->create_publisher<flyscan_interfaces::msg::FrontierArray>(
        "/frontiers_ranked", qos);

    // Service clients  
    set_control_mode_client_ = this->create_client<flyscan_interfaces::srv::SetControlMode>(
        "/px4_controller/set_control_mode", 
        rclcpp::ServicesQoS() 
    );
    
    RCLCPP_INFO(this->get_logger(), "Created frontiers publisher");
    
    // Create exploration timer
    exploration_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / exploration_rate_),
        std::bind(&FrontierExplorer::ExplorationTimerCallback, this));
    exploration_timer_->cancel();
    
    RCLCPP_INFO(this->get_logger(), "Created exploration timer (%.2f Hz)", exploration_rate_);
    
    RCLCPP_INFO(this->get_logger(), "Frontier Explorer configured successfully");
    return OperationStatus::kOK;
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

    RCLCPP_INFO(this->get_logger(), "Service available, calling set_control_mode with autonomous mode...");
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

void FrontierExplorer::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    current_map_ = msg;
}


void FrontierExplorer::ExplorationTimerCallback() {
    if (!exploration_active_ || exploration_complete_) {
        return;
    }
    
    // ==== STAGE 1: MAP AND POSITION VALIDATION ====
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
    
    RCLCPP_DEBUG(get_logger(), "Stage 1: Map and position validation complete");
    
    // ==== STAGE 2: FRONTIER DETECTION WITH OPTIMIZED DFS ====
    auto frontiers = DetectFrontiers(*map);
    
    RCLCPP_DEBUG(get_logger(), "Stage 2: Detected %zu raw frontiers using DFS", frontiers.size());
    
    // ==== STAGE 3: DBSCAN CLUSTERING ====
    // (DBSCAN is already integrated into DetectFrontiers, but we log it separately here)
    if (!frontiers.empty()) {
        RCLCPP_DEBUG(get_logger(), "Stage 3: DBSCAN clustering produced %zu valid frontier clusters", frontiers.size());
    }
    
    // ==== STAGE 4: FRONTIER VALIDATION (ROI removed) ====
    RCLCPP_DEBUG(get_logger(), "Stage 4: Found %zu frontiers for validation", frontiers.size());
    
    // ==== STAGE 5: FRONTIER VALIDATION AND COMPLETION CHECK ====
    if (frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "Stage 5: No valid frontiers detected - exploration complete!");
        exploration_complete_ = true;
        return;
    }
    
    // ==== STAGE 6: UTILITY CALCULATION AND FRONTIER RANKING ====
    auto ranked_frontiers = RankFrontiersByUtility(frontiers, *map);
    
    if (ranked_frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "Stage 6: No suitable frontiers found (below utility threshold) - exploration complete!");
        exploration_complete_ = true;
        return;
    }
    
    RCLCPP_DEBUG(get_logger(), "Stage 6: Ranked %zu frontiers in decreasing order of utility", ranked_frontiers.size());
    
    // ==== STAGE 7: PUBLISH RANKED FRONTIERS ====
    PublishRankedFrontiers(ranked_frontiers);
    
    RCLCPP_INFO(get_logger(), 
        "Stage 7: Complete - Published %zu ranked frontiers, best frontier at (%.2f, %.2f), utility: %.3f",
        ranked_frontiers.size(), ranked_frontiers[0].center.x, ranked_frontiers[0].center.y, 
        ranked_frontiers[0].utility_score);
}


// ============================================================================
// Optimized Frontier Detection Implementation
// ============================================================================

std::vector<Frontier> FrontierExplorer::DetectFrontiers(const nav_msgs::msg::OccupancyGrid& grid) {
    int width = grid.info.width;
    int height = grid.info.height;
    
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    std::vector<cv::Point> all_frontier_points;
    
    RCLCPP_DEBUG(get_logger(), "Detecting frontiers in %dx%d grid", 
                width, height);
    
    // DFS flood-fill to find frontier points
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            if (visited[y][x]) continue;
            
            int index = y * width + x;
            // Check if current cell is free
            if (grid.data[index] >= 0 && grid.data[index] < 50) {
                // Check if it has unknown neighbors
                bool has_unknown_neighbor = false;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int neighbor_index = (y + dy) * width + (x + dx);
                        if (grid.data[neighbor_index] == -1) {
                            has_unknown_neighbor = true;
                            break;
                        }
                    }
                    if (has_unknown_neighbor) break;
                }
                
                if (has_unknown_neighbor) {
                    std::vector<cv::Point> cluster_points;
                    DfsFloodFill(grid, x, y, visited, cluster_points);
                    
                    // Add points to global frontier list
                    all_frontier_points.insert(all_frontier_points.end(), 
                                             cluster_points.begin(), cluster_points.end());
                }
            }
        }
    }
    
    RCLCPP_INFO(get_logger(), "Found %zu raw frontier points", all_frontier_points.size());
    
    // Apply DBSCAN clustering
    return DbscanClustering(all_frontier_points, dbscan_eps_, dbscan_min_points_, grid);
}

void FrontierExplorer::DfsFloodFill(const nav_msgs::msg::OccupancyGrid& grid, int start_x, int start_y,
                                   std::vector<std::vector<bool>>& visited, 
                                   std::vector<cv::Point>& frontier_points) {
    std::stack<cv::Point> stack;
    stack.push(cv::Point(start_x, start_y));
    
    int width = grid.info.width;
    int height = grid.info.height;
    
    while (!stack.empty()) {
        cv::Point current = stack.top();
        stack.pop();
        
        int x = current.x;
        int y = current.y;
        
        if (x < 1 || x >= width-1 || y < 1 || y >= height-1 || visited[y][x]) {
            continue;
        }
        
        int index = y * width + x;
        if (grid.data[index] < 0 || grid.data[index] >= 50) {
            continue;
        }
        
        visited[y][x] = true;
        frontier_points.push_back(cv::Point(x, y));
        
        // Add 4-connected neighbors to stack for more controlled expansion
        stack.push(cv::Point(x+1, y));
        stack.push(cv::Point(x-1, y));
        stack.push(cv::Point(x, y+1));
        stack.push(cv::Point(x, y-1));
    }
}

std::vector<Frontier> FrontierExplorer::DbscanClustering(const std::vector<cv::Point>& points, 
                                                        double eps, int min_points,
                                                        const nav_msgs::msg::OccupancyGrid& grid) {
    if (points.empty()) return {};
    
    std::vector<DBSCANPoint> dbscan_points;
    for (const auto& point : points) {
        dbscan_points.emplace_back(point);
    }
    
    int cluster_id = 0;
    
    for (size_t i = 0; i < dbscan_points.size(); ++i) {
        if (dbscan_points[i].visited) continue;
        
        dbscan_points[i].visited = true;
        std::vector<size_t> neighbors = GetNeighbors(points, i, eps);
        
        if (neighbors.size() < static_cast<size_t>(min_points)) {
            // Mark as noise (cluster_id remains -1)
            continue;
        }
        
        // Start new cluster
        dbscan_points[i].cluster_id = cluster_id;
        dbscan_points[i].is_core = true;
        
        // Process neighbors
        std::queue<size_t> seed_set;
        for (size_t neighbor_idx : neighbors) {
            seed_set.push(neighbor_idx);
        }
        
        while (!seed_set.empty()) {
            size_t current_idx = seed_set.front();
            seed_set.pop();
            
            if (!dbscan_points[current_idx].visited) {
                dbscan_points[current_idx].visited = true;
                std::vector<size_t> current_neighbors = GetNeighbors(points, current_idx, eps);
                
                if (current_neighbors.size() >= static_cast<size_t>(min_points)) {
                    dbscan_points[current_idx].is_core = true;
                    for (size_t neighbor_idx : current_neighbors) {
                        seed_set.push(neighbor_idx);
                    }
                }
            }
            
            if (dbscan_points[current_idx].cluster_id == -1) {
                dbscan_points[current_idx].cluster_id = cluster_id;
            }
        }
        
        cluster_id++;
    }
    
    // Convert clusters to Frontier objects
    std::map<int, std::vector<cv::Point>> clusters;
    for (const auto& point : dbscan_points) {
        if (point.cluster_id >= 0) {
            clusters[point.cluster_id].push_back(point.position);
        }
    }
    
    std::vector<Frontier> frontiers;
    for (const auto& [id, cluster_points] : clusters) {
        if (cluster_points.size() >= static_cast<size_t>(min_frontier_size_)) {
            Frontier frontier;
            frontier.cluster_id = id;
            frontier.size = cluster_points.size();
            frontier.pixel_points = cluster_points;
            
            // Calculate centroid in world coordinates
            double sum_x = 0.0, sum_y = 0.0;
            for (const auto& point : cluster_points) {
                sum_x += grid.info.origin.position.x + point.x * grid.info.resolution;
                sum_y += grid.info.origin.position.y + point.y * grid.info.resolution;
            }
            
            frontier.center.x = sum_x / cluster_points.size();
            frontier.center.y = sum_y / cluster_points.size();
            frontier.center.z = 0.0;
            
            // Calculate distance to robot
            double dx = frontier.center.x - robot_position_.x;
            double dy = frontier.center.y - robot_position_.y;
            frontier.distance_to_robot = sqrt(dx*dx + dy*dy);
            
            // Filter out frontiers that are too close (encourage exploration)
            if (frontier.distance_to_robot >= min_exploration_distance_) {
                frontiers.push_back(frontier);
                RCLCPP_DEBUG(get_logger(), "Added frontier %d at (%.2f, %.2f), distance: %.2f", 
                           id, frontier.center.x, frontier.center.y, frontier.distance_to_robot);
            } else {
                RCLCPP_DEBUG(get_logger(), "Filtered out close frontier %d at (%.2f, %.2f), distance: %.2f < %.2f", 
                           id, frontier.center.x, frontier.center.y, frontier.distance_to_robot, min_exploration_distance_);
            }
        }
    }
    
    // Log frontier range information for debugging
    if (!frontiers.empty()) {
        double min_x = frontiers[0].center.x, max_x = frontiers[0].center.x;
        double min_y = frontiers[0].center.y, max_y = frontiers[0].center.y;
        double min_dist = frontiers[0].distance_to_robot, max_dist = frontiers[0].distance_to_robot;
        
        for (const auto& f : frontiers) {
            min_x = std::min(min_x, f.center.x); max_x = std::max(max_x, f.center.x);
            min_y = std::min(min_y, f.center.y); max_y = std::max(max_y, f.center.y);
            min_dist = std::min(min_dist, f.distance_to_robot); max_dist = std::max(max_dist, f.distance_to_robot);
        }
        
        RCLCPP_INFO(get_logger(), "DBSCAN clustering: %zu points -> %zu clusters -> %zu valid frontiers", 
                    points.size(), clusters.size(), frontiers.size());
        RCLCPP_INFO(get_logger(), "Frontier ranges - X: [%.2f, %.2f], Y: [%.2f, %.2f], Distance: [%.2f, %.2f]", 
                    min_x, max_x, min_y, max_y, min_dist, max_dist);
        RCLCPP_INFO(get_logger(), "Robot position: (%.2f, %.2f)", robot_position_.x, robot_position_.y);
    } else {
        RCLCPP_INFO(get_logger(), "DBSCAN clustering: %zu points -> %zu clusters -> 0 valid frontiers", 
                    points.size(), clusters.size());
    }
    
    return frontiers;
}

double FrontierExplorer::CalculateUtility(Frontier& frontier, const nav_msgs::msg::OccupancyGrid& grid) {
    // Distance score - configurable preference for far vs near frontiers
    double distance_score;
    if (prefer_far_frontiers_) {
        // Strongly prefer farther frontiers with exponential scaling
        double normalized_distance = frontier.distance_to_robot / max_frontier_distance_;
        distance_score = std::min(std::pow(normalized_distance, frontier_spread_factor_), 1.0);
    } else {
        // Prefer closer frontiers
        distance_score = 1.0 / (1.0 + frontier.distance_to_robot / 10.0);
    }
    
    // Size score (larger frontiers are better, normalized to max 20 points)
    double size_score = std::min(frontier.size / 20.0, 1.0);
    
    // Information gain score
    int unknown_count = 0, total_count = 0;
    int radius_cells = static_cast<int>(information_radius_ / grid.info.resolution);
    
    // Convert frontier center to grid coordinates
    int center_x = static_cast<int>((frontier.center.x - grid.info.origin.position.x) / grid.info.resolution);
    int center_y = static_cast<int>((frontier.center.y - grid.info.origin.position.y) / grid.info.resolution);
    
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx*dx + dy*dy <= radius_cells*radius_cells) {
                int x = center_x + dx;
                int y = center_y + dy;
                
                if (x >= 0 && x < static_cast<int>(grid.info.width) && 
                    y >= 0 && y < static_cast<int>(grid.info.height)) {
                    
                    int index = y * grid.info.width + x;
                    if (grid.data[index] == -1) {
                        unknown_count++;
                    }
                    total_count++;
                }
            }
        }
    }
    
    double information_gain = (total_count > 0) ? static_cast<double>(unknown_count) / total_count : 0.0;
    
    // Combined utility function (removed camera cone filtering since safe navigation handles positioning)
    double utility = 
                    distance_weight_ * distance_score +
                    size_weight_ * size_score + 
                    information_weight_ * information_gain;
    
    frontier.distance_score = distance_score;
    frontier.size_score = size_score;
    frontier.information_gain = information_gain;
    frontier.utility_score = utility;
    
    return utility;
}

std::vector<Frontier> FrontierExplorer::RankFrontiersByUtility(std::vector<Frontier>& frontiers,
                                                                  const nav_msgs::msg::OccupancyGrid& grid) {
    if (frontiers.empty()) return {};
    
    // Calculate utility for all frontiers
    for (auto& frontier : frontiers) {
        CalculateUtility(frontier, grid);
    }
    
    // Sort frontiers by utility score (highest first)
    std::sort(frontiers.begin(), frontiers.end(),
        [](const Frontier& a, const Frontier& b) {
            return a.utility_score > b.utility_score;
        });
    
    // Filter out frontiers below utility threshold and similar ones
    std::vector<Frontier> ranked_frontiers;
    
    for (auto& frontier : frontiers) {
        if (frontier.utility_score < min_utility_threshold_) {
            RCLCPP_DEBUG(get_logger(), "Frontier below utility threshold %.3f, score: %.3f", 
                        min_utility_threshold_, frontier.utility_score);
            break; // Since sorted by utility, all remaining will be below threshold
        }
        
        // Check if this frontier is too similar to already selected frontiers
        // Use dynamic similarity threshold based on exploration progress
        double dynamic_threshold = similarity_threshold_ * (1.0 + ranked_frontiers.size() * 0.5);
        bool is_too_similar = false;
        for (const auto& existing_frontier : ranked_frontiers) {
            double dx = frontier.center.x - existing_frontier.center.x;
            double dy = frontier.center.y - existing_frontier.center.y;
            double distance = sqrt(dx*dx + dy*dy);
            
            if (distance <= dynamic_threshold) {
                is_too_similar = true;
                RCLCPP_DEBUG(get_logger(), "Skipping similar frontier at (%.2f, %.2f), too close to (%.2f, %.2f) (threshold: %.2f)", 
                           frontier.center.x, frontier.center.y, existing_frontier.center.x, existing_frontier.center.y, dynamic_threshold);
                break;
            }
        }
        
        if (is_too_similar) {
            continue;  // Skip this frontier and try the next one
        }
        
        // Also check against historical frontiers for the best frontier only
        if (ranked_frontiers.empty() && DetectSimilarFrontiers(frontier.center)) {
            RCLCPP_INFO(get_logger(), "Skipping historically similar best frontier at (%.2f, %.2f) with utility %.3f", 
                       frontier.center.x, frontier.center.y, frontier.utility_score);
            continue;  // Skip this frontier and try the next one
        }
        
        // Add frontier to ranked list
        ranked_frontiers.push_back(frontier);
        
        RCLCPP_DEBUG(get_logger(), "Added frontier %zu: utility=%.3f (dist=%.3f, size=%.3f, info=%.3f) at (%.2f, %.2f)", 
                    ranked_frontiers.size(), frontier.utility_score, frontier.distance_score, 
                    frontier.size_score, frontier.information_gain, frontier.center.x, frontier.center.y);
    }
    
    if (ranked_frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "No suitable frontiers found - all were similar or below threshold");
    } else {
        RCLCPP_INFO(get_logger(), "Ranked %zu frontiers in decreasing utility order", ranked_frontiers.size());
    }
    
    return ranked_frontiers;
}

void FrontierExplorer::PublishRankedFrontiers(const std::vector<Frontier>& ranked_frontiers) {
    flyscan_interfaces::msg::FrontierArray frontiers_msg;
    frontiers_msg.header.stamp = this->now();
    frontiers_msg.header.frame_id = "map";
    
    for (const auto& frontier : ranked_frontiers) {
        flyscan_interfaces::msg::Frontier frontier_msg;
        frontier_msg.center = frontier.center;
        frontier_msg.size = frontier.size;
        frontier_msg.utility_score = frontier.utility_score;
        frontier_msg.distance_score = frontier.distance_score;
        frontier_msg.size_score = frontier.size_score;
        frontier_msg.information_gain = frontier.information_gain;
        frontier_msg.distance_to_robot = frontier.distance_to_robot;
        
        frontiers_msg.frontiers.push_back(frontier_msg);
    }
    
    frontiers_publisher_->publish(frontiers_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Published %zu ranked frontiers", ranked_frontiers.size());
}


std::vector<size_t> FrontierExplorer::GetNeighbors(const std::vector<cv::Point>& points, 
                                                   size_t point_idx, double eps) {
    std::vector<size_t> neighbors;
    const cv::Point& query_point = points[point_idx];
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (i == point_idx) continue;
        
        const cv::Point& point = points[i];
        double dx = query_point.x - point.x;
        double dy = query_point.y - point.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance <= eps) {
            neighbors.push_back(i);
        }
    }
    
    return neighbors;
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


bool FrontierExplorer::DetectSimilarFrontiers(const geometry_msgs::msg::Point& new_frontier) {
    rclcpp::Time current_time = this->now();
    
    // Add new frontier to history
    frontier_history_.emplace_back(new_frontier, current_time);
    
    // Keep only recent history (within time threshold)
    auto cutoff_time = current_time - rclcpp::Duration::from_seconds(similarity_time_threshold_);
    frontier_history_.erase(
        std::remove_if(frontier_history_.begin(), frontier_history_.end(),
                      [cutoff_time](const FrontierHistory& fh) {
                          return fh.timestamp < cutoff_time;
                      }),
        frontier_history_.end());
    
    // Need at least 3 frontiers to detect similarity
    if (frontier_history_.size() < 3) {
        return false;
    }
    
    // Check if recent frontiers are all within similarity threshold
    int similar_count = 0;
    for (const auto& history_frontier : frontier_history_) {
        double dx = new_frontier.x - history_frontier.position.x;
        double dy = new_frontier.y - history_frontier.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        if (distance <= similarity_threshold_) {
            similar_count++;
        }
    }
    
    // If more than 80% of recent frontiers are similar, consider stuck
    double similarity_ratio = static_cast<double>(similar_count) / frontier_history_.size();
    bool is_stuck = similarity_ratio > 0.8;
    
    if (is_stuck) {
        RCLCPP_WARN(this->get_logger(), "Stuck condition detected: %.1f%% of %zu recent frontiers within %.2fm", 
                   similarity_ratio * 100.0, frontier_history_.size(), similarity_threshold_);
    }
    
    return is_stuck;
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