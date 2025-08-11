#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "flyscan_interfaces/srv/navigate_to_pose.hpp"
#include "flyscan_navigation/navigator.hpp"
#include "flyscan_common/string_util.hpp"


namespace flyscan
{
namespace navigation
{

Navigator::Navigator(const rclcpp::NodeOptions& options,
                    const std::string& node_name,
                    const NodeType& node_type,
                    const std::vector<std::string>& capabilities)
    : BaseNode(options, node_name, node_type, capabilities)
{
}

Navigator::~Navigator()
{
    RCLCPP_INFO(get_logger(), "Navigator destructor called");
}

OperationStatus Navigator::HandleConfigure()
{
    RCLCPP_INFO(this->get_logger(), "Configuring Navigator...");
    
    try {
        drone_id_ = declare_parameter<int>("drone_id", 0);
        
        PlannerConfig planner_config;
        planner_config.max_velocity = declare_parameter<double>("max_velocity", 2.0);
        planner_config.max_acceleration = declare_parameter<double>("max_acceleration", 1.0);
        planner_config.waypoint_tolerance = declare_parameter<double>("waypoint_tolerance", 0.5);
        planner_config.planning_horizon = declare_parameter<double>("planning_horizon", 10.0);
        planner_config.safety_margin = declare_parameter<double>("safety_margin", 1.0);

        CollisionConfig collision_config;
        collision_config.robot_radius = declare_parameter<double>("robot_radius", 0.5);
        collision_config.safety_margin = declare_parameter<double>("safety_margin", 0.3);
        collision_config.min_obstacle_distance = declare_parameter<double>("min_obstacle_distance", 1.0);

        path_planner_ = std::make_unique<PathPlanner>(planner_config);
        collision_checker_ = std::make_unique<CollisionChecker>(collision_config);
        
        configure();
        
        RCLCPP_INFO(this->get_logger(), "Navigator configured successfully with drone_id: %d", drone_id_);
        return OperationStatus::kOK;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure Navigator: %s", e.what());
        return OperationStatus::kNotInitialized;
    }
}

void Navigator::configure()
{
    const std::string ns = (drone_id_ == 0) ? "" : "/drone_" + std::to_string(drone_id_);

    frontiers_sub_ = create_subscription<flyscan_interfaces::msg::FrontierArray>(
        ns + "/frontiers_ranked", 
        rclcpp::QoS(1).reliable(),
        std::bind(&Navigator::frontiers_callback, this, std::placeholders::_1));

    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rtabmap/cloud_obstacles",
        rclcpp::QoS(1).best_effort(),
        std::bind(&Navigator::point_cloud_callback, this, std::placeholders::_1));

    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
        ns + "/navigator/trajectory", 
        rclcpp::QoS(1).reliable());

    heartbeat_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        ns + "/navigator/heartbeat",
        rclcpp::QoS(1).reliable());

    control_mode_client_ = create_client<flyscan_interfaces::srv::SetControlMode>(
        ns + "/px4_controller/set_control_mode");

    follow_path_server_ = rclcpp_action::create_server<FollowPath>(
        this,
        ns + "/navigator/follow_path",
        std::bind(&Navigator::handle_follow_path_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Navigator::handle_follow_path_cancel, this, std::placeholders::_1),
        std::bind(&Navigator::handle_follow_path_accepted, this, std::placeholders::_1));

    navigate_to_pose_server_ = rclcpp_action::create_server<NavigateToPose3D>(
        this,
        ns + "/navigator/navigate_to_pose_3d",
        std::bind(&Navigator::handle_navigate_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Navigator::handle_navigate_to_pose_cancel, this, std::placeholders::_1),
        std::bind(&Navigator::handle_navigate_to_pose_accepted, this, std::placeholders::_1));

    heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            if (autonomous_mode_active_) {
                auto heartbeat_msg = geometry_msgs::msg::PoseStamped();
                heartbeat_msg.header.stamp = now();
                heartbeat_msg.header.frame_id = "map";
                heartbeat_pub_->publish(heartbeat_msg);
            }
        });

}

OperationStatus Navigator::HandleActivate()
{
    RCLCPP_INFO(this->get_logger(), "Activating Navigator...");
    
    activate();
    
    RCLCPP_INFO(this->get_logger(), "Navigator activated successfully");
    return OperationStatus::kOK;
}

void Navigator::activate()
{
    RCLCPP_INFO(get_logger(), "Navigator activated");
}

OperationStatus Navigator::HandleDeactivate()
{
    RCLCPP_INFO(this->get_logger(), "Deactivating Navigator...");
    
    deactivate();
    
    RCLCPP_INFO(this->get_logger(), "Navigator deactivated successfully");
    return OperationStatus::kOK;
}

void Navigator::deactivate()
{
    autonomous_mode_active_ = false;
    RCLCPP_INFO(get_logger(), "Navigator deactivated");
}

OperationStatus Navigator::HandleCleanup()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up Navigator...");
    
    cleanup();
    
    RCLCPP_INFO(this->get_logger(), "Navigator cleanup complete");
    return OperationStatus::kOK;
}

OperationStatus Navigator::HandleShutdown()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Navigator...");
    return HandleCleanup();
}

OperationStatus Navigator::HandleError()
{
    RCLCPP_ERROR(this->get_logger(), "Navigator error state - attempting recovery...");
    
    autonomous_mode_active_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Navigator returned to safe state");
    return OperationStatus::kOK;
}

void Navigator::cleanup()
{
    frontiers_sub_.reset();
    point_cloud_sub_.reset();
    trajectory_pub_.reset();
    heartbeat_pub_.reset();
    control_mode_client_.reset();
    follow_path_server_.reset();
    navigate_to_pose_server_.reset();
    heartbeat_timer_.reset();
}

void Navigator::frontiers_callback(const flyscan_interfaces::msg::FrontierArray::SharedPtr msg)
{
    latest_frontiers_ = msg;
    RCLCPP_DEBUG(get_logger(), "Received %zu frontiers", msg->frontiers.size());
}

void Navigator::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (path_planner_) {
        path_planner_->update_obstacle_cloud(msg);
    }
    if (collision_checker_) {
        collision_checker_->update_obstacle_cloud(msg);
    }
}

rclcpp_action::GoalResponse Navigator::handle_follow_path_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FollowPath::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received follow path goal with %zu waypoints", goal->path.poses.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Navigator::handle_follow_path_cancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Follow path goal cancelled");
    autonomous_mode_active_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Navigator::handle_follow_path_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
    std::thread{&Navigator::execute_follow_path, this, goal_handle}.detach();
}

rclcpp_action::GoalResponse Navigator::handle_navigate_to_pose_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigateToPose3D::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received navigate to pose goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Navigator::handle_navigate_to_pose_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose3D> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Navigate to pose goal cancelled");
    autonomous_mode_active_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Navigator::handle_navigate_to_pose_accepted(const std::shared_ptr<GoalHandleNavigateToPose3D> goal_handle)
{
    std::thread{&Navigator::execute_navigate_to_pose, this, goal_handle}.detach();
}

void Navigator::execute_follow_path(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowPath::Feedback>();
    auto result = std::make_shared<FollowPath::Result>();

    request_control_mode("AUTONOMOUS");
    autonomous_mode_active_ = true;

    publish_trajectory_to_controller(goal->path);

    rclcpp::Rate rate(10);
    size_t current_waypoint = 0;

    while (rclcpp::ok() && current_waypoint < goal->path.poses.size() && autonomous_mode_active_)
    {
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "Goal was cancelled";
            goal_handle->canceled(result);
            autonomous_mode_active_ = false;
            return;
        }

        feedback->current_waypoint = current_waypoint;
        feedback->distance_to_goal = 0.0;
        goal_handle->publish_feedback(feedback);

        current_waypoint++;
        rate.sleep();
    }

    result->success = true;
    result->message = "Path execution completed";
    goal_handle->succeed(result);
    autonomous_mode_active_ = false;
}

void Navigator::execute_navigate_to_pose(const std::shared_ptr<GoalHandleNavigateToPose3D> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPose3D::Feedback>();
    auto result = std::make_shared<NavigateToPose3D::Result>();

    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.stamp = now();
    start_pose.header.frame_id = "map";

    nav_msgs::msg::Path planned_path;
    if (path_planner_) {
        planned_path = path_planner_->plan_to_pose(start_pose, goal->goal_pose);
    }
    
    if (planned_path.poses.empty())
    {
        result->success = false;
        result->message = "Failed to plan path to goal";
        goal_handle->abort(result);
        return;
    }

    request_control_mode("AUTONOMOUS");
    autonomous_mode_active_ = true;

    publish_trajectory_to_controller(planned_path);

    rclcpp::Rate rate(10);

    while (rclcpp::ok() && autonomous_mode_active_)
    {
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "Goal was cancelled";
            goal_handle->canceled(result);
            autonomous_mode_active_ = false;
            return;
        }

        feedback->distance_to_goal = 0.0;
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }

    result->success = true;
    result->message = "Navigation completed";
    goal_handle->succeed(result);
    autonomous_mode_active_ = false;
}

void Navigator::request_control_mode(const std::string& mode)
{
    if (!control_mode_client_->service_is_ready())
    {
        RCLCPP_WARN(get_logger(), "Control mode service not available");
        return;
    }

    auto request = std::make_shared<flyscan_interfaces::srv::SetControlMode::Request>();
    uint8_t mode_value = 0;
    if (mode == "MANUAL") mode_value = 0;
    else if (mode == "TELEOP") mode_value = 1;
    else if (mode == "AUTONOMOUS") mode_value = 2;
    else if (mode == "MISSION") mode_value = 3;
    else if (mode == "RTL") mode_value = 4;
    else if (mode == "LAND") mode_value = 5;
    
    request->mode = mode_value;

    auto future = control_mode_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(get_logger(), "Control mode set to %s", mode.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Failed to set control mode: %s", response->message.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call control mode service");
    }
}

void Navigator::publish_trajectory_to_controller(const nav_msgs::msg::Path& path)
{
    current_trajectory_ = path;
    trajectory_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Published trajectory with %zu waypoints", path.poses.size());
}

} // namespace navigation
} // namespace flyscan