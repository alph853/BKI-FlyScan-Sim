#include "flyscan_drone_controller/px4_ros_node.hpp"

#include <csignal>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto px4_ros_node = std::make_shared<flyscan_drone_controller::PX4RosNode>("");
        
        if (!px4_ros_node->initialize()) {
            RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to initialize PX4 ROS Node");
            return 1;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "PX4 ROS Node running. Use Ctrl+C to exit.");
        
        std::signal(SIGINT, [](int) {
            rclcpp::shutdown();
        });
        
        rclcpp::spin(px4_ros_node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}