#include "flyscan_drone_controller/teleop_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

std::shared_ptr<flyscan::drone_controller::TeleopNode> teleop_node = nullptr;

void signalHandler(int signum) {
    if (teleop_node) {
        RCLCPP_INFO(teleop_node->get_logger(), "Caught signal %d, shutting down teleop node", signum);
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Set up signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        teleop_node = std::make_shared<flyscan::drone_controller::TeleopNode>();
        rclcpp::spin(teleop_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("teleop_main"), "Exception caught: %s", e.what());
    }
    
    teleop_node.reset();
    rclcpp::shutdown();
    return 0;
}