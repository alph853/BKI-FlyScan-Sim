#include "flyscan_navigation/navigator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto navigator = std::make_shared<flyscan::navigation::Navigator>(options, "navigator");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(navigator->get_node_base_interface());
    
    executor.spin();

    rclcpp::shutdown();
    return 0;
}