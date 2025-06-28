#include "flyscan_vision/slam_node.hpp"

namespace flyscan {

namespace vision {

SlamNode::SlamNode(const std::string& node_name, 
                  const NodeType& node_type, 
                  const std::vector<std::string>& capabilities)
    : BaseNode(node_name, node_type, capabilities)
{
  // Initialize subscriptions, publishers, and other resources here
}

SlamNode::~SlamNode()
{
  // Clean up subscriptions, publishers, and other resources here

}

} // namespace vision

} // namespace flyscan

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<LifeMonitor>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
}