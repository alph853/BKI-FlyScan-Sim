#include "flyscan_vision/vio_node.hpp"
#include "flyscan_common/constants.hpp"

namespace flyscan {

namespace vision {

VioNode::VioNode(const std::string& node_name,
                const NodeType& node_type, 
                const std::vector<std::string>& capabilities)
: BaseNode(node_name, node_type, capabilities) 
{

  using namespace std::placeholders;
  using flyscan::common::constants::topic::CAMERA_IMAGE;
  using flyscan::common::constants::topic::DEPTH_IMAGE;
  using flyscan::common::constants::topic::IMU;
  using flyscan::common::constants::topic::ODOM;

  m_image_sub = create_subscription<ImageMsg>(CAMERA_IMAGE, 10,
      std::bind(&VioNode::OnImageCallback, this, _1));
  m_imu_sub = create_subscription<ImuMsg>(DEPTH_IMAGE, 10,
      std::bind(&VioNode::OnImuCallback, this, _1));
  m_odom_pub = create_publisher<OdometryMsg>(ODOM, 10);

}

VioNode::~VioNode() {
  m_image_sub.reset();
  m_imu_sub.reset();
  m_odom_pub.reset();
}

OperationStatus VioNode::ProcessData() {
  OdometryMsg odom;
  odom.header.stamp = now();
  odom.header.frame_id = "vio_odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.orientation.w = 1.0;

  m_odom_pub->publish(odom);

  return OperationStatus::kOK;
}

void VioNode::OnImageCallback(const ImageMsg::ConstSharedPtr& msg) {
  ProcessData();
}

void VioNode::OnImuCallback(const ImuMsg::ConstSharedPtr& msg) {
  ProcessData();
}

OperationStatus VioNode::HandleConfigure() {
  return OperationStatus::kOK;
}

OperationStatus VioNode::HandleActivate() {
  return OperationStatus::kOK;
}

OperationStatus VioNode::HandleDeactivate() {
  return OperationStatus::kOK;
}

OperationStatus VioNode::HandleCleanup() {
  return OperationStatus::kOK;
}

OperationStatus VioNode::HandleShutdown() {
  return OperationStatus::kOK;
}

OperationStatus VioNode::HandleError() {
  return OperationStatus::kOK;
}

}  // namespace vision

}  // namespace flyscan
  

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  std::vector<std::string> caps = {};
  flyscan::common::NodeType node_type = flyscan::common::NodeType::kVision;

  auto node = std::make_shared<flyscan::vision::VioNode>("viso_node", node_type, caps);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());

  try {
    exec.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Unknown exception occurred");
  }
  rclcpp::shutdown();

  return 0;
}