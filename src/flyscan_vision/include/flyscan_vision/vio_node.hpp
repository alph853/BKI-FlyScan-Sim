#pragma once

#include "flyscan_common/types.hpp"
#include "flyscan_common/enums.hpp"

#include "flyscan_core/base_node.hpp"

namespace flyscan {

namespace vision {

using ImageMsg          = flyscan::common::vision_types::ImageMsg;
using ImuMsg            = flyscan::common::vision_types::ImuMsg;
using OdometryMsg       = flyscan::common::vision_types::OdometryMsg;

using BaseNode          = flyscan::core::BaseNode;
using NodeType          = flyscan::common::NodeType;
using OperationStatus   = flyscan::common::OperationStatus;

class VioNode : public BaseNode {
public:
  VioNode(const std::string& node_name, 
          const NodeType& node_type, 
          const std::vector<std::string>& capabilities);
  ~VioNode();

private:
  /**
   * @brief Callback function for image messages
   * @param msg The image message
   */
  void OnImageCallback(const ImageMsg::ConstSharedPtr& msg);

  /**
   * @brief Callback function for IMU messages
   * @param msg The IMU message
   */
  void OnImuCallback(const ImuMsg::ConstSharedPtr& msg);

  /**
   * @brief Process the data
   * @return The operation status
   */
  OperationStatus ProcessData();

  // Private Attributes
  rclcpp::Subscription<ImageMsg>::SharedPtr   m_image_sub;
  rclcpp::Subscription<ImuMsg>::SharedPtr     m_imu_sub;
  rclcpp::Publisher<OdometryMsg>::SharedPtr   m_odom_pub;

private:
  /**
   * @brief Handle the configuration of the node
   * @return The operation status
   */
  OperationStatus HandleConfigure() override;

  /**
   * @brief Handle the activation of the node
   * @return The operation status
   */
  OperationStatus HandleActivate() override;

  /**
   * @brief Handle the deactivation of the node
   * @return The operation status
   */
  OperationStatus HandleDeactivate() override;

  /**
   * @brief Handle the cleanup of the node
   * @return The operation status
   */
  OperationStatus HandleCleanup() override;

  /**
   * @brief Handle the shutdown of the node
   * @return The operation status
   */
  OperationStatus HandleShutdown() override;
  /**
   * @brief Handle the error state of the node
   * @return The operation status
   */
  OperationStatus HandleError() override;

};

}  // namespace vision

}  // namespace flyscan
