/**
 * @file types.hpp
 * @brief 
 *
 * 
 *
 * @author UAV team@/flyscan
 * @date 2025/06/18
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "flyscan_interfaces/msg/node_heartbeat.hpp"

#include "flyscan_interfaces/srv/register_node.hpp"
#include "flyscan_interfaces/srv/unregister_node.hpp"
#include "flyscan_interfaces/srv/request_recovery.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace flyscan {

namespace common {

namespace core_types {
    using LifecycleCallbackReturn   = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using LifecycleState            = rclcpp_lifecycle::State;
    using LifecycleStateMsg         = lifecycle_msgs::msg::State;
    using LifecycleTransition       = lifecycle_msgs::msg::Transition;

    using NodeHeartbeatMsg          = flyscan_interfaces::msg::NodeHeartbeat;

    using RegisterNodeSrv       = flyscan_interfaces::srv::RegisterNode;
    using UnregisterNodeSrv     = flyscan_interfaces::srv::UnregisterNode;
    using RequestRecoverySrv    = flyscan_interfaces::srv::RequestRecovery;

}  // namespace core_types

namespace vision_types {
    using ImageMsg          = sensor_msgs::msg::Image;
    using ImuMsg            = sensor_msgs::msg::Imu;
    using OdometryMsg       = nav_msgs::msg::Odometry;

} // namespace vision_types

}  // namespace common

}  // namespace flyscan

