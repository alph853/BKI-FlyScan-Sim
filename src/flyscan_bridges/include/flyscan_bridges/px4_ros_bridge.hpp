/**
 * @file px4_ros_bridge.hpp
 * @brief Bridge between PX4 and ROS2 for various sensor data and state information
 * @author FlyScan Team
 * @date 2025
 */

#ifndef FLYSCAN_BRIDGES__PX4_ROS_BRIDGE_HPP_
#define FLYSCAN_BRIDGES__PX4_ROS_BRIDGE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

/**
 * @class PX4ROSBridge
 * @brief Bridge class that converts PX4 messages to standard ROS2 messages
 * 
 * This class subscribes to PX4 vehicle odometry messages and publishes
 * corresponding ROS2 nav_msgs/Odometry messages. It handles coordinate
 * frame transformations from NED (North-East-Down) to ENU (East-North-Up).
 */
class PX4ROSBridge : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for PX4ROSBridge
   * 
   * Initializes subscribers, publishers, and parameters for the bridge node.
   */
  PX4ROSBridge();

private:
  /**
   * @brief Callback function for PX4 vehicle odometry messages
   * @param msg Shared pointer to VehicleOdometry message from PX4
   * 
   * Converts PX4 odometry data from NED to ENU coordinate system and
   * publishes as standard ROS2 odometry message.
   */
  void vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  /// Subscription to PX4 vehicle odometry topic
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;
  
  /// Publisher for ROS2 standard odometry messages
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  /// Transform broadcaster for odom→base_link transform
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif  // FLYSCAN_BRIDGES__PX4_ROS_BRIDGE_HPP_