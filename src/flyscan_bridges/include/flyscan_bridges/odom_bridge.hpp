#ifndef FLYSCAN_BRIDGES__ODOM_BRIDGE_HPP_
#define FLYSCAN_BRIDGES__ODOM_BRIDGE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdometryBridge : public rclcpp::Node
{
public:
  OdometryBridge();

private:
  void vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path odom_path_;
  size_t max_path_length_;

  bool debug_;
};

#endif  // FLYSCAN_BRIDGES__ODOM_BRIDGE_HPP_