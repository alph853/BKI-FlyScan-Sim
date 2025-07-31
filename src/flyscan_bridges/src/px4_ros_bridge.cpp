/**
 * @file px4_ros_bridge.cpp
 * @brief Implementation of PX4 to ROS2 bridge for sensor data and state information
 * @author FlyScan Team
 * @date 2025
 */

#include "flyscan_bridges/px4_ros_bridge.hpp"

PX4ROSBridge::PX4ROSBridge()
: Node("px4_ros_bridge")
{
  // Declare and get parameters
  if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", false);
  }

  // Subscribe to PX4 VehicleOdometry
  vehicle_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&PX4ROSBridge::vehicleOdomCallback, this, std::placeholders::_1)
  );

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(this->get_logger(), "PX4 ROS Bridge started");
}

void PX4ROSBridge::vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // Create ROS2 odometry message
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // Position: NED to ENU conversion
  odom_msg.pose.pose.position.x = msg->position[1];  // East = NED_Y
  odom_msg.pose.pose.position.y = msg->position[0];  // North = NED_X
  odom_msg.pose.pose.position.z = -msg->position[2]; // Up = -NED_Z
  
  // Linear velocity: NED to ENU conversion
  odom_msg.twist.twist.linear.x = msg->velocity[1];  // East = NED_Y
  odom_msg.twist.twist.linear.y = msg->velocity[0];  // North = NED_X
  odom_msg.twist.twist.linear.z = -msg->velocity[2]; // Up = -NED_Z
  
  // Angular velocity: NED to ENU conversion
  odom_msg.twist.twist.angular.x = msg->angular_velocity[1];  // East = NED_Y
  odom_msg.twist.twist.angular.y = msg->angular_velocity[0];  // North = NED_X
  odom_msg.twist.twist.angular.z = -msg->angular_velocity[2]; // Up = -NED_Z

  // Orientation: NED to ENU quaternion conversion
  odom_msg.pose.pose.orientation.w = msg->q[0];
  odom_msg.pose.pose.orientation.x = msg->q[2];  // NED Y -> ENU X
  odom_msg.pose.pose.orientation.y = msg->q[1];  // NED X -> ENU Y
  odom_msg.pose.pose.orientation.z = -msg->q[3]; // NED Z -> -ENU Z

  // Publish odometry message
  odom_pub_->publish(odom_msg);

    // Broadcast odom to base_link transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = odom_msg.header.stamp;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  
  transform.transform.translation.x = odom_msg.pose.pose.position.x; 
  transform.transform.translation.y = odom_msg.pose.pose.position.y;
  transform.transform.translation.z = odom_msg.pose.pose.position.z;
  
  transform.transform.rotation = odom_msg.pose.pose.orientation;
  
  tf_broadcaster_->sendTransform(transform);
}

/**
 * @brief Main function for PX4 ROS Bridge node
 * @param argc Command line argument count
 * @param argv Command line arguments
 * @return Exit status
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4ROSBridge>());
  rclcpp::shutdown();
  return 0;
}