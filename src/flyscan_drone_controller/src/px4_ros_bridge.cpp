/**
 * @file px4_ros_bridge.cpp
 * @brief Implementation of PX4 to ROS2 bridge for sensor data and state information
 * @author FlyScan Team
 * @date 2025
 */

#include "flyscan_drone_controller/px4_ros_bridge.hpp"
#include "flyscan_drone_controller/transform.hpp"

PX4ROSBridge::PX4ROSBridge()
: Node("px4_ros_bridge")
{
  using namespace std::placeholders;
  // Declare and get parameters
  if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", false);
  }
  
  // Single drone support - drone ID parameter
  if (!this->has_parameter("drone_id")) {
      this->declare_parameter("drone_id", 0);
  }
  
  drone_id_ = this->get_parameter("drone_id").as_int();
  
  // Subscribe to PX4 VehicleOdometry for this drone
  std::string vehicle_odom_topic = (drone_id_ == 0) ? "/fmu/out/vehicle_odometry" : "/px4_" + std::to_string(drone_id_) + "/fmu/out/vehicle_odometry";
  
  vehicle_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    vehicle_odom_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PX4ROSBridge::vehicleOdomCallback, this, std::placeholders::_1)
  );
  
  // Create odometry publisher for this drone
  std::string odom_topic = (drone_id_ == 0) ? "/odom" : "/px4_" + std::to_string(drone_id_) + "/odom";
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  
  RCLCPP_INFO(this->get_logger(), "Set up bridge for drone %d: %s -> %s", drone_id_, vehicle_odom_topic.c_str(), odom_topic.c_str());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(this->get_logger(), "PX4 ROS Bridge started for drone %d", drone_id_);
}

void PX4ROSBridge::vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // Create ROS2 odometry message
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.header.frame_id = (drone_id_ == 0) ? "odom" : "px4_" + std::to_string(drone_id_) + "_odom";
  odom_msg.child_frame_id = (drone_id_ == 0) ? "base_link" : "px4_" + std::to_string(drone_id_) + "_base_link";
\
  // Position: NED to ENU conversion
  odom_msg.pose.pose.position.x = msg->position[1];
  odom_msg.pose.pose.position.y = msg->position[0];
  odom_msg.pose.pose.position.z = -msg->position[2];
  
  // Linear + angular velocity: FRD to FLU conversion
  odom_msg.twist.twist.linear.x = msg->velocity[0];  
  odom_msg.twist.twist.linear.y = -msg->velocity[1];  
  odom_msg.twist.twist.linear.z = -msg->velocity[2]; 
  odom_msg.twist.twist.angular.x = msg->angular_velocity[0];  
  odom_msg.twist.twist.angular.y = -msg->angular_velocity[1];  
  odom_msg.twist.twist.angular.z = -msg->angular_velocity[2]; 

  // Orientation: NED to FLU quaternion conversion  
  Eigen::Quaterniond q;
  q.w() = msg->q[0];
  q.x() = msg->q[1]; 
  q.y() = msg->q[2];
  q.z() = msg->q[3];
  auto rotated_q = NED_TO_ENU_Q * q * FRD_TO_FLU_Q;

  odom_msg.pose.pose.orientation.w = rotated_q.w();
  odom_msg.pose.pose.orientation.x = rotated_q.x();
  odom_msg.pose.pose.orientation.y = rotated_q.y();
  odom_msg.pose.pose.orientation.z = rotated_q.z();

  // Publish odometry message
  odom_pub_->publish(odom_msg);

    // Broadcast odom to base_link transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = odom_msg.header.stamp;
  transform.header.frame_id = odom_msg.header.frame_id;
  transform.child_frame_id = odom_msg.child_frame_id;
  
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