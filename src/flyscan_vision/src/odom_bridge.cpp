#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdometryBridge : public rclcpp::Node
{
public:
  OdometryBridge()
  : Node("odometry_bridge")
  {
    // Subscribe to PX4 VehicleOdometry
    vehicle_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryBridge::vehicleOdomCallback, this, std::placeholders::_1)
    );

    // Publish standard nav_msgs Odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  }

private:
  void vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Position
    odom_msg.pose.pose.position.x = msg->position[0];
    odom_msg.pose.pose.position.y = msg->position[1];
    odom_msg.pose.pose.position.z = msg->position[2];

    // Orientation (quaternion)
    odom_msg.pose.pose.orientation.x = msg->q[0];
    odom_msg.pose.pose.orientation.y = msg->q[1];
    odom_msg.pose.pose.orientation.z = msg->q[2];
    odom_msg.pose.pose.orientation.w = msg->q[3];

    // Linear velocity
    odom_msg.twist.twist.linear.x = msg->velocity[0];
    odom_msg.twist.twist.linear.y = msg->velocity[1];
    odom_msg.twist.twist.linear.z = msg->velocity[2];

    // Angular velocity
    odom_msg.twist.twist.angular.x = msg->angular_velocity[0];
    odom_msg.twist.twist.angular.y = msg->angular_velocity[1];
    odom_msg.twist.twist.angular.z = msg->angular_velocity[2];

    odom_pub_->publish(odom_msg);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
