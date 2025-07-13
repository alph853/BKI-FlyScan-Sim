#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <Eigen/Geometry>
#include <visualization_msgs/msg/marker.hpp>

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
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", false);
    }
    
    // Publish standard nav_msgs Odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/world_origin_marker", 10);
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
    odom_msg.pose.pose.position.y = -msg->position[1];
    odom_msg.pose.pose.position.z = -msg->position[2];

    // Orientation (quaternion)
    Eigen::Quaterniond q_ned(msg->q[3], msg->q[0], msg->q[1], msg->q[2]); // PX4 order: w, x, y, z

    Eigen::Quaterniond q_rotate(0, 1, 0, 0); // 180° about X

    Eigen::Quaterniond q_enu = q_rotate * q_ned;

    odom_msg.pose.pose.orientation.x = q_enu.x();
    odom_msg.pose.pose.orientation.y = q_enu.y();
    odom_msg.pose.pose.orientation.z = q_enu.z();
    odom_msg.pose.pose.orientation.w = q_enu.w();

    // Linear velocity
    odom_msg.twist.twist.linear.x = msg->velocity[0];
    odom_msg.twist.twist.linear.y = -msg->velocity[1];
    odom_msg.twist.twist.linear.z = -msg->velocity[2];

    // Angular velocity
    odom_msg.twist.twist.angular.x = msg->angular_velocity[0];
    odom_msg.twist.twist.angular.y = msg->angular_velocity[1];
    odom_msg.twist.twist.angular.z = msg->angular_velocity[2];


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

    auto marker_msg = visualization_msgs::msg::Marker();
    marker_msg.header.stamp = odom_msg.header.stamp;
    marker_msg.header.frame_id = "odom";
    marker_msg.type = visualization_msgs::msg::Marker::ARROW;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.position.x = 0;
    marker_msg.pose.position.y = 0;
    marker_msg.pose.position.z = 0;
    marker_msg.pose.orientation = odom_msg.pose.pose.orientation;
    marker_msg.scale.x = 0.1;
    marker_msg.scale.y = 0.1;
    marker_msg.scale.z = 0.1;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_msg.id = 0;
    marker_msg.lifetime = rclcpp::Duration::from_seconds(10);
    marker_pub_->publish(marker_msg);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
