#include "flyscan_bridges/odom_bridge.hpp"

OdometryBridge::OdometryBridge()
: Node("odometry_bridge")
{

  // if (!this->has_parameter("use_sim_time")) {
  //   this->declare_parameter<bool>("use_sim_time", false);
  // }

  // if (!this->has_parameter("debug")) {
  //   this->declare_parameter<bool>("debug", false);
  // }

  // if (!this->has_parameter("max_path_length")) {
  //   this->declare_parameter<int>("max_path_length", 1000);
  // }

  // this->get_parameter("debug", debug_);
  // this->get_parameter("max_path_length", max_path_length_);

  this->declare_parameter<bool>("use_sim_time", false);
  max_path_length_ = this->declare_parameter<int>("max_path_length", 1000);
  debug_ = this->declare_parameter<bool>("debug", false);

  // Subscribe to PX4 VehicleOdometry
  vehicle_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&OdometryBridge::vehicleOdomCallback, this, std::placeholders::_1)
  );

  // Publish standard nav_msgs Odometry
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (debug_) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/odom_path", 10);
  }
}

void OdometryBridge::vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = msg->position[1];  
  odom_msg.pose.pose.position.y = msg->position[0];  
  odom_msg.pose.pose.position.z = -msg->position[2]; 
  
  // Linear velocity: NED to ENU conversion
  odom_msg.twist.twist.linear.x = msg->velocity[1];  // East = NED_Y
  odom_msg.twist.twist.linear.y = msg->velocity[0];  // North = NED_X
  odom_msg.twist.twist.linear.z = -msg->velocity[2]; // Up = -NED_Z
  
  // Angular velocity: NED to ENU conversion
  odom_msg.twist.twist.angular.x = msg->angular_velocity[1];  // East = NED_Y
  odom_msg.twist.twist.angular.y = msg->angular_velocity[0];  // North = NED_X
  odom_msg.twist.twist.angular.z = -msg->angular_velocity[2]; // Up = -NED_Z

  odom_msg.pose.pose.orientation.w = msg->q[0];
  odom_msg.pose.pose.orientation.x = msg->q[2];  // NED Y -> ENU X
  odom_msg.pose.pose.orientation.y = msg->q[1];  // NED X -> ENU Y
  odom_msg.pose.pose.orientation.z = -msg->q[3]; // NED Z -> -ENU Z

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

  if (!debug_) {
    return;
  }
  
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;
  
  odom_path_.header = odom_msg.header;
  odom_path_.poses.push_back(pose_stamped);
  
  if (odom_path_.poses.size() > max_path_length_) {
    odom_path_.poses.erase(odom_path_.poses.begin());
  }
  
  path_pub_->publish(odom_path_);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryBridge>());
  rclcpp::shutdown();
  return 0;
}