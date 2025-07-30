#include "flyscan_sim/marker_publisher.hpp"

MarkerPublisher::MarkerPublisher() : Node("marker_publisher")
{
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization_marker_array", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&MarkerPublisher::publish_markers, this));
        
    RCLCPP_INFO(this->get_logger(), "Marker publisher node started");
}

void MarkerPublisher::publish_markers()
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = this->get_clock()->now();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Publish coordinate system at map origin (0,0,0)
    add_coordinate_system(marker_array, 0.0, 0.0, 0.0, "map_origin", 0);
    // add_coordinate_system(marker_array, -2.0, 2.0, 0.24, "omega1", 90);
    
    // // Add text labels
    add_text_marker(marker_array, 0.0, 0.0, 2.0, "origin", "origin_text", 200);
    // add_text_marker(marker_array, -2.0, 2.0, 2.24, "omega1", "omega1_text", 201);
    
    // Add QR code markers
    add_qr_markers(marker_array);
    
    marker_pub_->publish(marker_array);
}

void MarkerPublisher::add_coordinate_system(visualization_msgs::msg::MarkerArray& marker_array, 
                         double x, double y, double z, 
                         const std::string& ns, int id_base)
{
    // X-axis (i) - Red
    tf2::Quaternion q_x;
    q_x.setRPY(0, 0, 0);

    auto x_axis = create_axis_marker(x, y, z, q_x.x(), q_x.y(), q_x.z(), q_x.w(), 
                                   1.0, 0.0, 0.0, 1.0, ns + "_x", id_base);
    marker_array.markers.push_back(x_axis);
    
    // Y-axis (j) - Green  
    tf2::Quaternion q_y;
    q_y.setRPY(0, 0, M_PI/2);

    auto y_axis = create_axis_marker(x, y, z, q_y.x(), q_y.y(), q_y.z(), q_y.w(),
                                   0.0, 1.0, 0.0, 1.0, ns + "_y", id_base + 1);
    marker_array.markers.push_back(y_axis);
    
    // Z-axis (k) - Blue
    tf2::Quaternion q_z;
    q_z.setRPY(0, -M_PI/2, 0);

    auto z_axis = create_axis_marker(x, y, z, q_z.x(), q_z.y(), q_z.z(), q_z.w(),
                                   0.0, 0.0, 1.0, 1.0, ns + "_z", id_base + 2);
    marker_array.markers.push_back(z_axis);
}

visualization_msgs::msg::Marker MarkerPublisher::create_axis_marker(double x, double y, double z,
                                                  double qx, double qy, double qz, double qw,
                                                  double r, double g, double b, double a,
                                                  const std::string& ns, int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;
    
    marker.scale.x = 2.0;  // Length
    marker.scale.y = 0.1;  // Width
    marker.scale.z = 0.1;  // Height
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    return marker;
}

void MarkerPublisher::add_text_marker(visualization_msgs::msg::MarkerArray& marker_array,
                    double x, double y, double z,
                    const std::string& text, const std::string& ns, int id)
{
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = this->get_clock()->now();
    text_marker.ns = ns;
    text_marker.id = id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    text_marker.pose.position.x = x;
    text_marker.pose.position.y = y;
    text_marker.pose.position.z = z;
    text_marker.pose.orientation.x = 0.0;
    text_marker.pose.orientation.y = 0.0;
    text_marker.pose.orientation.z = 0.0;
    text_marker.pose.orientation.w = 1.0;
    
    text_marker.scale.z = 0.5;  // Text height
    
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    text_marker.text = text;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    marker_array.markers.push_back(text_marker);
}

void MarkerPublisher::add_qr_markers(visualization_msgs::msg::MarkerArray& marker_array)
{
    // QR code positions from warehouse_outdoor.sdf
    struct QRCode {
        std::string name;
        double x, y, z;
    };
    
    std::vector<QRCode> qr_codes = {
        {"qr_shelfbig4_1", -4.9, -4.5, 2.2},
        {"qr_shelfbig4_2", -4.9, -5.4, 2.2},
        {"qr_shelfbig4_3", -4.88, -6.3, 2.2},
        {"qr_shelfbig4_4", -4.9, -7.2, 2.2},
        {"qr_shelfbig4_5", -4.9, -8.1, 2.2},
        {"qr_shelfbig4_6", -4.9, -5.4, 2.81},
        {"qr_shelfbig4_7", -4.9, -6.7, 2.81},
        {"qr_shelfbig4_8", -4.9, -7.9, 2.81}
    };
    
    for (size_t i = 0; i < qr_codes.size(); ++i) {
        auto qr_marker = create_qr_marker(qr_codes[i].x, qr_codes[i].y, qr_codes[i].z, 
                                         qr_codes[i].name, 300 + i);
        marker_array.markers.push_back(qr_marker);
    }
}

visualization_msgs::msg::Marker MarkerPublisher::create_qr_marker(double x, double y, double z,
                                                                  const std::string& /* name */, int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "qr_codes";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // QR code size (0.2x0.2 from SDF)
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    
    // White color like QR codes
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;  // Slightly transparent
    
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    return marker;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}