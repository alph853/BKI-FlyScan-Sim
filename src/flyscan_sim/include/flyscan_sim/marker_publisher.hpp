#ifndef FLYSCAN_SIM__MARKER_PUBLISHER_HPP_
#define FLYSCAN_SIM__MARKER_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>

class MarkerPublisher : public rclcpp::Node
{
public:
    MarkerPublisher();

private:
    void publish_markers();
    
    void add_coordinate_system(visualization_msgs::msg::MarkerArray& marker_array, 
                             double x, double y, double z, 
                             const std::string& ns, int id_base);
    
    visualization_msgs::msg::Marker create_axis_marker(double x, double y, double z,
                                                      double qx, double qy, double qz, double qw,
                                                      double r, double g, double b, double a,
                                                      const std::string& ns, int id);
    
    void add_text_marker(visualization_msgs::msg::MarkerArray& marker_array,
                        double x, double y, double z,
                        const std::string& text, const std::string& ns, int id);
    
    void add_qr_markers(visualization_msgs::msg::MarkerArray& marker_array);
    
    visualization_msgs::msg::Marker create_qr_marker(double x, double y, double z,
                                                     const std::string& name, int id);
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // FLYSCAN_SIM__MARKER_PUBLISHER_HPP_