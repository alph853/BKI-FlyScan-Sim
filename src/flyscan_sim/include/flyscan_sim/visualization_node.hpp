/**
 * @file visualization_node.hpp
 * @brief Visualization node for coordinate systems, QR markers, and transform publishing
 * @author FlyScan Team
 * @date 2025
 */

#ifndef FLYSCAN_SIM__VISUALIZATION_NODE_HPP_
#define FLYSCAN_SIM__VISUALIZATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @class VisualizationNode
 * @brief Node for publishing visualization markers and transforms
 * 
 * This node publishes coordinate system markers, QR code positions,
 * and broadcasts transforms for visualization and debugging purposes.
 */
class VisualizationNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for VisualizationNode
     */
    VisualizationNode();

private:
    /**
     * @brief Publishes visualization markers periodically
     */
    void publish_markers();
    
    /**
     * @brief Adds coordinate system visualization to marker array
     * @param marker_array Reference to marker array to add to
     * @param x X position of coordinate system
     * @param y Y position of coordinate system
     * @param z Z position of coordinate system
     * @param ns Namespace for the markers
     * @param id_base Base ID for marker identification
     */
    void add_coordinate_system(visualization_msgs::msg::MarkerArray& marker_array, 
                             double x, double y, double z, 
                             const std::string& ns, int id_base);
    
    /**
     * @brief Creates an individual axis marker (arrow)
     * @param x X position
     * @param y Y position
     * @param z Z position
     * @param qx Quaternion x component
     * @param qy Quaternion y component
     * @param qz Quaternion z component
     * @param qw Quaternion w component
     * @param r Red color component
     * @param g Green color component
     * @param b Blue color component
     * @param a Alpha (transparency) component
     * @param ns Namespace for the marker
     * @param id Unique marker ID
     * @return Configured marker message
     */
    visualization_msgs::msg::Marker create_axis_marker(double x, double y, double z,
                                                      double qx, double qy, double qz, double qw,
                                                      double r, double g, double b, double a,
                                                      const std::string& ns, int id);
    
    /**
     * @brief Adds text marker to the marker array
     * @param marker_array Reference to marker array to add to
     * @param x X position of text
     * @param y Y position of text
     * @param z Z position of text
     * @param text Text content to display
     * @param ns Namespace for the marker
     * @param id Unique marker ID
     */
    void add_text_marker(visualization_msgs::msg::MarkerArray& marker_array,
                        double x, double y, double z,
                        const std::string& text, const std::string& ns, int id);
    
    /**
     * @brief Adds QR code markers to the marker array
     * @param marker_array Reference to marker array to add to
     */
    void add_qr_markers(visualization_msgs::msg::MarkerArray& marker_array);
    
    /**
     * @brief Creates a QR code marker
     * @param x X position of QR code
     * @param y Y position of QR code
     * @param z Z position of QR code
     * @param name Name identifier for the QR code
     * @param id Unique marker ID
     * @return Configured QR marker message
     */
    visualization_msgs::msg::Marker create_qr_marker(double x, double y, double z,
                                                     const std::string& name, int id);
    
    /**
     * @brief Callback for odometry messages to create path visualization
     * @param msg Odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /// Publisher for visualization marker arrays
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    /// Publisher for odometry path visualization
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    /// Subscription to odometry messages
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    /// Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;
    
    /// Path message for storing odometry history
    nav_msgs::msg::Path odom_path_;
    
    /// Maximum number of poses to keep in path history
    size_t max_path_length_;
};

#endif  // FLYSCAN_SIM__VISUALIZATION_NODE_HPP_