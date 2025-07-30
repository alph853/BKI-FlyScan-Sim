/**
 * @file video_streamer_minimal.hpp
 * @brief Minimal video streamer for debugging visualization
 * 
 * This file contains a minimal VideoStreamer class that subscribes to video
 * and calls the SemanticPerception service to display detection results.
 * 
 * @author FlyScan
 * @date 2025-07-30
 * @version 1.0
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "flyscan_interfaces/msg/detection_array.hpp"

namespace flyscan {
namespace sim {

/**
 * @brief Structure to hold object detection results for display
 */
struct DetectionDisplay {
    cv::Rect bbox;
    float confidence;
    int class_id;
    std::string class_name;
    std::string decoded_data;
};

/**
 * @brief Minimal video streamer for debugging visualization
 * 
 * Simple node that subscribes to video topic and displays frames with
 * detection overlays by calling the SemanticPerception service.
 */
class VideoStreamer : public rclcpp::Node
{
public:
    VideoStreamer();
    ~VideoStreamer();

    /**
     * @brief Main display loop for video streaming
     * @note This is a blocking call that runs until termination
     */
    void DisplayVideo();

private:
    /**
     * @brief Callback function for processing incoming video frames
     * @param msg Shared pointer to the incoming ROS2 Image message
     */
    void VideoCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief Add informational overlay to video frame
     * @param frame Input OpenCV frame to add overlay to
     * @return cv::Mat Frame with overlay information added
     */
    cv::Mat AddInfoOverlay(const cv::Mat& frame);
    
    /**
     * @brief Save current frame as screenshot
     */
    void SaveScreenshot();
    
    /**
     * @brief Draw detection bounding boxes and labels on frame
     * @param frame Input OpenCV frame to draw detections on
     * @param detections Vector of DetectionDisplay objects to visualize
     * @return cv::Mat Frame with detection visualizations drawn
     */
    cv::Mat DrawDetections(const cv::Mat& frame, const std::vector<DetectionDisplay>& detections);

    /**
     * @brief Callback function for processing incoming detection messages
     * @param msg Shared pointer to the incoming DetectionArray message
     */
    void DetectionCallback(const flyscan_interfaces::msg::DetectionArray::SharedPtr msg);

    /**
     * @brief Update FPS calculation
     */
    void UpdateFpsCalculation();

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_subscription_;
    rclcpp::Subscription<flyscan_interfaces::msg::DetectionArray>::SharedPtr detection_subscription_;
    
    // Video processing
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat current_frame_;
    int frame_count_;
    
    // FPS calculation
    std::chrono::steady_clock::time_point last_fps_time_;
    int fps_frame_count_;
    double current_fps_;
    
    // Display settings
    std::string window_name_;
    bool show_info_overlay_;
    
    // Detection storage
    std::vector<DetectionDisplay> current_detections_;
};

} // namespace perception
} // namespace sim