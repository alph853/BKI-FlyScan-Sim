/**
 * @file semantic_perception.hpp
 * @brief Semantic perception node for FlyScann system
 * 
 * This file contains the SemanticPerception class which provides object detection
 * capabilities with YOLOv8 for QR codes and barcodes.
 * 
 * @author FlyScan
 * @date 2025-07-30
 * @version 1.0
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "flyscan_core/base_node.hpp"
#include "flyscan_interfaces/msg/detection_array.hpp"

namespace flyscan {
namespace perception {

/**
 * @brief Structure to hold object detection results
 * 
 * Contains information about detected objects including bounding box,
 * confidence score, and class identification.
 */
struct Detection {
    cv::Rect bbox;
    float confidence;
    int class_id;
    std::string decoded_data;
    std::string decode_params;
    geometry_msgs::msg::Point position_3d;  // 3D position in camera frame
};

/**
 * @brief Semantic perception node for object detection
 * 
 * The SemanticPerception class provides YOLOv8 object detection capabilities.
 * It subscribes to ROS2 image topics, performs real-time object detection,
 * and provides a service for other nodes to request inference data.
 */
class SemanticPerception : public flyscan::core::BaseNode
{
public:
    explicit SemanticPerception(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
        const std::string& node_name = "semantic_perception",
        const flyscan::common::NodeType& node_type = flyscan::common::NodeType::kPerception,
        const std::vector<std::string>& capabilities = {"object_detection", "yolo_inference", "qr_decoding"}
    );
    
    ~SemanticPerception();

protected:
    // ============================================================================
    // Lifecycle Management (BaseNode overrides)
    // ============================================================================
    
    flyscan::common::OperationStatus HandleConfigure() override;
    flyscan::common::OperationStatus HandleActivate() override;
    flyscan::common::OperationStatus HandleDeactivate() override;
    flyscan::common::OperationStatus HandleCleanup() override;
    flyscan::common::OperationStatus HandleShutdown() override;
    flyscan::common::OperationStatus HandleError() override;

private:
    /**
     * @brief Callback function for processing incoming video frames
     * @param msg Shared pointer to the incoming ROS2 Image message
     */
    void VideoCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief Callback function for processing incoming depth images
     * @param msg Shared pointer to the incoming ROS2 depth Image message
     */
    void DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief Callback function for processing camera info
     * @param msg Shared pointer to the incoming ROS2 CameraInfo message
     */
    void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    
    /**
     * @brief Publish detection results to topic
     * @param detections Vector of detected objects
     */
    void PublishDetections(const std::vector<Detection>& detections);

    // YOLO inference methods
    /**
     * @brief Initialize YOLOv8 ONNX model for object detection
     * @param model_path Path to the YOLOv8 ONNX model file
     * @return bool True if model initialization successful, false otherwise
     */
    bool InitializeYoloModel(const std::string& model_path);
    
    /**
     * @brief Run YOLOv8 inference on video frame
     * @param frame Input OpenCV frame for object detection
     * @return std::vector<Detection> Vector of detected objects with bounding boxes and confidence scores
     */
    std::vector<Detection> RunYoloInference(const cv::Mat& frame);
    
    /**
     * @brief Preprocess frame for YOLOv8 model input
     * @param frame Input OpenCV frame to preprocess
     * @return cv::Mat Preprocessed frame ready for model inference
     */
    cv::Mat PreprocessFrame(const cv::Mat& frame);
    
    /**
     * @brief Process YOLOv8 model outputs to extract detections
     * 
     * @param outputs Vector of ONNX Value tensors from model inference
     * @param img_width Original image width for coordinate scaling
     * @param img_height Original image height for coordinate scaling
     * @return std::vector<Detection> Vector of filtered and processed detections
     */
    std::vector<Detection> PostprocessOutputs(const std::vector<Ort::Value>& outputs, 
                                               float img_width, float img_height);
    
    void ProcessQrDecoding(const cv::Mat& frame, std::vector<Detection>& detections);
    
    std::string DecodeQrFromRoi(const cv::Mat& roi);
    
    cv::Mat SharpenImage(const cv::Mat& image);
    
    cv::Mat DeskewQr(const cv::Mat& img);
    
    std::string TryDecodeVariant(const cv::Mat& img, float alpha, int beta, float clahe_clip, 
                                cv::Size clahe_grid, int angle);
    
    /**
     * @brief Localize QR code in 3D space using depth information
     * @param detection QR detection with bounding box
     * @param depth_image Depth image corresponding to the RGB frame
     * @param camera_info Camera intrinsic parameters
     * @return geometry_msgs::msg::PointStamped QR position in camera frame
     */
    geometry_msgs::msg::PointStamped LocalizeQrPosition(const Detection& detection,
                                                        const cv::Mat& depth_image,
                                                        const sensor_msgs::msg::CameraInfo& camera_info);
    
    /**
     * @brief Send HTTP POST request with QR code data (with deduplication)
     * @param qr_data Decoded QR code string
     * @param position 3D position of the QR code in camera frame
     */
    void SendQrHttpRequest(const std::string& qr_data, const geometry_msgs::msg::Point& position);
    
    /**
     * @brief Check if QR code is a duplicate based on position similarity
     * @param qr_data Decoded QR code string
     * @param position 3D position of the QR code
     * @return bool True if this is a new/different QR code
     */
    bool IsNewQrCode(const std::string& qr_data, const geometry_msgs::msg::Point& position);
    
    /**
     * @brief Callback for curl write operations
     */
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp);

    // ============================================================================
    // ROS2 Publishers and Subscribers
    // ============================================================================
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    
    rclcpp::Publisher<flyscan_interfaces::msg::DetectionArray>::SharedPtr detection_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr qr_position_publisher_;
    
    // ============================================================================
    // Image Processing and State Management
    // ============================================================================
    
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat current_frame_;
    cv::Mat current_depth_image_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    
    int frame_count_;
    int frame_skip_counter_;
    std::vector<Detection> last_detections_;
    
    // ============================================================================
    // Thread-Safe Data Access
    // ============================================================================
    
    std::mutex depth_mutex_;
    std::mutex camera_info_mutex_;
    
    // ============================================================================
    // TF2 Transform Components
    // ============================================================================
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ============================================================================
    // QR Code Deduplication
    // ============================================================================
    
    struct QrRecord {
        std::string data;
        geometry_msgs::msg::Point position;
        rclcpp::Time timestamp;
    };
    
    std::vector<QrRecord> processed_qr_codes_;
    std::mutex qr_records_mutex_;
    double qr_position_threshold_;  // Distance threshold for considering QR codes as same
    
    // ============================================================================
    // YOLO Model Components
    // ============================================================================
    
    std::unique_ptr<Ort::Session> ort_session_;
    std::unique_ptr<Ort::Env> ort_env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::vector<std::string> input_names_str_;
    std::vector<std::string> output_names_str_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<int64_t> input_shape_;
    bool yolo_initialized_;
    std::vector<std::string> class_names_;
    
    // ============================================================================
    // ROS Parameters (cached from parameter server)
    // ============================================================================
    
    int inference_frame_skip_;
    double confidence_threshold_;
    double nms_threshold_;
    bool gpu_enabled_;
    std::string camera_frame_;
};

} // namespace perception
} // namespace flyscan