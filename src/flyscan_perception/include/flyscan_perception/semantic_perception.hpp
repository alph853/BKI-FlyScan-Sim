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
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <memory>
#include <string>
#include <vector>

#include "flyscan_core/base_node.hpp"
#include "flyscan_common/types.hpp"
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
    SemanticPerception();
    ~SemanticPerception();

protected:
    flyscan::common::OperationStatus HandleConfigure() override;
    flyscan::common::OperationStatus HandleActivate() override;
    flyscan::common::OperationStatus HandleDeactivate() override;

private:
    /**
     * @brief Callback function for processing incoming video frames
     * @param msg Shared pointer to the incoming ROS2 Image message
     */
    void VideoCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
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

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_subscription_;
    
    // Video processing
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat current_frame_;
    int frame_count_;
    
    // Performance optimization
    int inference_frame_skip_;
    int frame_skip_counter_;
    std::vector<Detection> last_detections_;
    
    // Publisher
    rclcpp::Publisher<flyscan_interfaces::msg::DetectionArray>::SharedPtr detection_publisher_;
    
    // YOLO model components
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
};

} // namespace perception
} // namespace flyscan