#ifndef FLYSCAN_PERCEPTION_VIDEO_STREAMER_HPP_
#define FLYSCAN_PERCEPTION_VIDEO_STREAMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <memory>
#include <string>
#include <vector>

namespace flyscan_perception
{

struct Detection {
    cv::Rect bbox;
    float confidence;
    int class_id;
};

class VideoStreamer : public rclcpp::Node
{
public:
    VideoStreamer();
    ~VideoStreamer();
    
    void display_video();

private:
    void video_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat add_info_overlay(const cv::Mat& frame);
    void save_screenshot();
    void toggle_recording();
    
    // YOLO inference methods
    bool initialize_yolo_model(const std::string& model_path);
    std::vector<Detection> run_yolo_inference(const cv::Mat& frame);
    cv::Mat draw_detections(const cv::Mat& frame, const std::vector<Detection>& detections);
    cv::Mat preprocess_frame(const cv::Mat& frame);
    std::vector<Detection> postprocess_outputs(const std::vector<Ort::Value>& outputs, 
                                               float img_width, float img_height);
    
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_subscription_;
    
    // Video processing
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat current_frame_;
    int frame_count_;
    
    // Display settings
    std::string window_name_;
    bool show_info_overlay_;
    std::string video_topic_;
    
    // Performance optimization
    int inference_frame_skip_;
    int frame_skip_counter_;
    std::vector<Detection> last_detections_;
    
    // Recording
    bool recording_;
    cv::VideoWriter video_writer_;
    
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
    
    // Parameters
    static constexpr double SPIN_TIMEOUT_SEC = 0.01;
    static constexpr int WAITING_FRAME_WIDTH = 1280;
    static constexpr int WAITING_FRAME_HEIGHT = 720;
    static constexpr double FPS = 30.0;
    static constexpr int YOLO_INPUT_SIZE = 640;
    static constexpr float CONFIDENCE_THRESHOLD = 0.5f;
    static constexpr float NMS_THRESHOLD = 0.4f;
    static constexpr int INFERENCE_FRAME_SKIP = 3;  // Run inference every N frames
};

} // namespace flyscan_perception

#endif // FLYSCAN_PERCEPTION_VIDEO_STREAMER_HPP_